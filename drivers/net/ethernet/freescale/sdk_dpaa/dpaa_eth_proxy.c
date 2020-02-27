/* Copyright 2008-2013 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifdef CONFIG_FSL_DPAA_ETH_DEBUG
#define pr_fmt(fmt) \
	KBUILD_MODNAME ": %s:%hu:%s() " fmt, \
	KBUILD_BASENAME".c", __LINE__, __func__
#else
#define pr_fmt(fmt) \
	KBUILD_MODNAME ": " fmt
#endif

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include "dpaa_eth.h"
#include "dpaa_eth_common.h"
#include "dpaa_eth_base.h"
#include "lnxwrp_fsl_fman.h" /* fm_get_rx_extra_headroom(), fm_get_max_frm() */
#include "mac.h"

#define DPA_DESCRIPTION "FSL DPAA Proxy initialization driver"

MODULE_LICENSE("Dual BSD/GPL");

MODULE_DESCRIPTION(DPA_DESCRIPTION);

static int __cold dpa_eth_proxy_remove(struct platform_device *of_dev);
#ifdef CONFIG_PM

static int proxy_suspend(struct device *dev)
{
	struct proxy_device *proxy_dev = dev_get_drvdata(dev);
	struct mac_device *mac_dev = proxy_dev->mac_dev;
	int err = 0;

	err = fm_port_suspend(mac_dev->port_dev[RX]);
	if (err)
		goto port_suspend_failed;

	err = fm_port_suspend(mac_dev->port_dev[TX]);
	if (err)
		err = fm_port_resume(mac_dev->port_dev[RX]);

port_suspend_failed:
	return err;
}

static int proxy_resume(struct device *dev)
{
	struct proxy_device *proxy_dev = dev_get_drvdata(dev);
	struct mac_device	*mac_dev = proxy_dev->mac_dev;
	int			err = 0;

	err = fm_port_resume(mac_dev->port_dev[TX]);
	if (err)
		goto port_resume_failed;

	err = fm_port_resume(mac_dev->port_dev[RX]);
	if (err)
		err = fm_port_suspend(mac_dev->port_dev[TX]);

port_resume_failed:
	return err;
}

static const struct dev_pm_ops proxy_pm_ops = {
	.suspend = proxy_suspend,
	.resume = proxy_resume,
};

#define PROXY_PM_OPS (&proxy_pm_ops)

#else /* CONFIG_PM */

#define PROXY_PM_OPS NULL

#endif /* CONFIG_PM */

static int dpaa_eth_proxy_probe(struct platform_device *_of_dev)
{
	int err = 0, i;
	struct device *dev;
	struct device_node *dpa_node;
	struct dpa_bp *dpa_bp;
	struct list_head proxy_fq_list;
	size_t count;
	struct fm_port_fqs port_fqs;
	struct dpa_buffer_layout_s *buf_layout = NULL;
	struct mac_device *mac_dev;
	struct proxy_device *proxy_dev;

	dev = &_of_dev->dev;

	dpa_node = dev->of_node;

	if (!of_device_is_available(dpa_node))
		return -ENODEV;

	/* Get the buffer pools assigned to this interface */
	dpa_bp = dpa_bp_probe(_of_dev, &count);
	if (IS_ERR(dpa_bp))
		return PTR_ERR(dpa_bp);

	mac_dev = dpa_mac_probe(_of_dev);
	if (IS_ERR(mac_dev))
		return PTR_ERR(mac_dev);

	proxy_dev = devm_kzalloc(dev, sizeof(*proxy_dev), GFP_KERNEL);
	if (!proxy_dev) {
		dev_err(dev, "devm_kzalloc() failed\n");
		return -ENOMEM;
	}

	proxy_dev->mac_dev = mac_dev;
	dev_set_drvdata(dev, proxy_dev);

	/* We have physical ports, so we need to establish
	 * the buffer layout.
	 */
	buf_layout = devm_kzalloc(dev, 2 * sizeof(*buf_layout),
				  GFP_KERNEL);
	if (!buf_layout) {
		dev_err(dev, "devm_kzalloc() failed\n");
		return -ENOMEM;
	}
	dpa_set_buffers_layout(mac_dev, buf_layout);

	INIT_LIST_HEAD(&proxy_fq_list);

	memset(&port_fqs, 0, sizeof(port_fqs));

	err = dpa_fq_probe_mac(dev, &proxy_fq_list, &port_fqs, true, RX);
	if (!err)
		err = dpa_fq_probe_mac(dev, &proxy_fq_list, &port_fqs, true,
				       TX);
	if (err < 0) {
		devm_kfree(dev, buf_layout);
		return err;
	}

	/* Proxy initializer - Just configures the MAC on behalf of
	 * another partition.
	 */
	dpaa_eth_init_ports(mac_dev, dpa_bp, count, &port_fqs,
			buf_layout, dev);

	/* Proxy interfaces need to be started, and the allocated
	 * memory freed
	 */
	devm_kfree(dev, buf_layout);
	devm_kfree(dev, dpa_bp);

	/* Free FQ structures */
	devm_kfree(dev, port_fqs.rx_defq);
	devm_kfree(dev, port_fqs.rx_errq);
	devm_kfree(dev, port_fqs.tx_defq);
	devm_kfree(dev, port_fqs.tx_errq);

	for_each_port_device(i, mac_dev->port_dev) {
		err = fm_port_enable(mac_dev->port_dev[i]);
		if (err)
			goto port_enable_fail;
	}

	dev_info(dev, "probed MAC device with MAC address: %02hx:%02hx:%02hx:%02hx:%02hx:%02hx\n",
		     mac_dev->addr[0], mac_dev->addr[1], mac_dev->addr[2],
		     mac_dev->addr[3], mac_dev->addr[4], mac_dev->addr[5]);

	return 0; /* Proxy interface initialization ended */

port_enable_fail:
	for_each_port_device(i, mac_dev->port_dev)
		fm_port_disable(mac_dev->port_dev[i]);
	dpa_eth_proxy_remove(_of_dev);

	return err;
}

int dpa_proxy_set_mac_address(struct proxy_device *proxy_dev,
			  struct net_device *net_dev)
{
	struct mac_device	*mac_dev;
	int			 _errno;

	mac_dev = proxy_dev->mac_dev;

	_errno = mac_dev->change_addr(mac_dev->get_mac_handle(mac_dev),
			net_dev->dev_addr);
	if (_errno < 0)
		return _errno;

	return 0;
}
EXPORT_SYMBOL(dpa_proxy_set_mac_address);

int dpa_proxy_set_rx_mode(struct proxy_device *proxy_dev,
		       struct net_device *net_dev)
{
	struct mac_device *mac_dev = proxy_dev->mac_dev;
	int _errno;

	if (!!(net_dev->flags & IFF_PROMISC) != mac_dev->promisc) {
		mac_dev->promisc = !mac_dev->promisc;
		_errno = mac_dev->set_promisc(mac_dev->get_mac_handle(mac_dev),
				mac_dev->promisc);
		if (unlikely(_errno < 0))
			netdev_err(net_dev, "mac_dev->set_promisc() = %d\n",
					_errno);
	}

	_errno = mac_dev->set_multi(net_dev, mac_dev);
	if (unlikely(_errno < 0))
		return _errno;

	return 0;
}
EXPORT_SYMBOL(dpa_proxy_set_rx_mode);

int dpa_proxy_start(struct net_device *net_dev)
{
	struct mac_device	*mac_dev;
	const struct dpa_priv_s	*priv;
	struct proxy_device	*proxy_dev;
	int			 _errno;
	int			i;

	priv = netdev_priv(net_dev);
	proxy_dev = (struct proxy_device *)priv->peer;
	mac_dev = proxy_dev->mac_dev;

	_errno = mac_dev->init_phy(net_dev, mac_dev);
	if (_errno < 0) {
		if (netif_msg_drv(priv))
			netdev_err(net_dev, "init_phy() = %d\n",
					_errno);
		return _errno;
	}

	for_each_port_device(i, mac_dev->port_dev) {
		_errno = fm_port_enable(mac_dev->port_dev[i]);
		if (_errno)
			goto port_enable_fail;
	}

	_errno = mac_dev->start(mac_dev);
	if (_errno < 0) {
		if (netif_msg_drv(priv))
			netdev_err(net_dev, "mac_dev->start() = %d\n",
					_errno);
		goto port_enable_fail;
	}

	return _errno;

port_enable_fail:
	for_each_port_device(i, mac_dev->port_dev)
		fm_port_disable(mac_dev->port_dev[i]);

	return _errno;
}
EXPORT_SYMBOL(dpa_proxy_start);

int dpa_proxy_stop(struct proxy_device *proxy_dev, struct net_device *net_dev)
{
	struct mac_device *mac_dev = proxy_dev->mac_dev;
	const struct dpa_priv_s	*priv = netdev_priv(net_dev);
	int _errno, i, err;

	_errno = mac_dev->stop(mac_dev);
	if (_errno < 0) {
		if (netif_msg_drv(priv))
			netdev_err(net_dev, "mac_dev->stop() = %d\n",
					_errno);
		return _errno;
	}

	for_each_port_device(i, mac_dev->port_dev) {
		err = fm_port_disable(mac_dev->port_dev[i]);
		_errno = err ? err : _errno;
	}

	if (mac_dev->phy_dev)
		phy_disconnect(mac_dev->phy_dev);
	mac_dev->phy_dev = NULL;

	return _errno;
}
EXPORT_SYMBOL(dpa_proxy_stop);

static int __cold dpa_eth_proxy_remove(struct platform_device *of_dev)
{
	struct device *dev = &of_dev->dev;
	struct proxy_device *proxy_dev = dev_get_drvdata(dev);

	kfree(proxy_dev);

	dev_set_drvdata(dev, NULL);

	return 0;
}

static const struct of_device_id dpa_proxy_match[] = {
	{
		.compatible	= "fsl,dpa-ethernet-init"
	},
	{}
};
MODULE_DEVICE_TABLE(of, dpa_proxy_match);

static struct platform_driver dpa_proxy_driver = {
	.driver = {
		.name		= KBUILD_MODNAME "-proxy",
		.of_match_table	= dpa_proxy_match,
		.owner		= THIS_MODULE,
		.pm		= PROXY_PM_OPS,
	},
	.probe		= dpaa_eth_proxy_probe,
	.remove		= dpa_eth_proxy_remove
};

static int __init __cold dpa_proxy_load(void)
{
	int	 _errno;

	pr_info(DPA_DESCRIPTION "\n");

	/* Initialize dpaa_eth mirror values */
	dpa_rx_extra_headroom = fm_get_rx_extra_headroom();
	dpa_max_frm = fm_get_max_frm();

	_errno = platform_driver_register(&dpa_proxy_driver);
	if (unlikely(_errno < 0)) {
		pr_err(KBUILD_MODNAME
			": %s:%hu:%s(): platform_driver_register() = %d\n",
			KBUILD_BASENAME".c", __LINE__, __func__, _errno);
	}

	pr_debug(KBUILD_MODNAME ": %s:%s() ->\n",
		KBUILD_BASENAME".c", __func__);

	return _errno;
}
module_init(dpa_proxy_load);

static void __exit __cold dpa_proxy_unload(void)
{
	platform_driver_unregister(&dpa_proxy_driver);

	pr_debug(KBUILD_MODNAME ": %s:%s() ->\n",
		KBUILD_BASENAME".c", __func__);
}
module_exit(dpa_proxy_unload);
