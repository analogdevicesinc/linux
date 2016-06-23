/*
 * MathWorks Streaming Channel
 *
 * Copyright 2016 The MathWorks, Inc
 *
 * Licensed under the GPL-2.
 */

#include <linux/idr.h>
#include <linux/version.h>

#include "mw_stream_channel.h"
#include "mathworks_ipcore.h"

#define STREAMDEV_TO_MWCHAN(dev_ptr)	(container_of(dev_ptr, struct mwadma_chan, dev))
#define MWDEV_TO_MWIP(mwdev)			(mwdev->mw_ipcore_dev->mw_ip_info)
#define IP2DEVP(mwdev)  (MWDEV_TO_MWIP(mwdev)->dev)
#define	IP2DEV(mwdev)	(*IP2DEVP(mwdev))

static DEFINE_IDA(mw_stream_channel_ida);
/*
 * Forward declaration of functions
 */
/*************************************************************************/
static int mw_axidma_setupchannel(struct mwadma_dev *mwdev,
        struct mwadma_chan *mwchan,
        struct mw_axidma_params *usrbuf);

static int mw_axidma_alloc(struct mwadma_dev *mwdev, size_t bufferSize);

static int mwadma_mmap(struct file *fp, struct vm_area_struct *vma);

static void mwadma_free_channel(struct mwadma_dev *mwdev,
        struct mwadma_chan *mwchan);

static void mwdma_test_loopback(struct mwadma_dev * mwdev,
        struct mw_axidma_params chn_prm);

int mwadma_start(struct mwadma_dev *mwdev,struct mwadma_chan *mwchan);

/*
 * @brief mwadma_fasync_impl
 */
static int mwadma_fasync_impl(int fd, struct file* fp, int mode)
{
    struct mwadma_dev *mwdev = fp->private_data;
    return fasync_helper(fd, fp, mode, &mwdev->asyncq);
}

/*
 * @brief mwadma_open
 */
static int mwadma_open(struct inode *inode, struct file *fp)
{
	struct mathworks_ip_info *mw_ip_info;
	struct mathworks_ipcore_dev *mw_ipcore_dev;
	struct mwadma_dev *mwdev;
    if (inode == NULL)
    {
        MW_DBG_text("INODE is NULL\n");
    }
    mw_ip_info = container_of(inode->i_cdev, struct mathworks_ip_info, cdev);
    mw_ipcore_dev = (struct mathworks_ipcore_dev *)mw_ip_info->private;
    mwdev = (struct mwadma_dev *)mw_ipcore_dev->private;
    fp->private_data = mwdev;

    return 0;
}

/*
 * @brief mwadma_allocate_desc
 */
static int mwadma_allocate_desc(struct mwadma_slist **new, struct mwadma_chan *mwchan, unsigned int idx)
{
    struct scatterlist *this_sg;
    struct mwadma_slist * tmp;
    void *sg_buff;
    int ret, i = 0;
    size_t ring_bytes;

    ring_bytes = mwchan->length/mwchan->ring_total;
    tmp = (struct mwadma_slist *)kmalloc(sizeof(struct mwadma_slist),GFP_KERNEL);
    tmp->status = BD_UNALLOC;
    tmp->sg_t = (struct sg_table *)kmalloc(sizeof(struct sg_table),GFP_KERNEL);
    if (tmp->sg_t == NULL) {
        pr_err("Error in sgtable KMALLOC\n");
        return -ENOMEM;
    }
    ret = sg_alloc_table(tmp->sg_t, mwchan->sg_entries, GFP_ATOMIC);
    if (ret) {
        pr_err("Error in sg_alloc_table\n");
        sg_free_table(tmp->sg_t);
        return -ENOMEM;
    }
    if (mwchan->buf == NULL) {
        tmp->buf = (char*)__get_free_pages(GFP_KERNEL|__GFP_ZERO, get_order(ring_bytes));
        pr_err("Channel buffer was null. This should never happen.\n");
    }
    else {
        /* set buffer at offset from larger buffer */
        tmp->buf = &mwchan->buf[idx * ring_bytes];
        tmp->buffer_index = idx;
    }


    for_each_sg(tmp->sg_t->sgl, this_sg, mwchan->sg_entries, i)
    {
        sg_buff = &(tmp->buf[(mwchan->bd_bytes)*i]);
        if (ring_bytes > mwchan->bd_bytes) {
            sg_set_buf(this_sg,sg_buff,mwchan->bd_bytes);
        } else {
            sg_set_buf(this_sg,sg_buff,ring_bytes);
        }
        ring_bytes -= mwchan->bd_bytes;
        if (ring_bytes < 0) {
            pr_err("Error occurred, SG entries do not match the size of one ring buffer\n");
        }
    }
    tmp->status = BD_ALLOC;
    *new = tmp;
    return 0;
}

/*
 * @brief mwadma_unmap_desc
 */
static void mwadma_unmap_desc(struct mwadma_dev *mwdev, struct mwadma_chan *mwchan, struct mwadma_slist *input_slist)
{
    dma_unmap_sg(IP2DEVP(mwdev), input_slist->sg_t->sgl, mwchan->sg_entries, mwchan->direction);
    input_slist->status = BD_UNALLOC;
}

/*
 * @brief mwadma_map_desc
 */
static int mwadma_map_desc(struct mwadma_dev *mwdev, struct mwadma_chan *mwchan, struct mwadma_slist *ipsl)
{
    int retVal;
    retVal = dma_map_sg(IP2DEVP(mwdev), ipsl->sg_t->sgl, mwchan->sg_entries, mwchan->direction);
    if (retVal == 0)  {
        dev_err(IP2DEVP(mwdev),"no buffers available\n");
        ipsl->status = BD_UNALLOC;
        return -ENOMEM;
    }
    ipsl->status = BD_MAPPED;
    return 0;
}

/*
 * @brief mwadma_prep_desc
 */
static int mwadma_prep_desc(struct mwadma_dev *mwdev, struct mwadma_chan * mwchan)
{
    unsigned int i = 0;
    int ret;
    struct mwadma_slist *new;

    /* First BD RING */
    ret = mwadma_allocate_desc(&(mwchan->scatter), mwchan, 0);
    if (ret < 0){
        dev_err(IP2DEVP(mwdev), "Failed in mwadma_allocate_desc");
        return -ENOMEM;
    }
    mwadma_map_desc(mwdev, mwchan, mwchan->scatter);
    /* New List of BD RING */
    INIT_LIST_HEAD(&(mwchan->scatter->list));
    for(i = 1; i < mwchan->ring_total; i++) /* POOL_SIZE - 1 */
    {
        ret = mwadma_allocate_desc(&(new), mwchan, i);
        if ((ret < 0) || (new == NULL)) {
            dev_err(IP2DEVP(mwdev), "Failed in mwadma_allocate_desc");
            return -ENOMEM;
        }
        list_add_tail(&(new->list),&(mwchan->scatter->list));
        mwadma_map_desc(mwdev, mwchan, new);
    }
    mwchan->curr = list_entry(mwchan->scatter->list.prev, struct mwadma_slist, list); /*Before first index (last index)*/
    mwchan->prev = list_entry(mwchan->curr->list.prev, struct mwadma_slist, list);
    return 0;
}

void mwadma_tx_cb_single_signal(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->tx;
    struct device *dev = IP2DEVP(mwdev);
    static int ct = 1;
    spin_lock_bh(&mwchan->slock);
    mwchan->transfer_count++;
    mwchan->status = ready;
    spin_unlock_bh(&mwchan->slock);

    /* Signal userspace */
    if (likely(mwdev->asyncq)) {
        kill_fasync(&mwdev->asyncq, SIGIO, POLL_OUT);
    }
    dev_dbg(dev, "Notify from %s : count:%d\n",__func__,ct++);
    sysfs_notify(&dev->kobj, NULL, "dma_ch2");
    sysfs_notify_dirent(mwchan->irq_kn);
}

void mwadma_tx_cb_continuous_signal_dataflow(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->tx;

    mwchan->transfer_queued--;
    MW_DBG_printf( ": Queue fill level = %ld\n",mwchan->transfer_queued);

    if(mwchan->transfer_queued > 0)
    {
        mwadma_start(mwdev,mwchan);

        if(mwchan->transfer_queued > TX_WATERMARK_QFULL) /* High watermark */
        {
            mwchan->error = TX_ERROR_QFULL;
            kill_fasync(&mwdev->asyncq, SIGIO, POLL_OUT);
        }
        else if(mwchan->transfer_queued > TX_WATERMARK_QPRIME) /* Normal */
        {
            mwchan->error = TX_ERROR_QPRIME;
        }
        else if(mwchan->transfer_queued >= TX_WATERMARK_QLOW) /* Low */
        {
            mwchan->error = TX_ERROR_QLOW;
            kill_fasync(&mwdev->asyncq, SIGIO, POLL_OUT);
        }

    }
    else /* Underflow */
    {
        mwchan->error = TX_ERROR_QUNDERFLOW;
        kill_fasync(&mwdev->asyncq, SIGIO, POLL_OUT);
        mwchan->status = waiting;
        MW_DBG_text( ": Underflow condition\n");
    }
    mwchan->transfer_count++;
}


void mwadma_tx_cb_continuous_signal(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->tx;
    mwchan->transfer_queued--;
    mwadma_start(mwdev,mwchan);
    if (likely(mwdev->asyncq))
    {
        kill_fasync(&mwdev->asyncq, SIGIO, POLL_OUT);
    }
    mwchan->transfer_count++;
}


void mwadma_rx_cb_single_signal(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->rx;
    struct device *dev = IP2DEVP(mwdev);
    static int ct = 1;

    spin_lock_bh(&mwchan->slock);
    mwchan->transfer_count++;
    mwchan->completed = mwchan->prev;
    mwchan->next_index = mwchan->completed->buffer_index;
    mwchan->status = ready;
    spin_unlock_bh(&mwchan->slock);

    mwchan->completed = mwchan->prev;
    /* Signal userspace */
    if (likely(mwdev->asyncq))
    {
        kill_fasync(&mwdev->asyncq, SIGIO, POLL_IN);
    }
    dev_dbg(dev, "Notify from %s : count:%d\n",__func__,ct++);
    sysfs_notify(&dev->kobj, NULL, "dma_ch1");
    sysfs_notify_dirent(mwchan->irq_kn);
}

void mwadma_rx_cb_burst(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->rx;
    mwchan->completed = mwchan->prev;
    mwchan->transfer_queued--;
    if (mwchan->transfer_queued)
    {
        mwadma_start(mwdev,mwchan);
    }
    else
    {
        if (likely(mwdev->asyncq))
        {
            kill_fasync(&mwdev->asyncq, SIGIO, POLL_IN);
        }
        else
        {
            mwchan->next_index = mwchan->prev->buffer_index;
        }
    }
    /*MW_DBG_printf( "Completed buffer index = %d\n",mwchan->completed->buffer_index);*/
    mwchan->transfer_count++;
#ifdef DEBUG_IN_RATE
    mwchan->stop = ktime_get();
#endif
}

void mwadma_rx_cb_continuous_signal(struct mwadma_dev *mwdev)
{
    struct mwadma_chan *mwchan = mwdev->rx;
    struct device *dev = IP2DEVP(mwdev);
    long int current_transfers_completed, current_transfers_queued;

    if(unlikely(mwchan->status == ready)) {
        dev_err(dev, "Channel is busy.\n");
        return;
    }
    dev_dbg(dev, "Continuous-mode callback---\n");
    spin_lock_bh(&mwchan->slock);
    mwchan->completed = list_entry(mwchan->prev->list.prev,struct mwadma_slist,list);
    mwchan->transfer_count++;
    mwchan->transfer_queued++;
    current_transfers_completed = mwchan->transfer_count;
    current_transfers_queued = mwchan->transfer_queued;
    spin_unlock_bh(&mwchan->slock);
    mwadma_start(mwdev, mwchan);
    if(current_transfers_completed == 1) {
        spin_lock_bh(&mwchan->slock);
        mwchan->next_index = mwchan->completed->buffer_index;
        spin_unlock_bh(&mwchan->slock);
    }
    else if(current_transfers_queued > mwchan->ring_total) /* The completed transfer overflowed the buffer */ {
        mwchan->error = ERR_RING_OVERFLOW;
        dev_dbg(dev, "Overflow condition:%s at %d\n", __func__, __LINE__);
        spin_lock_bh(&mwchan->slock);
        mwchan->transfer_queued = 0;
        /* We should start processing data from the completed transfer index
         * Might not be required, since application is going to process this
         * ring anyway. Just making it explicit
         */
        mwchan->next_index = mwchan->completed->buffer_index;
        spin_unlock_bh(&mwchan->slock);
    }
    if(likely(mwdev->asyncq)) {
        /* Signal userspace */
        kill_fasync(&mwdev->asyncq, SIGIO, POLL_IN);
    }
    dev_dbg(dev, "Notify from %s : count:%ld\n",__func__,current_transfers_completed);
    sysfs_notify(&dev->kobj, NULL, "dma_ch1");
    sysfs_notify_dirent(mwchan->irq_kn);
}

/*
 * @brief mwadma_start
 */
int mwadma_start(struct mwadma_dev *mwdev,struct mwadma_chan *mwchan)
{
    int ret = 0;
    unsigned int nents;
    struct mwadma_slist *newList;
    struct dma_async_tx_descriptor *thisDesc;
    struct sg_table *thisSgt;
    struct dma_chan *chan;

    dev_dbg(IP2DEVP(mwdev),"In %s\n",__func__);
    if((mwdev == NULL) || (NULL == mwchan)) {
        pr_err("mw-axidma: Received null pointer in client driver or channel structure.\n");
        ret = -ENODEV;
        goto start_failed;
    }
    chan     = mwchan->chan;
    thisSgt  = mwchan->curr->sg_t;
    nents    = mwchan->sg_entries;
    thisDesc = dmaengine_prep_slave_sg(chan, thisSgt->sgl, nents, mwchan->direction, mwchan->flags);
    if (NULL == thisDesc) {
        mwadma_unmap_desc(mwdev, mwchan, mwchan->curr);
        dev_err(IP2DEVP(mwdev), "Unable to prepare scatter-gather descriptor.\n");
        goto start_failed;
    }
    thisDesc->callback = mwchan->callback;
    thisDesc->callback_param = mwdev;
    mwchan->curr->desc = thisDesc;
    dmaengine_submit(mwchan->curr->desc);
    if (dma_submit_error(mwchan->curr->desc->cookie)) {
        dev_err(IP2DEVP(mwdev), "Failure in dmaengine_submit.\n");
        ret = -ENOSYS;
        goto start_failed;
    }
    spin_lock_bh(&mwchan->slock);
    newList = list_entry(mwchan->curr->list.next,struct mwadma_slist,list);
    mwchan->prev = mwchan->curr;
    mwchan->curr = newList;
    if (mwchan->direction == DMA_MEM_TO_DEV){
        mwchan->transfer_queued--;
    }
    spin_unlock_bh(&mwchan->slock);
    return ret;

start_failed:
    return ret;
}

/*
 * @brief mwadma_stop
 */
static int mwadma_stop(struct mwadma_dev *mwdev, struct mwadma_chan *mwchan)
{
    struct xilinx_dma_config config;

    config.coalesc = 0;
    config.delay = 0;
    #if LINUX_VERSION_CODE <= KERNEL_VERSION(3,18,00)
    dmaengine_device_control(mwchan->chan, DMA_TERMINATE_ALL, (unsigned long)&config);
    #else
    dmaengine_terminate_all(mwchan->chan);
    #endif
    dev_dbg(IP2DEVP(mwdev),"DMA STOP\nIterations = %lu\n",mwchan->transfer_count);
    return 0;
}

/*
 * @brief mwadma_rx_ctl
 */
static long mwadma_rx_ctl(struct mwadma_dev *mwdev, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    unsigned long userval;
    struct mw_axidma_params usrbuf;
    struct mwadma_chan *mwchan = mwdev->rx;
    enum mwadma_chan_status     status;
    unsigned int                next_index;
    unsigned int                error;
    switch(cmd)
    {
        case MWADMA_SETUP_RX_CHANNEL:
            if(copy_from_user(&usrbuf, (struct mw_axidma_params *)arg, sizeof(struct mw_axidma_params)))
            {
                return -EACCES;
            }
            if (mwchan == NULL)
            {
                dev_err(IP2DEVP(mwdev),"Invalid Memory\n");
                return -ENOMEM;
            }
            ret = mw_axidma_setupchannel(mwdev, mwchan, &usrbuf);
            break;
        case MWADMA_RX_SINGLE:
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof(userval)))
            {
                return -EACCES;
            }
            switch(userval)
            {
                case SIGNAL_TRANSFER_COMPLETE:
                    mwchan->callback = (dma_async_tx_callback)mwadma_rx_cb_single_signal;
                    break;
                default:
                    mwchan->callback = (dma_async_tx_callback)mwadma_rx_cb_continuous_signal;
            }
            spin_lock_bh(&mwchan->slock);
            mwchan->error = 0;
            mwchan->transfer_count = 0;
            spin_unlock_bh(&mwchan->slock);
            mwadma_start(mwdev, mwchan);
            dma_async_issue_pending(mwchan->chan);
            break;
        case MWADMA_RX_BURST:
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof(userval)))
            {
                return -EACCES;
            }
            mwchan->callback = (dma_async_tx_callback)mwadma_rx_cb_burst;
            /* Start from the first */
            if(userval > mwchan->ring_total)
            {
                return -EINVAL;
            }

            spin_lock_bh(&mwchan->slock);
            mwchan->transfer_queued = userval;
            mwchan->transfer_count = 0;
            mwchan->error = 0;
            spin_unlock_bh(&mwchan->slock);

            dev_dbg(IP2DEVP(mwdev), "Start DMA Burst of size %lu\n", userval);

            mwadma_stop(mwdev, mwchan);
            mwadma_start(mwdev,mwchan);
            spin_lock_bh(&mwchan->slock);
            mwchan->status = running;
            spin_unlock_bh(&mwchan->slock);
            dma_async_issue_pending(mwchan->chan);
            break;
        case MWADMA_RX_CONTINUOUS:
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof(userval))) {
                return -EACCES;
            }
            switch(userval) {
                case SIGNAL_TRANSFER_COMPLETE:
                    mwchan->callback = (dma_async_tx_callback)mwadma_rx_cb_continuous_signal;
                    break;
                default:
                    mwchan->callback = (dma_async_tx_callback)mwadma_rx_cb_continuous_signal;
            }
            dev_dbg(IP2DEVP(mwdev),"Start Continuous Rx DMA\n");

            spin_lock_bh(&mwchan->slock);
            mwchan->transfer_queued = 0;
            mwchan->transfer_count = 0;
            mwchan->error = 0;
            spin_unlock_bh(&mwchan->slock);

            mwadma_start(mwdev,mwchan);
            dma_async_issue_pending(mwchan->chan);
            spin_lock_bh(&mwchan->slock);/*!!!LOCK!!!*/
            mwchan->status = running;
            spin_unlock_bh(&mwchan->slock);/*!!!UNLOCK!!!*/
            mwadma_start(mwdev,mwchan);
            break;
        case MWADMA_RX_STOP:
            spin_lock_bh(&mwchan->slock);
            status = (unsigned long) mwchan->status;
            spin_unlock_bh(&mwchan->slock);
            memset(mwchan->buf, 0, mwchan->length);

            if(status != ready)
            {
                ret = mwadma_stop(mwdev,mwchan);
                if (ret) {
                    dev_err(IP2DEVP(mwdev),"Error while stopping DMA\n");
                    return ret;
                }
            }
            spin_lock_bh(&mwchan->slock);
            mwchan->transfer_count = 0;
            mwchan->status = ready;
            spin_unlock_bh(&mwchan->slock);
            break;
        case MWADMA_RX_GET_NEXT_INDEX:
            spin_lock_bh(&mwchan->slock);
            next_index = (unsigned long) mwchan->next_index;
            mwchan->transfer_queued--; /* user space has consumed a signal */
            mwchan->next_index = (mwchan->next_index + 1) % mwchan->buffer_interrupts;
            spin_unlock_bh(&mwchan->slock);
            if(copy_to_user((unsigned long *) arg, &next_index, sizeof(unsigned long))) {
                return -EACCES;
            }
            break;
        case MWADMA_RX_GET_ERROR:

            spin_lock_bh(&mwchan->slock);
            error = mwchan->error;
            mwchan->error = 0;
            spin_unlock_bh(&mwchan->slock);

            if(copy_to_user((unsigned long *) arg, &error, sizeof(unsigned long)))
            {
                return -EACCES;
            }
            break;
        case MWADMA_FREE_RX_CHANNEL:
            mwadma_free_channel(mwdev, mwchan);
            break;
        default:
            return 1;
    }
    return 0;
}


/*
 * @brief mwadma_tx_ctl
 */
static long mwadma_tx_ctl(struct mwadma_dev *mwdev, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    struct mwadma_chan *mwchan = mwdev->tx;
    unsigned long userval;
    enum mwadma_chan_status status;
    long int transfer_queued;
    struct mw_axidma_params usrbuf;
    struct mwadma_slist *new;
    switch(cmd)
    {
        case MWADMA_SETUP_TX_CHANNEL:
            if(copy_from_user(&usrbuf, (struct mw_axidma_params *)arg, sizeof(struct mw_axidma_params)))
            {
                return -EACCES;
            }
            if (mwchan == NULL)
            {
                return -ENOMEM;
            }
            ret = mw_axidma_setupchannel(mwdev, mwchan, &usrbuf);
            break;

        case MWADMA_TX_ENQUEUE:
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof(userval))) {
                return -EACCES;
            }
            spin_lock_bh(&mwchan->slock);
            transfer_queued = mwchan->transfer_queued;
            status = mwchan->status;
            spin_unlock_bh(&mwchan->slock);
            if((status == ready) && (0 == transfer_queued )) {
                spin_lock_bh(&mwchan->slock);
                mwchan->next_index =  mwchan->curr->buffer_index;
                spin_unlock_bh(&mwchan->slock);
            }
            else
            {
                mwchan->next_index = (mwchan->next_index + 1) % mwchan->ring_total;
            }
            if(transfer_queued >= mwchan->ring_total) {
                dev_err(IP2DEVP(mwdev), \
                        ":queue:%lu, user-queue:%lu, ring:%u\n", \
                        mwchan->transfer_queued, \
                        userval, \
                        mwchan->ring_total);
                spin_lock_bh(&mwchan->slock);
                mwchan->error = TX_ERROR_QFULL;
                spin_unlock_bh(&mwchan->slock);
                return 0;
            }
            spin_lock_bh(&mwchan->slock);
            mwchan->transfer_queued += (long)userval;
            transfer_queued = mwchan->transfer_queued;
            spin_unlock_bh(&mwchan->slock);
            if(unlikely((status == waiting) && (transfer_queued >= TX_WATERMARK_QPRIME))) /* restart if required */
            {
                dev_dbg(IP2DEVP(mwdev),"Fill level reached = %ld\n", transfer_queued);
                mwadma_start(mwdev,mwchan);
                spin_lock_bh(&mwchan->slock);
                dma_async_issue_pending(mwchan->chan);
                mwchan->status = running; /*Data ready */
                spin_unlock_bh(&mwchan->slock);
                mwadma_start(mwdev,mwchan);
            }
            break;
        case MWADMA_TX_SINGLE:
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof(userval)))  {
                return -EACCES;
            }
            switch(userval)
            {
                case SIGNAL_TRANSFER_COMPLETE:
                    mwchan->callback = (dma_async_tx_callback)mwadma_tx_cb_single_signal;
                    break;
                default:
                    mwchan->callback = (dma_async_tx_callback)mwadma_tx_cb_single_signal;
            }
            spin_lock_bh(&mwchan->slock);
            transfer_queued = mwchan->transfer_queued;
            spin_unlock_bh(&mwchan->slock);
            if (!transfer_queued) {
                dev_err(IP2DEVP(mwdev),"Queue is empty\n");
                return -EINVAL;
            }
            spin_lock_bh(&mwchan->slock);
            mwchan->next_index = (mwchan->next_index + 1) % mwchan->ring_total;
            spin_unlock_bh(&mwchan->slock);
            mwadma_start(mwdev,mwchan);
            dma_async_issue_pending(mwchan->chan);
            break;
        case MWADMA_TX_CONTINUOUS:
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof(userval)))
            {
                return -EACCES;
            }
            spin_lock(&mwchan->slock);
            switch(userval)
            {
                case SIGNAL_TRANSFER_COMPLETE:
                    mwchan->callback = (dma_async_tx_callback)mwadma_tx_cb_continuous_signal;
                    break;
                case SIGNAL_DATAFLOW:
                    mwchan->callback = (dma_async_tx_callback)mwadma_tx_cb_continuous_signal_dataflow;
                    break;
                default:
                    mwchan->callback = (dma_async_tx_callback)mwadma_tx_cb_continuous_signal;
            }
            mwchan->status = waiting; /* Wait on queued data */
            mwchan->next_index = (mwchan->next_index + 1) % mwchan->ring_total;
            spin_unlock(&mwchan->slock);/*!!!UNLOCK!!!*/
            break;
        case MWADMA_TX_STOP:
            spin_lock(&mwchan->slock);/*!!!LOCK!!!*/
            if(mwchan->status == running)
            {

                ret = mwadma_stop(mwdev,mwchan);
                if (ret)
                {
                    dev_err(IP2DEVP(mwdev),"Error while stopping DMA\n");
                    spin_unlock(&mwchan->slock);/*!!!UNLOCK-EXIT!!!*/
                    return ret;
                }
                mwchan->status = ready;
            }
            mwchan->transfer_queued = 0; /* Reset pending transfers */
            spin_unlock(&mwchan->slock);
            break;
        case MWADMA_TX_GET_ERROR:
            dev_dbg(IP2DEVP(mwdev), "Requested Tx error status = %d\n",mwchan->error);

            spin_lock_bh(&mwchan->slock);/*!!!LOCK!!!*/
            userval = mwchan->error; /* error code */
            /*mwchan->error = 0;*/
            spin_unlock_bh(&mwchan->slock);/*!!!UNLOCK!!!*/

            if(copy_to_user((unsigned long *) arg, &userval, sizeof(unsigned long)))
            {
                return -EACCES;
            }
            break;
        case MWADMA_TX_GET_NEXT_INDEX:
            spin_lock_bh(&mwchan->slock);/*!!!LOCK!!!*/
            new = list_entry(mwchan->curr->list.next,struct mwadma_slist,list);
            userval = (unsigned long) new->buffer_index;
            /*dev_info(IP2DEVP(mwdev), "Next index = %d\n", userval);*/
            spin_unlock_bh(&mwchan->slock);/*!!!UNLOCK!!!*/
            if(copy_to_user((unsigned long *) arg, &userval, sizeof(unsigned long)))
            {
                return -EACCES;
            }
            break;
        case MWADMA_FREE_TX_CHANNEL:
            mwadma_free_channel(mwdev, mwchan);
            break;
        default:
            return 1;
    }
    return 0;
}

/*
 * @brief mwadma_generic_ctl
 */
static long mwadma_generic_ctl(struct mwadma_dev *mwdev, unsigned int cmd, unsigned long arg)
{
    struct mw_axidma_params usrbuf;
    switch(cmd)
    {
        case MWADMA_GET_PROPERTIES:
            usrbuf.size = MWDEV_TO_MWIP(mwdev)->dma_info.size;
            usrbuf.phys = (dma_addr_t)MWDEV_TO_MWIP(mwdev)->dma_info.phys;
            if(copy_to_user((struct mw_axidma_params *)arg, &usrbuf, sizeof(struct mw_axidma_params))) {
                return -EACCES;
            }
            break;
        case MWADMA_TEST_LOOPBACK:
            if(copy_from_user(&usrbuf, (struct mw_axidma_params *)arg, sizeof(struct mw_axidma_params)))
            {
                return -EACCES;
            }
            mwdma_test_loopback(mwdev, usrbuf);
            break;
        default:
            return 1;
    }
    return 0;
}

/*
 * @brief mwadma_ioctl
 */
static long mwadma_ioctl(struct file *fp, unsigned int cmd, unsigned long arg)
{
    int ret_rx = 0;
    int ret_tx = 0;
    int ret_generic = 0;
    struct mwadma_dev *mwdev = fp->private_data;

    if (NULL == mwdev)
    {
        return -ENODEV;
    }

    ret_rx = mwadma_rx_ctl(mwdev,cmd,arg);
    ret_tx = mwadma_tx_ctl(mwdev,cmd,arg);
    ret_generic = mwadma_generic_ctl(mwdev,cmd,arg);
    /* Errors */
    if(ret_rx < 0)
    {
        return ret_rx;
    }
    if(ret_tx < 0)
    {
        return ret_tx;
    }
    if(ret_generic < 0)
    {
        return ret_generic;
    }

    /* No valid case found */
    if(3 == (ret_rx + ret_tx + ret_generic))
    {
        dev_dbg(IP2DEVP(mwdev), "Invalid ioctl: command: %u\n", cmd);
        return -EINVAL;
    }
    return 0;
}


/*
 * @brief mwadma_close
 */
static int mwadma_close(struct inode *inode, struct file *fp)
{
    struct mwadma_dev *mwdev = fp->private_data;
    int ret = 0;

    if (NULL == mwdev) {
        return -ENODEV;
    }
    dev_dbg(IP2DEVP(mwdev),"Closing the file-descriptor\n");
    mwadma_fasync_impl(-1, fp, 0);
    return ret;
}

/*
 * @brief mwadma_mmap_dma_open
 */
static void mwadma_mmap_dma_open(struct vm_area_struct *vma)
{
    struct mwadma_dev * mwdev = vma->vm_private_data;
	dev_info(IP2DEVP(mwdev), "DMA VMA open, virt %lx, phys %lx \n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);

}

/*
 * @brief mwadma_free_channel
 */
static void mwadma_free_channel(struct mwadma_dev *mwdev, struct mwadma_chan *mwchan)
{
    struct mwadma_slist *slist, *_slist;
    #if LINUX_VERSION_CODE <= KERNEL_VERSION(3,18,00)
    struct xilinx_dma_config config;
    #endif
    unsigned long flags;

    spin_lock_irqsave(&mwchan->slock, flags);
    list_for_each_entry_safe(slist, _slist, &mwchan->scatter->list, list) {
        mwadma_unmap_desc(mwdev, mwchan, slist);
        sg_free_table(slist->sg_t);
        kfree(slist->sg_t);
        list_del(&slist->list);
        kfree(&slist->list);
    }
    spin_unlock_irqrestore(&mwchan->slock, flags);
    #if LINUX_VERSION_CODE <= KERNEL_VERSION(3,18,00)
    dmaengine_device_control(mwchan->chan, DMA_TERMINATE_ALL, (struct xilinx_dma_config *)&config);
    #else
    dmaengine_terminate_all(mwchan->chan);
    #endif
    dev_dbg(IP2DEVP(mwdev), "MWADMA Free channel done.");
}

/*
 * @brief mwadma_mmap_dma_close
 */
static void mwadma_mmap_dma_close(struct vm_area_struct *vma)
{
    struct mwadma_dev * mwdev = vma->vm_private_data;
	dev_info(IP2DEVP(mwdev), "DMA VMA close.\n");
	/* Free the memory DMA */
    if (MWDEV_TO_MWIP(mwdev)->dma_info.size) {
        dev_info(IP2DEVP(mwdev), "free dma memory.\n");
        dmam_free_coherent(IP2DEVP(mwdev), MWDEV_TO_MWIP(mwdev)->dma_info.size, MWDEV_TO_MWIP(mwdev)->dma_info.virt, MWDEV_TO_MWIP(mwdev)->dma_info.phys);
        MWDEV_TO_MWIP(mwdev)->dma_info.size = 0;
        mwdev->channel_offset = 0;
        MWDEV_TO_MWIP(mwdev)->dma_info.virt = NULL;
        MWDEV_TO_MWIP(mwdev)->dma_info.phys = 0;
    }
}

/*
 * @brief mwadma_mmap_open
 */
static void mwadma_mmap_open(struct vm_area_struct *vma)
{
    struct mwadma_dev * mwdev = vma->vm_private_data;
	dev_info(IP2DEVP(mwdev), "Simple VMA open, virt %lx, phys %lx \n", vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

/*
 * @brief mwadma_mmap_close
 */
static void mwadma_mmap_close(struct vm_area_struct *vma)
{
    struct mwadma_dev * mwdev = vma->vm_private_data;
	dev_info(&IP2DEV(mwdev ), "Simple VMA close.\n");
}

/*
 * @brief mwadma_mmap_fault
 */
static int mwadma_mmap_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
    struct mwadma_dev * mwdev = vma->vm_private_data;
    struct page *thisPage;
    unsigned long offset;
    offset = (vmf->pgoff - vma->vm_pgoff) << PAGE_SHIFT;
    thisPage = virt_to_page(MWDEV_TO_MWIP(mwdev)->mem->start + offset);
    get_page(thisPage);
    vmf->page = thisPage;
    return 0;
}

struct vm_operations_struct  mwadma_mmap_ops = {
    .open           = mwadma_mmap_open,
    .close          = mwadma_mmap_close,
    .fault          = mwadma_mmap_fault,
};

struct vm_operations_struct mwadma_mmap_dma_ops = {
    .open           = mwadma_mmap_dma_open,
    .close          = mwadma_mmap_dma_close,
};


struct file_operations mwadma_cdev_fops = {
    .owner          = THIS_MODULE,
    .open           = mwadma_open,
    .fasync         = mwadma_fasync_impl,
    .release        = mwadma_close,
    .mmap		    = mwadma_mmap,
    .unlocked_ioctl = mwadma_ioctl,
};

/*
 * @brief mwadma_mmap
 */
static int mwadma_mmap(struct file *fp, struct vm_area_struct *vma)
{
    struct mwadma_dev *mwdev = fp->private_data;
    size_t size = vma->vm_end - vma->vm_start;
    int status = 0;
    vma->vm_private_data = mwdev;
    dev_info(IP2DEVP(mwdev), "[MMAP] size:%X pgoff: %lx\n", (unsigned int)size, vma->vm_pgoff);

    switch(vma->vm_pgoff) {
		case 0:
			/* mmap the Memory Mapped I/O's base address */
                        vma->vm_flags |= VM_IO | VM_DONTDUMP;
			vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
			if (remap_pfn_range(vma, vma->vm_start,
					MWDEV_TO_MWIP(mwdev)->mem->start >> PAGE_SHIFT,
					size,
					vma->vm_page_prot))
			{
				return -EAGAIN;
			}
			vma->vm_ops = &mwadma_mmap_ops;
			break;
		default:
                      /* mmap the DMA region */
                      status = mw_axidma_alloc(mwdev, size);
                     if ((status) && (status != -EEXIST))  {
                         return -ENOMEM;
                     }
                    dev_dbg(IP2DEVP(mwdev), "dma setup_cdev successful\n");

                    status = 0;
			if (MWDEV_TO_MWIP(mwdev)->dma_info.virt == NULL){
                            return -EINVAL;
                        }
                    vma->vm_pgoff = 0;
                    status = dma_mmap_coherent(IP2DEVP(mwdev), vma, MWDEV_TO_MWIP(mwdev)->dma_info.virt,
                        MWDEV_TO_MWIP(mwdev)->dma_info.phys, MWDEV_TO_MWIP(mwdev)->dma_info.size);
                    if (status) {
                        dev_dbg(IP2DEVP(mwdev),"Remapping memory failed, error: %d\n", status);
                        return status;
                    }
                    vma->vm_ops = &mwadma_mmap_dma_ops;
                    dev_dbg(IP2DEVP(mwdev),"%s: mapped dma addr 0x%08lx at 0x%08lx, size %u\n",
                          __func__, (unsigned long)MWDEV_TO_MWIP(mwdev)->dma_info.phys, vma->vm_start,
                          (unsigned int)MWDEV_TO_MWIP(mwdev)->dma_info.size);
                     break;
    }
	return status;
}


/*
 * @brief mw_axidma_alloc
 */
static int mw_axidma_alloc(struct mwadma_dev *mwdev, size_t bufferSize)
{
    if (mwdev == NULL)
    {
        return -ENOMEM;
    }
    if (MWDEV_TO_MWIP(mwdev)->dma_info.virt != NULL)
    {
		dev_err(IP2DEVP(mwdev), "DMA memory already allocated\n");
		return -EEXIST;
	}
    MWDEV_TO_MWIP(mwdev)->dma_info.virt = dmam_alloc_coherent(IP2DEVP(mwdev), bufferSize, \
            &MWDEV_TO_MWIP(mwdev)->dma_info.phys, \
            GFP_KERNEL);
    if (MWDEV_TO_MWIP(mwdev)->dma_info.virt == NULL)
    {
        dev_err(IP2DEVP(mwdev), "Failed to allocate continguous memory\nUsing multiple buffers\n");
    }

    else {
        dev_info(IP2DEVP(mwdev), "Address of buffer = 0x%p, Length = %u Bytes\n",\
                (void *)virt_to_phys(MWDEV_TO_MWIP(mwdev)->dma_info.virt),(unsigned int)bufferSize);
        MWDEV_TO_MWIP(mwdev)->dma_info.size = bufferSize;
    }
    return 0;
}

/*
 * @brief mw_axidma_setupchannel
 */
static int mw_axidma_setupchannel(struct mwadma_dev *mwdev,
        struct mwadma_chan *mwchan,
        struct mw_axidma_params *usrbuf)
{
    int status = 0;
    static int idx = 0;
    char *buf;
    struct xilinx_dma_config    config;
    if ( (mwdev == NULL) || (mwchan == NULL) ) {
        return -EINVAL;
    }
    mwchan->flags               = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
    mwchan->ring_total          = usrbuf->total_rings;
    mwchan->length              = usrbuf->bytes_per_ring * usrbuf->total_rings;

    mwchan->bd_bytes            = usrbuf->desc_length;
    mwchan->buffer_interrupts   = mwchan->ring_total;

    if (usrbuf->bytes_per_ring % usrbuf->desc_length)
        mwchan->sg_entries          = (size_t)(usrbuf->bytes_per_ring/usrbuf->desc_length) + 1;
    else {
        mwchan->sg_entries          = (size_t)(usrbuf->bytes_per_ring/usrbuf->desc_length);
    }

    /* Write to the IPCore_PacketSize_AXI4_Stream_Master 0x8 to specify the length*/
    /*reset pcore*/
    mw_ip_write32(MWDEV_TO_MWIP(mwdev), mwdev->mw_ipcore_dev->rst_reg, 0x1);
    /*reset pcore*/
    mw_ip_write32(MWDEV_TO_MWIP(mwdev), 0x8, usrbuf->counter);
    buf = MWDEV_TO_MWIP(mwdev)->dma_info.virt;
    mwchan->buf                 = &(buf[mwdev->channel_offset]);
    if (mwchan->buf == NULL) {
        dev_err(IP2DEVP(mwdev), "Buffer is NULL. Failed to allocate memory\n");
        return -ENOMEM;
	}
    mwchan->offset              =  mwdev->channel_offset;
    mwdev->channel_offset              =  mwdev->channel_offset + mwchan->length;
    /*
     * Set channel-index : used to notify appropriate DMA_CHX SYFS node
     */
    mwchan->chan_id             =  idx;
    idx++;
    dev_dbg(IP2DEVP(mwdev), "### Printing Channel info...\n");
    dev_dbg(IP2DEVP(mwdev), "Virtual Address        :0x%p\n", mwchan->buf);
    dev_dbg(IP2DEVP(mwdev), "Channel Length/Size    :%lu\n", mwchan->length);
    dev_dbg(IP2DEVP(mwdev), "Channel direction      :%d\n", mwchan->direction);
    dev_dbg(IP2DEVP(mwdev), "Total number of rings  :%d\n", mwchan->ring_total);
    dev_dbg(IP2DEVP(mwdev), "Buffer Descriptor size :%d\n", mwchan->bd_bytes);
    dev_dbg(IP2DEVP(mwdev), "Channel SG Entries     :%d\n", mwchan->sg_entries);
    dev_dbg(IP2DEVP(mwdev), "Buffer Interrupts      :%d\n", mwchan->buffer_interrupts);
    /* Get channel for DMA */
    mutex_init(&mwchan->lock);
    config.coalesc = 0;
    config.delay = 0;
    #if LINUX_VERSION_CODE <= KERNEL_VERSION(3,18,00)
    dmaengine_device_control(mwchan->chan, DMA_SLAVE_CONFIG, (unsigned long)&config);
    #else
    /* xilinx_dma_channel_set_config(mwchan->chan, (unsigned long)&config); */
    dmaengine_slave_config(mwchan->chan, (struct dma_slave_config *)&config);
    #endif
    dev_dbg(IP2DEVP(mwdev),"Name:%s, mwchan:0x%p, mwchan->chan:0x%p\n",
            dma_chan_name(mwchan->chan), mwchan, mwchan->chan);

    if (mwchan->ring_total >= 2) {
        status = mwadma_prep_desc(mwdev, mwchan);
    } else {
        /*
         * Instantiate scatter structure, it contains cookie, desc, callback
         */
        mwchan->scatter = devm_kzalloc(IP2DEVP(mwdev),
                sizeof(struct mwadma_slist), GFP_KERNEL);
    }
    init_completion(&mwchan->dma_complete);
    spin_lock_init(&mwchan->slock);
    mwchan->transfer_queued = 0;
    return status;
}

static void mwdma_test_loopback(struct mwadma_dev * mwdev,
        struct mw_axidma_params chan_prm)
{
    int i = 0;
    size_t len;
    char *dma_addr = MWDEV_TO_MWIP(mwdev)->dma_info.virt;
    unsigned int *tmp;
    /* rx = &dma_addr[0];
     * tx = &dma_addr[chan_prm.size];
     */
    dev_dbg(IP2DEVP(mwdev),"### test loopback\n");

    len = chan_prm.size;
    /* prime the rx & tx buffers */
    tmp = (unsigned int *) dma_addr;
    for (i=0;i<(len/sizeof(unsigned int));i++)
    {
        tmp[i] = 0xDEADC0DE;
    }
    tmp = (unsigned int *) (dma_addr + len);
    for (i=0;i<(len/sizeof(unsigned int));i++)
    {
        tmp[i] = (i+1) % (chan_prm.bytes_per_ring/sizeof(unsigned int));
    }
    /* Receive single ring */
    mwdev->rx->callback = (dma_async_tx_callback)mwadma_rx_cb_single_signal;
    mwdev->rx->error = 0;
    mwdev->rx->transfer_count = 0;
    mwadma_start(mwdev, mwdev->rx);
    dma_async_issue_pending(mwdev->rx->chan);
    /* Transmit single ring */
    mwdev->tx->transfer_queued += 1;
    mwdev->tx->callback = (dma_async_tx_callback)mwadma_tx_cb_single_signal;
    mwdev->tx->next_index = (mwdev->tx->next_index + 1) % mwdev->tx->ring_total;
    mwadma_start(mwdev,mwdev->tx);
    dma_async_issue_pending(mwdev->tx->chan);
}


static ssize_t mwdma_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    dev_dbg(dev,"sysfs_notify :%s\n", attr->attr.name);
    // sysfs_notify(&dev->kobj, NULL, attr->attr.name);
    return (sizeof(int));
}

static ssize_t mwdma_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    dev_dbg(dev, "sysfs_read :%s\n",attr->attr.name);
    return sprintf(buf, "%s\n", attr->attr.name);
}

static DEVICE_ATTR(dma_ch1, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch2, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch3, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch4, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch5, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch6, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch7, S_IRUGO, mwdma_show, mwdma_store);
static DEVICE_ATTR(dma_ch8, S_IRUGO, mwdma_show, mwdma_store);

static struct attribute *mwdma_attributes[] = {
    &dev_attr_dma_ch1.attr,
    &dev_attr_dma_ch2.attr,
    &dev_attr_dma_ch3.attr,
    &dev_attr_dma_ch4.attr,
    &dev_attr_dma_ch5.attr,
    &dev_attr_dma_ch6.attr,
    &dev_attr_dma_ch7.attr,
    &dev_attr_dma_ch8.attr,
    NULL,
};

static const struct attribute_group mwdma_attr_group = {
    .attrs = mwdma_attributes,
};

static void mwadma_get_devname(struct mathworks_ip_info *mw_ip_info,char *devname){
	snprintf(devname,MATHWORKS_IP_DEVNAME_LEN, "%s", mw_ip_info->name);
}

static struct mathworks_ip_ops mwadma_ip_ops = {
	.get_devname = mwadma_get_devname,
	.get_param = NULL,
	.fops = &mwadma_cdev_fops,
};

struct mathworks_ip_ops* mw_stream_channel_get_ops(void) {
	return &mwadma_ip_ops;
}

EXPORT_SYMBOL_GPL(mw_stream_channel_get_ops);

/********************************
 * Channel Sysfs
 ********************************/

static ssize_t mw_stream_chan_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    dev_dbg(dev,"sysfs_notify :%s\n", attr->attr.name);
    // sysfs_notify(&dev->kobj, NULL, attr->attr.name);
    return (sizeof(int));
}

static ssize_t mw_stream_chan_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
    dev_dbg(dev, "sysfs_read :%s\n",attr->attr.name);
    return sprintf(buf, "%s\n", attr->attr.name);
}

static DEVICE_ATTR(dma_irq, S_IRUGO, mw_stream_chan_show, mw_stream_chan_store);

static struct attribute *mw_stream_channel_attributes[] = {
    &dev_attr_dma_irq.attr,
    NULL,
};

static const struct attribute_group mw_stream_chan_group = {
	.attrs = mw_stream_channel_attributes,
};

static const struct attribute_group *mw_stream_chan_groups[] = {
	&mw_stream_chan_group,
	NULL
};

/********************************
 * Channel IDA
 ********************************/

static void mw_stream_chan_ida_remove(void *opaque){
	struct mwadma_chan* mwchan = opaque;
	ida_simple_remove(&mw_stream_channel_ida, mwchan->dev.id);
}

/* Nothing to actually do upon release */
static void mw_stream_chan_release(struct device *dev)
{
	struct mwadma_chan* mwchan = STREAMDEV_TO_MWCHAN(dev);
	dev_dbg(dev, "Freeing scatter channel dma memory\n");
	if ( (mwchan->scatter !=NULL) && (&mwchan->scatter->list != NULL))
	{
		mwadma_free_channel(mwchan->mwdev, mwchan);
	}
}

static struct mwadma_chan* __must_check mw_stream_chan_probe(
		struct mwadma_dev *mwdev,
		enum dma_transfer_direction direction,
		const char *name)
{
	struct dma_chan *chan = NULL;
	int status;
	struct mwadma_chan* mwchan;
	void* resID;

	resID = devres_open_group(IP2DEVP(mwdev), NULL, GFP_KERNEL);
	if(!resID)
		return ERR_PTR(-ENOMEM);

	chan = dma_request_slave_channel_reason(IP2DEVP(mwdev), name);
	if(IS_ERR(chan)){
		if (PTR_ERR(chan) == -EPROBE_DEFER) {
			dev_info(IP2DEVP(mwdev), "Deferring probe for channel %s\n", name);
		} else {
			dev_err(IP2DEVP(mwdev), "Could not find DMA channel %s\n", name);
		}
		return (void *)chan;
	}
	/* Create the cleanup action */
	status = devm_add_action(IP2DEVP(mwdev), (devm_action_fn)dma_release_channel, chan);
	if(status){
		dma_release_channel(chan);
		return ERR_PTR(status);
	}
	mwchan = (struct mwadma_chan*)devm_kzalloc(IP2DEVP(mwdev),
			sizeof(struct mwadma_chan),GFP_KERNEL);
	if(!mwchan){
		dev_err(IP2DEVP(mwdev), "Failed to allocate memory for channel %s\n", name);
		return ERR_PTR(-ENOMEM);
	}
	mwchan->mwdev = mwdev;
	mwchan->chan = chan;
	mwchan->direction = direction;

	device_initialize(&mwchan->dev);
	mwchan->dev.parent = IP2DEVP(mwdev);
	mwchan->dev.of_node = chan->dev->device.of_node;
	mwchan->dev.groups = mw_stream_chan_groups;
	mwchan->dev.id = ida_simple_get(&mw_stream_channel_ida, 0, 0, GFP_KERNEL);
	mwchan->dev.release = mw_stream_chan_release;
	if (mwchan->dev.id < 0) {
		return ERR_PTR(mwchan->dev.id);
	}
	status = devm_add_action(IP2DEVP(mwdev),mw_stream_chan_ida_remove, mwchan);
	if(status){
		ida_simple_remove(&mw_stream_channel_ida, mwchan->dev.id);
		return ERR_PTR(status);
	}

	status = dev_set_name(&mwchan->dev, "%s:%s", dev_name(MWDEV_TO_MWIP(mwdev)->char_device), name);
	if (status)
		return ERR_PTR(status);

	status = device_add(&mwchan->dev);
	if (status)
		return ERR_PTR(status);
	status = devm_add_action(IP2DEVP(mwdev), (devm_action_fn)device_unregister, &mwchan->dev);
	if(status){
		device_unregister(&mwchan->dev);
		return ERR_PTR(status);
	}
	devres_close_group(IP2DEVP(mwdev), resID);

	mwchan->irq_kn = sysfs_get_dirent(mwchan->dev.kobj.sd, "dma_irq");
	if(!mwchan->irq_kn){
		return ERR_PTR(-ENODEV);
	}
	status = devm_add_action(&mwchan->dev, (devm_action_fn)sysfs_put, mwchan->irq_kn);
	if(status) {
		sysfs_put(mwchan->irq_kn);
		return ERR_PTR(status);
	}

	return mwchan;
}

void mw_stream_channels_release(void *opaque){
	struct mwadma_dev *mwdev = opaque;
	dev_info(IP2DEVP(mwdev), "Removing sysfs entries...");
	sysfs_remove_group(&IP2DEVP(mwdev)->kobj, &mwdma_attr_group);
}

int mw_stream_channels_probe(struct mathworks_ipcore_dev *mw_ipcore_dev) {
	struct mwadma_dev *mwdev;
	struct device *dev = mw_ipcore_dev->mw_ip_info->dev;
	int nchan;
	int status;

	mwdev = (struct mwadma_dev*)devm_kzalloc(dev, sizeof(struct mwadma_dev),GFP_KERNEL);
	if (!mwdev) {
		dev_err(dev, "Failed to allocate memory for device context\n");
		return -ENOMEM;
	}

	mwdev->mw_ipcore_dev = mw_ipcore_dev;
	mw_ipcore_dev->private = (void*)mwdev;

	nchan = of_property_count_strings(dev->of_node, "dma-names");
	if (nchan == -EINVAL){
		dev_dbg(IP2DEVP(mwdev), "DMA Channels not found in device tree\n");
		return 0;
	}
	if (nchan < 0){
		dev_err(IP2DEVP(mwdev), "Invalid dma-names specification\n");
		return nchan;
	}

	mwdev->tx = mw_stream_chan_probe(mwdev, DMA_MEM_TO_DEV, "mm2s");
	if (IS_ERR(mwdev->tx))
		return PTR_ERR(mwdev->tx);
	mwdev->rx = mw_stream_chan_probe(mwdev, DMA_DEV_TO_MEM, "s2mm");
	if (IS_ERR(mwdev->rx))
		return PTR_ERR(mwdev->rx);

	status = sysfs_create_group(&dev->kobj, &mwdma_attr_group);
	if (status) {
		dev_err(IP2DEVP(mwdev), "Error creating the sysfs devices\n");
		return status;
	}
	status = devm_add_action(dev, mw_stream_channels_release, mwdev);
	if(status) {
		mw_stream_channels_release(mwdev);
		return status;
	}

	return 0;
}

EXPORT_SYMBOL_GPL(mw_stream_channels_probe);

static int __init mw_stream_channel_init(void)
{
	return 0;
}

static void __exit mw_stream_channel_exit(void)
{

}

module_init(mw_stream_channel_init);
module_exit(mw_stream_channel_exit);

MODULE_AUTHOR("MathWorks, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("MathWorks Streaming DMA Channel");
MODULE_ALIAS(DRIVER_NAME);
