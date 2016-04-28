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
static atomic64_t rxcount = ATOMIC64_INIT(0);
static LIST_HEAD(mwadma_rx_userid);
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

int mwadma_start(struct mwadma_chan *mwchan);
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
static int mwadma_allocate_desc(struct mwadma_slist **new, struct mwadma_chan *mwchan, unsigned int this_idx)
{
    struct mwadma_slist *tmp;
    size_t ring_bytes;
    
    ring_bytes = mwchan->length/mwchan->ring_total;
    tmp = devm_kmalloc(&mwchan->dev, sizeof(struct mwadma_slist),GFP_KERNEL);
    if (!tmp) {
    	return -ENOMEM;
    }
    /* set buffer at offset from larger buffer */
    tmp->phys = mwchan->phys+this_idx*ring_bytes;
    tmp->length = ring_bytes;
    tmp->buffer_index = this_idx;
    tmp->state = MWDMA_READY;
    tmp->qchan = mwchan;
    INIT_LIST_HEAD(&(tmp->userid));
    dev_dbg(&mwchan->dev,"buf_phys_addr 0x%08lx, size %zu\n", (unsigned long) tmp->phys, tmp->length);
    *new = tmp;
    return 0;
}

/*
 * @brief mwadma_free_desc
 */
static void mwadma_free_desc(struct mwadma_slist *desc, struct mwadma_chan *mwchan) {
	devm_kfree(&mwchan->dev, desc);
}

/*
 * @brief mwadma_prep_desc
 */
static int mwadma_prep_desc(struct mwadma_dev *mwdev, struct mwadma_chan * mwchan)
{
    unsigned int i = 0;
    int ret;
    struct mwadma_slist *new;
    struct mwadma_slist *b;
    struct mwadma_slist **blocks;

    blocks = devm_kmalloc(&mwchan->dev, sizeof(*blocks)*mwchan->ring_total, GFP_KERNEL);
    if (!blocks) {
        return -ENOMEM; 
    }

    ret = mwadma_allocate_desc(&(mwchan->scatter), mwchan, 0);
    if (ret) {
        dev_err(&mwchan->dev, "Failed in mwadma_allocate_desc");
        return -ENOMEM;
    }
    INIT_LIST_HEAD(&(mwchan->scatter->list));
    for(i = 1; i < mwchan->ring_total; i++) /* POOL_SIZE - 1 */
    {
        ret = mwadma_allocate_desc(&(new), mwchan, i);
        if ((ret < 0) || (new == NULL)) {
            dev_err(&mwchan->dev, "Failed in mwadma_allocate_desc");
            return -ENOMEM;
        }
        list_add_tail(&(new->list),&(mwchan->scatter->list));
    }
    mwchan->curr = mwchan->scatter; /*Head of the list*/
    mwchan->prev = list_entry(mwchan->curr->list.prev, struct mwadma_slist, list);
    blocks[0] = mwchan->scatter;
    i = 1;
    list_for_each_entry(b, &(mwchan->scatter->list), list){
        blocks[i] = b;
        i++;
    }
    mwchan->blocks = blocks;
    mwchan->status = ready;
    return 0;
}

void mwadma_tx_cb_single_signal(void *data)
{
    struct mwadma_slist *block = data;
    struct mwadma_chan *mwchan = block->qchan;
    unsigned long flags;

    spin_lock_irqsave(&mwchan->slock, flags);
    mwchan->blocks[block->buffer_index]->state = MWDMA_READY;
    mwchan->transfer_queued--;
    mwchan->transfer_count++;
    mwchan->status = ready;
    spin_unlock_irqrestore(&mwchan->slock, flags);

    sysfs_notify_dirent(mwchan->irq_kn);
}

void mwadma_tx_cb_continuous_signal_dataflow(void *data)
{
    struct mwadma_slist *block = data;
    struct mwadma_chan *mwchan = block->qchan;
    unsigned char start_next = 0;
    unsigned long flags;

    spin_lock_irqsave(&mwchan->slock, flags);
    mwchan->blocks[block->buffer_index]->state = MWDMA_READY;
    mwchan->transfer_queued--;
    if(mwchan->transfer_queued > 0) {
        if(mwchan->transfer_queued > TX_WATERMARK_QFULL) /* High watermark */ {
            mwchan->error = TX_ERROR_QFULL;
        } else if(mwchan->transfer_queued > TX_WATERMARK_QPRIME) /* Normal */ {
            mwchan->error = TX_ERROR_QPRIME;
        } else if(mwchan->transfer_queued >= TX_WATERMARK_QLOW) /* Low */ {
            mwchan->error = TX_ERROR_QLOW;
        }
    }
    else /* Underflow */ {
        mwchan->error = TX_ERROR_QUNDERFLOW;
        mwchan->status = waiting;
    }
    mwchan->transfer_count++;
    spin_unlock_irqrestore(&mwchan->slock, flags);
    sysfs_notify_dirent(mwchan->irq_kn);
    if (start_next) {
        mwadma_start(mwchan);
    }
}


void mwadma_tx_cb_continuous_signal(void *data)
{
    struct mwadma_slist *block= data;
    struct mwadma_chan *mwchan = block->qchan;
    unsigned long int flags;
    spin_lock_irqsave(&mwchan->slock, flags);
    mwchan->transfer_queued--;
    mwchan->transfer_count++;
    mwchan->blocks[block->buffer_index]->state = MWDMA_READY;
    spin_unlock_irqrestore(&mwchan->slock, flags);
    sysfs_notify_dirent(mwchan->irq_kn);
    mwadma_start(mwchan);
}


void mwadma_rx_cb_single_signal(void *data)
{
    struct mwadma_slist *block = data;
    struct mwadma_chan *mwchan = block->qchan;
    unsigned long flags;
    
    spin_lock_irqsave(&mwchan->slock, flags);
    mwchan->transfer_queued--;
    mwchan->transfer_count++;
    list_add_tail(&(block->userid), &mwadma_rx_userid);
    mwchan->blocks[block->buffer_index]->state = MWDMA_PENDING;
    mwchan->status = ready;
    spin_unlock_irqrestore(&mwchan->slock, flags);
    
    sysfs_notify_dirent(mwchan->irq_kn);
}

void mwadma_rx_cb_burst(void *data)
{
    struct mwadma_slist *block = data;
    struct mwadma_chan *mwchan = block->qchan;
    unsigned long flags;
    unsigned char start_next = 0;

    sysfs_notify_dirent(mwchan->irq_kn);
    spin_lock_irqsave(&mwchan->slock, flags);
    mwchan->transfer_queued--;
    mwchan->transfer_count++;
    if (mwchan->transfer_queued) {
        start_next = 1;
    } else {
        mwchan->next_index = block->buffer_index;
    }
    mwchan->blocks[block->buffer_index]->state = MWDMA_PENDING;
    list_add_tail(&(block->userid), &mwadma_rx_userid);
    spin_unlock_irqrestore(&mwchan->slock, flags);
    if(start_next) {
        mwadma_start(mwchan);
    }
}

void mwadma_rx_cb_continuous_signal(void *data)
{
    struct mwadma_slist *block = data;
    struct mwadma_chan *mwchan = block->qchan;
    unsigned long flags;
    unsigned int next_idx,start_next = 0;
    
    spin_lock_irqsave(&mwchan->slock, flags);
    mwchan->blocks[block->buffer_index]->state = MWDMA_PENDING;
    list_add_tail(&(block->userid), &mwadma_rx_userid);
    mwchan->transfer_count++;
    mwchan->transfer_queued++;
    next_idx = ( block->buffer_index + 1 ) % mwchan->ring_total;
    if (mwchan->blocks[next_idx]->state == MWDMA_PENDING) {
        mwchan->error = ERR_RING_OVERFLOW;
        start_next = 0;
    } else {
        start_next = 1;
    }
    spin_unlock_irqrestore(&mwchan->slock, flags);
    atomic64_inc(&rxcount);
    sysfs_notify_dirent(mwchan->irq_kn);
    if (start_next) {
        mwadma_start(mwchan);
    } 
}
/*
 * @brief mwadma_start
 */
int mwadma_start(struct mwadma_chan *mwchan)
{
    int ret = 0;
    struct mwadma_slist *newList;
    struct dma_async_tx_descriptor *thisDesc;
    dma_cookie_t ck;
    unsigned long flags;
    if(NULL == mwchan) {
        dev_err(&mwchan->dev, "mw-axidma: Channel queue pointer is NULL.\n");
        ret = -ENODEV;
        goto start_failed;
    } 
    if (mwchan->curr->state == MWDMA_PENDING) {
        return -ENOMEM;
    }
    thisDesc = dmaengine_prep_slave_single(mwchan->chan, mwchan->curr->phys, mwchan->curr->length, mwchan->direction, mwchan->flags);
    if (NULL == thisDesc) {
        dev_err(&mwchan->dev,"prep_slave_single failed: buf_phys_addr 0x%08lx, size %zu\n", (unsigned long) mwchan->curr->phys, mwchan->curr->length);
        ret = -ENOMEM;
        goto start_failed;
    }
    thisDesc->callback = mwchan->callback;
    thisDesc->callback_param = mwchan->curr;
    mwchan->curr->desc = thisDesc;
    mwchan->blocks[mwchan->curr->buffer_index]->state = MWDMA_ACTIVE;
    spin_lock_irqsave(&mwchan->slock, flags);
    newList = list_entry(mwchan->curr->list.next, struct mwadma_slist, list);
    mwchan->prev = mwchan->curr;
    mwchan->curr = newList;
    mwchan->status = running;
    spin_unlock_irqrestore(&mwchan->slock, flags);
    ck = dmaengine_submit(thisDesc);
    if (dma_submit_error(ck)) {
        dev_err(&mwchan->dev, "Failure in dmaengine_submit!\n");
        ret = -ENOSYS;
        goto start_failed;
    }
    dma_async_issue_pending(mwchan->chan);
start_failed:
    return ret;
}

/*
 * @brief mwadma_stop
 */
static int mwadma_stop(struct mwadma_chan *mwchan)
{
    dev_dbg(&mwchan->dev,"DMAENGINE_TERMINATE\n");
    dmaengine_terminate_all(mwchan->chan);
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
    unsigned int                next_index, done_index;
    unsigned int                error;
    unsigned long int flags;
    struct mwadma_slist *tmp = NULL;
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
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof(userval))) {
                return -EACCES;
            }
            mwchan->callback = (dma_async_tx_callback)mwadma_rx_cb_single_signal;
            spin_lock_bh(&mwchan->slock);
            mwchan->error = 0;
            mwchan->transfer_count = 0;
            spin_unlock_bh(&mwchan->slock);
            ret = mwadma_start(mwchan);
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
            ret = mwadma_start(mwchan);
            spin_lock_bh(&mwchan->slock);
            mwchan->status = running;
            spin_unlock_bh(&mwchan->slock);
            break;
        case MWADMA_RX_CONTINUOUS:
            if(copy_from_user(&userval, (unsigned long *)arg, sizeof(userval))) {
                return -EACCES;
            }
            mwchan->callback = (dma_async_tx_callback)mwadma_rx_cb_continuous_signal;
            ret = mwadma_start(mwchan); 
            break;
        case MWADMA_RX_STOP:
            spin_lock_bh(&mwchan->slock);
            status = (unsigned long) mwchan->status;
            spin_unlock_bh(&mwchan->slock);
            if(status != ready) {
                ret = mwadma_stop(mwchan);
                if (ret) {
                    dev_err(IP2DEVP(mwdev),"Error while stopping DMA\n");
                    return ret;
                }
            }
            spin_lock_bh(&mwchan->slock);
            mwchan->transfer_queued = 0;
            mwchan->transfer_count = 0;
            mwchan->status = ready;
            spin_unlock_bh(&mwchan->slock);
            INIT_LIST_HEAD(&mwadma_rx_userid);
            atomic64_set(&rxcount, 0LL);
            break;
        case MWADMA_RX_GET_NEXT_INDEX:
            spin_lock_irqsave(&mwchan->slock, flags);
            if (!list_empty(&mwadma_rx_userid)) {
                tmp = list_entry(mwadma_rx_userid.next, struct mwadma_slist, userid);
                next_index = tmp->buffer_index;
                list_del_init(mwadma_rx_userid.next);
            }
            spin_unlock_irqrestore(&mwchan->slock, flags);
            if (NULL == tmp) {
                return -ENOMEM;
            }
            if(copy_to_user((unsigned int *) arg, &next_index, sizeof(next_index))) {
                return -EACCES;
            }
            break;
        case MWADMA_RX_GET_ERROR:
            if(copy_from_user(&done_index, (unsigned int *)arg, sizeof(done_index))) {
                return -EACCES;
            }
            spin_lock_irqsave(&mwchan->slock, flags);
            mwchan->transfer_queued--;
            error = mwchan->error;
            mwchan->error = 0;
            mwchan->blocks[done_index]->state = MWDMA_READY;
            spin_unlock_irqrestore(&mwchan->slock, flags);
            atomic64_dec_if_positive(&rxcount);
            if(copy_to_user((unsigned int *) arg, &error, sizeof(error))) {
                return -EACCES;
            }
            break;
        case MWADMA_FREE_RX_CHANNEL:
            mwadma_free_channel(mwdev, mwchan);
            break;
        default:
            return 1;
    }
    return ret;
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
                mwadma_start(mwchan);
                spin_lock_bh(&mwchan->slock);
                mwchan->status = running; /*Data ready */
                spin_unlock_bh(&mwchan->slock);
                mwadma_start(mwchan);
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
            mwadma_start(mwchan);
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

                ret = mwadma_stop(mwchan);
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

            spin_lock_bh(&mwchan->slock);
            userval = mwchan->error; 
            /*mwchan->error = 0;*/
            spin_unlock_bh(&mwchan->slock);

            if(copy_to_user((unsigned long *) arg, &userval, sizeof(unsigned long)))
            {
                return -EACCES;
            }
            break;
        case MWADMA_TX_GET_NEXT_INDEX:
            spin_lock_bh(&mwchan->slock);
            userval = (unsigned long) mwchan->next_index;
            spin_unlock_bh(&mwchan->slock);
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
    struct mwadma_slist *curr, *next;
    spin_lock_bh(&mwchan->slock);
    list_for_each_entry_safe(curr, next, &(mwchan->scatter->list), list) {
	list_del(&curr->list);
	mwadma_free_desc(curr, mwchan);
    }
    mwadma_free_desc(mwchan->scatter, mwchan);
    spin_unlock_bh(&mwchan->slock);
    dmaengine_terminate_all(mwchan->chan);
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
        atomic64_set(&rxcount, 0LL);
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
static int mwadma_mmap_fault(struct vm_fault *vmf)
{
    struct vm_area_struct *vma = vmf->vma;
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
                (void *)((uintptr_t)MWDEV_TO_MWIP(mwdev)->dma_info.phys),
		(unsigned int)bufferSize);
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
    dma_addr_t phys;
    if ( (mwdev == NULL) || (mwchan == NULL) ) {
        return -EINVAL;
    }
    mwchan->flags               = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
    mwchan->ring_total          = usrbuf->total_rings;
    mwchan->length              = usrbuf->bytes_per_ring * usrbuf->total_rings;
    mwchan->bd_bytes            = usrbuf->bytes_per_ring;

    /* Write to the IPCore_PacketSize_AXI4_Stream_Master 0x8 to specify the length*/
    /*reset pcore*/
    mw_ip_reset(mwdev->mw_ipcore_dev);
    /*reset pcore*/
    mw_ip_write32(MWDEV_TO_MWIP(mwdev), 0x8, usrbuf->counter);
    if (MWDEV_TO_MWIP(mwdev)->dma_info.virt == NULL) {
		dev_err(IP2DEVP(mwdev), "Buffer is NULL. Failed to allocate memory\n");
		return -ENOMEM;
	}
    buf = MWDEV_TO_MWIP(mwdev)->dma_info.virt;
    phys = MWDEV_TO_MWIP(mwdev)->dma_info.phys;
    mwchan->buf                 = &(buf[mwdev->channel_offset]);

    mwchan->phys = (phys + mwdev->channel_offset);
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
    /* Get channel for DMA */
    mutex_init(&mwchan->lock);

    dev_dbg(IP2DEVP(mwdev),"Name:%s, mwchan:0x%p, mwchan->chan:0x%p\n",
            dma_chan_name(mwchan->chan), mwchan, mwchan->chan);
    status = mwadma_prep_desc(mwdev, mwchan);
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
    mwadma_start(mwdev->rx);
    /* Transmit single ring */
    mwdev->tx->transfer_queued += 1;
    mwdev->tx->callback = (dma_async_tx_callback)mwadma_tx_cb_single_signal;
    mwdev->tx->next_index = (mwdev->tx->next_index + 1) % mwdev->tx->ring_total;
    mwadma_start(mwdev->tx);
}


static ssize_t mwdma_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    dev_dbg(dev,"sysfs_notify :%s\n", attr->attr.name);
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
    dev_dbg(dev,"sysfs_notify:%s\n", attr->attr.name);
    return (sizeof(int));
}

static ssize_t mw_stream_chan_show(struct device *dev, struct device_attribute *attr,
        char *buf)
{
	return sprintf(buf, "%llu\n",
		       (unsigned long long)atomic64_read(&rxcount));
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

void mw_stream_channels_release(void *opaque) {
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
	if (nchan < 0) {
		dev_err(IP2DEVP(mwdev), "Invalid dma-names specification. Incorrect dma-names property.\n");
		return nchan;
	}

	mwdev->tx = mw_stream_chan_probe(mwdev, DMA_MEM_TO_DEV, "mm2s");
	if (IS_ERR(mwdev->tx) && (PTR_ERR(mwdev->tx) == -EPROBE_DEFER))
		return PTR_ERR(mwdev->tx);
	mwdev->rx = mw_stream_chan_probe(mwdev, DMA_DEV_TO_MEM, "s2mm");
	if (IS_ERR(mwdev->rx) && (PTR_ERR(mwdev->rx) == -EPROBE_DEFER))
		return PTR_ERR(mwdev->rx);

	if (nchan < 2) {
		if(IS_ERR(mwdev->tx) && IS_ERR(mwdev->rx)) {
			dev_err(IP2DEVP(mwdev),"MM2S/S2MM not found for nchan=%d\n",nchan);
			return PTR_ERR(mwdev->tx);
		}
	} else {
		if (IS_ERR(mwdev->tx)) {
			dev_err(IP2DEVP(mwdev),"MM2S not found for nchan=%d\n",nchan);
			return PTR_ERR(mwdev->tx);
		}
		if (IS_ERR(mwdev->rx)) {
			dev_err(IP2DEVP(mwdev),"S2MM not found for nchan=%d\n",nchan);
			return PTR_ERR(mwdev->rx);
		}
	}
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
