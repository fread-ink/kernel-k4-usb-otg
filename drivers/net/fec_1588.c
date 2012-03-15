/*
 * drivers/net/fec_1588.c
 *
 * Copyright (C) 2010 Freescale Semiconductor, Inc.
 * Copyright (C) 2009 IXXAT Automation, GmbH
 *
 * FEC Ethernet Driver -- IEEE 1588 interface functionality
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/io.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include "fec.h"
#include "fec_1588.h"

static DECLARE_WAIT_QUEUE_HEAD(ptp_rx_ts_wait);
#define PTP_GET_RX_TIMEOUT      (HZ/10)

static struct fec_ptp_private *ptp_private[2];

/* Alloc the ring resource */
static int fec_ptp_init_circ(struct circ_buf *ptp_buf)
{
	ptp_buf->buf = vmalloc(DEFAULT_PTP_RX_BUF_SZ *
					sizeof(struct fec_ptp_data_t));

	if (!ptp_buf->buf)
		return 1;
	ptp_buf->head = 0;
	ptp_buf->tail = 0;

	return 0;
}

static inline int fec_ptp_calc_index(int size, int curr_index, int offset)
{
	return (curr_index + offset) % size;
}

static int fec_ptp_is_empty(struct circ_buf *buf)
{
	return (buf->head == buf->tail);
}

static int fec_ptp_nelems(struct circ_buf *buf)
{
	const int front = buf->head;
	const int end = buf->tail;
	const int size = DEFAULT_PTP_RX_BUF_SZ;
	int n_items;

	if (end > front)
		n_items = end - front;
	else if (end < front)
		n_items = size - (front - end);
	else
		n_items = 0;

	return n_items;
}

static int fec_ptp_is_full(struct circ_buf *buf)
{
	if (fec_ptp_nelems(buf) ==
				(DEFAULT_PTP_RX_BUF_SZ - 1))
		return 1;
	else
		return 0;
}

static int fec_ptp_insert(struct circ_buf *ptp_buf,
			  struct fec_ptp_data_t *data,
			  struct fec_ptp_private *priv)
{
	struct fec_ptp_data_t *tmp;

	if (fec_ptp_is_full(ptp_buf))
		return 1;

	spin_lock(&priv->ptp_lock);
	tmp = (struct fec_ptp_data_t *)(ptp_buf->buf) + ptp_buf->tail;

	tmp->key = data->key;
	tmp->ts_time.sec = data->ts_time.sec;
	tmp->ts_time.nsec = data->ts_time.nsec;

	ptp_buf->tail = fec_ptp_calc_index(DEFAULT_PTP_RX_BUF_SZ,
					ptp_buf->tail, 1);
	spin_unlock(&priv->ptp_lock);

	return 0;
}

static int fec_ptp_find_and_remove(struct circ_buf *ptp_buf,
				   int key,
				   struct fec_ptp_data_t *data,
				   struct fec_ptp_private *priv)
{
	int i;
	int size = DEFAULT_PTP_RX_BUF_SZ;
	int end = ptp_buf->tail;
	unsigned long flags;
	struct fec_ptp_data_t *tmp;

	if (fec_ptp_is_empty(ptp_buf))
		return 1;

	i = ptp_buf->head;
	while (i != end) {
		tmp = (struct fec_ptp_data_t *)(ptp_buf->buf) + i;
		if (tmp->key == key)
			break;
		i = fec_ptp_calc_index(size, i, 1);
	}

	spin_lock_irqsave(&priv->ptp_lock, flags);
	if (i == end) {
		ptp_buf->head = end;
		spin_unlock_irqrestore(&priv->ptp_lock, flags);
		return 1;
	}

	data->ts_time.sec = tmp->ts_time.sec;
	data->ts_time.nsec = tmp->ts_time.nsec;

	ptp_buf->head = fec_ptp_calc_index(size, i, 1);
	spin_unlock_irqrestore(&priv->ptp_lock, flags);

	return 0;
}

/* 1588 Module intialization */
int fec_ptp_start(struct fec_ptp_private *priv)
{
	struct fec_ptp_private *fpp = priv;

	/* Select 1588 Timer source and enable module for starting Tmr Clock */
	writel(FEC_T_CTRL_RESTART, fpp->hwp + FEC_ATIME_CTRL);
	writel(FEC_T_INC_40MHZ << FEC_T_INC_OFFSET, fpp->hwp + FEC_ATIME_INC);
	writel(FEC_T_PERIOD_ONE_SEC, fpp->hwp + FEC_ATIME_EVT_PERIOD);
	/* start counter */
	writel(FEC_T_CTRL_PERIOD_RST | FEC_T_CTRL_ENABLE,
			fpp->hwp + FEC_ATIME_CTRL);

	return 0;
}

/* Cleanup routine for 1588 module.
 * When PTP is disabled this routing is called */
void fec_ptp_stop(struct fec_ptp_private *priv)
{
	struct fec_ptp_private *fpp = priv;

	writel(0, fpp->hwp + FEC_ATIME_CTRL);
	writel(FEC_T_CTRL_RESTART, fpp->hwp + FEC_ATIME_CTRL);

}

static void fec_get_curr_cnt(struct fec_ptp_private *priv,
			struct ptp_rtc_time *curr_time)
{
	writel(FEC_T_CTRL_CAPTURE, priv->hwp + FEC_ATIME_CTRL);
	curr_time->rtc_time.nsec = readl(priv->hwp + FEC_ATIME);
	curr_time->rtc_time.sec = priv->prtc;
	writel(FEC_T_CTRL_CAPTURE, priv->hwp + FEC_ATIME_CTRL);
	if (readl(priv->hwp + FEC_ATIME) < curr_time->rtc_time.nsec)
		curr_time->rtc_time.sec++;
}

/* Set the 1588 timer counter registers */
static void fec_set_1588cnt(struct fec_ptp_private *priv,
			struct ptp_rtc_time *fec_time)
{
	u32 tempval;
	unsigned long flags;

	spin_lock_irqsave(&priv->cnt_lock, flags);
	priv->prtc = fec_time->rtc_time.sec;

	tempval = fec_time->rtc_time.nsec;
	writel(tempval, priv->hwp + FEC_ATIME);
	spin_unlock_irqrestore(&priv->cnt_lock, flags);
}

/* Set the BD to ptp */
int fec_ptp_do_txstamp(struct sk_buff *skb)
{
	struct iphdr *iph;
	struct udphdr *udph;

	if (skb->len > 44) {
		/* Check if port is 319 for PTP Event, and check for UDP */
		iph = ip_hdr(skb);
		if (iph == NULL || iph->protocol != FEC_PACKET_TYPE_UDP)
			return 0;

		udph = udp_hdr(skb);
		if (udph != NULL && udph->source == 319)
			return 1;
	}

	return 0;
}

void fec_ptp_store_txstamp(struct fec_ptp_private *priv)
{
	struct fec_ptp_private *fpp = priv;
	unsigned int reg;

	reg = readl(fpp->hwp + FEC_TS_TIMESTAMP);
	fpp->txstamp.nsec = reg;
	fpp->txstamp.sec = fpp->prtc;
}

void fec_ptp_store_rxstamp(struct fec_ptp_private *priv,
			   struct sk_buff *skb,
			   struct bufdesc *bdp)
{
	int msg_type, seq_id, control;
	struct fec_ptp_data_t tmp_rx_time;
	struct fec_ptp_private *fpp = priv;
	struct iphdr *iph;
	struct udphdr *udph;

	/* Check for UDP, and Check if port is 319 for PTP Event */
	iph = (struct iphdr *)(skb->data + FEC_PTP_IP_OFFS);
	if (iph->protocol != FEC_PACKET_TYPE_UDP)
		return;

	udph = (struct udphdr *)(skb->data + FEC_PTP_UDP_OFFS);
	if (udph->source != 319)
		return;

	seq_id = *((u16 *)(skb->data + FEC_PTP_SEQ_ID_OFFS));
	control = *((u8 *)(skb->data + FEC_PTP_CTRL_OFFS));

	tmp_rx_time.key = seq_id;
	tmp_rx_time.ts_time.sec = fpp->prtc;
	tmp_rx_time.ts_time.nsec = bdp->ts;

	switch (control) {

	case PTP_MSG_SYNC:
		fec_ptp_insert(&(priv->rx_time_sync), &tmp_rx_time, priv);
		break;

	case PTP_MSG_DEL_REQ:
		fec_ptp_insert(&(priv->rx_time_del_req), &tmp_rx_time, priv);
		break;

	/* clear transportSpecific field*/
	case PTP_MSG_ALL_OTHER:
		msg_type = (*((u8 *)(skb->data +
				FEC_PTP_MSG_TYPE_OFFS))) & 0x0F;
		switch (msg_type) {
		case PTP_MSG_P_DEL_REQ:
			fec_ptp_insert(&(priv->rx_time_pdel_req),
						&tmp_rx_time, priv);
			break;
		case PTP_MSG_P_DEL_RESP:
			fec_ptp_insert(&(priv->rx_time_pdel_resp),
					&tmp_rx_time, priv);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}

	wake_up_interruptible(&ptp_rx_ts_wait);
}

static void fec_get_tx_timestamp(struct fec_ptp_private *priv,
				 struct ptp_time *tx_time)
{
	tx_time->sec = priv->txstamp.sec;
	tx_time->nsec = priv->txstamp.nsec;
}

static uint8_t fec_get_rx_time(struct fec_ptp_private *priv,
			       struct ptp_ts_data *pts,
			       struct ptp_time *rx_time)
{
	struct fec_ptp_data_t tmp;
	int key, flag;
	u8 mode;

	key = pts->seq_id;
	mode = pts->message_type;
	switch (mode) {
	case PTP_MSG_SYNC:
		flag = fec_ptp_find_and_remove(&(priv->rx_time_sync),
						key, &tmp, priv);
		break;
	case PTP_MSG_DEL_REQ:
		flag = fec_ptp_find_and_remove(&(priv->rx_time_del_req),
						key, &tmp, priv);
		break;

	case PTP_MSG_P_DEL_REQ:
		flag = fec_ptp_find_and_remove(&(priv->rx_time_pdel_req),
						key, &tmp, priv);
		break;
	case PTP_MSG_P_DEL_RESP:
		flag = fec_ptp_find_and_remove(&(priv->rx_time_pdel_resp),
						key, &tmp, priv);
		break;

	default:
		flag = 1;
		printk(KERN_ERR "ERROR\n");
		break;
	}

	if (!flag) {
		rx_time->sec = tmp.ts_time.sec;
		rx_time->nsec = tmp.ts_time.nsec;
		return 0;
	} else {
		wait_event_interruptible_timeout(ptp_rx_ts_wait, 0,
					PTP_GET_RX_TIMEOUT);

		switch (mode) {
		case PTP_MSG_SYNC:
			flag = fec_ptp_find_and_remove(&(priv->rx_time_sync),
				key, &tmp, priv);
			break;
		case PTP_MSG_DEL_REQ:
			flag = fec_ptp_find_and_remove(
				&(priv->rx_time_del_req), key, &tmp, priv);
			break;
		case PTP_MSG_P_DEL_REQ:
			flag = fec_ptp_find_and_remove(
				&(priv->rx_time_pdel_req), key, &tmp, priv);
			break;
		case PTP_MSG_P_DEL_RESP:
			flag = fec_ptp_find_and_remove(
				&(priv->rx_time_pdel_resp), key, &tmp, priv);
			break;
		}

		if (flag == 0) {
			rx_time->sec = tmp.ts_time.sec;
			rx_time->nsec = tmp.ts_time.nsec;
			return 0;
		}

		return -1;
	}
}

static int ptp_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int ptp_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int ptp_ioctl(
	struct inode *inode,
	struct file *file,
	unsigned int cmd,
	unsigned long arg)
{
	struct ptp_rtc_time *cnt;
	struct ptp_rtc_time curr_time;
	struct ptp_time rx_time, tx_time;
	struct ptp_ts_data *p_ts;
	struct fec_ptp_private *priv;
	unsigned int minor = MINOR(inode->i_rdev);
	int retval = 0;

	priv = (struct fec_ptp_private *) ptp_private[minor];
	switch (cmd) {
	case PTP_GET_RX_TIMESTAMP:
		p_ts = (struct ptp_ts_data *)arg;
		retval = fec_get_rx_time(priv, p_ts, &rx_time);
		if (retval == 0)
			copy_to_user((void __user *)(&(p_ts->ts)), &rx_time,
					sizeof(rx_time));
		break;
	case PTP_GET_TX_TIMESTAMP:
		p_ts = (struct ptp_ts_data *)arg;
		fec_get_tx_timestamp(priv, &tx_time);
		copy_to_user((void __user *)(&(p_ts->ts)), &tx_time,
				sizeof(tx_time));
		break;
	case PTP_GET_CURRENT_TIME:
		fec_get_curr_cnt(priv, &curr_time);
		copy_to_user((void __user *)arg, &curr_time, sizeof(curr_time));
		break;
	case PTP_SET_RTC_TIME:
		cnt = (struct ptp_rtc_time *)arg;
		fec_set_1588cnt(priv, cnt);
		break;
	case PTP_FLUSH_TIMESTAMP:
		/* reset sync buffer */
		priv->rx_time_sync.head = 0;
		priv->rx_time_sync.tail = 0;
		/* reset delay_req buffer */
		priv->rx_time_del_req.head = 0;
		priv->rx_time_del_req.tail = 0;
		/* reset pdelay_req buffer */
		priv->rx_time_pdel_req.head = 0;
		priv->rx_time_pdel_req.tail = 0;
		/* reset pdelay_resp buffer */
		priv->rx_time_pdel_resp.head = 0;
		priv->rx_time_pdel_resp.tail = 0;
		break;
	case PTP_SET_COMPENSATION:
		/* TBD */
		break;
	case PTP_GET_ORIG_COMP:
		/* TBD */
		break;
	default:
		return -EINVAL;
	}
	return retval;
}

static const struct file_operations ptp_fops = {
	.owner	= THIS_MODULE,
	.llseek	= NULL,
	.read	= NULL,
	.write	= NULL,
	.ioctl	= ptp_ioctl,
	.open	= ptp_open,
	.release = ptp_release,
};

static int init_ptp(void)
{
	if (register_chrdev(PTP_MAJOR, "ptp", &ptp_fops))
		printk(KERN_ERR "Unable to register PTP deivce as char\n");

	return 0;
}

static void ptp_free(void)
{
	/*unregister the PTP device*/
	unregister_chrdev(PTP_MAJOR, "ptp");
}

/*
 * Resource required for accessing 1588 Timer Registers.
 */
int fec_ptp_init(struct fec_ptp_private *priv, int id)
{
	fec_ptp_init_circ(&(priv->rx_time_sync));
	fec_ptp_init_circ(&(priv->rx_time_del_req));
	fec_ptp_init_circ(&(priv->rx_time_pdel_req));
	fec_ptp_init_circ(&(priv->rx_time_pdel_resp));

	spin_lock_init(&priv->ptp_lock);
	spin_lock_init(&priv->cnt_lock);
	ptp_private[id] = priv;
	if (id == 0)
		init_ptp();
	return 0;
}
EXPORT_SYMBOL(fec_ptp_init);

void fec_ptp_cleanup(struct fec_ptp_private *priv)
{

	if (priv->rx_time_sync.buf)
		vfree(priv->rx_time_sync.buf);
	if (priv->rx_time_del_req.buf)
		vfree(priv->rx_time_del_req.buf);
	if (priv->rx_time_pdel_req.buf)
		vfree(priv->rx_time_pdel_req.buf);
	if (priv->rx_time_pdel_resp.buf)
		vfree(priv->rx_time_pdel_resp.buf);

	ptp_free();
}
EXPORT_SYMBOL(fec_ptp_cleanup);
