
// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADI UART4 Serial Driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/clk.h>
#include <linux/compiler.h>
#include <linux/console.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/sysrq.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/io.h>
#include <linux/circ_buf.h>

#include <linux/soc/adi/uart4.h>

#if defined(CONFIG_SERIAL_ADI_UART4_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#define DRIVER_NAME "adi-uart4"

struct adi_uart4_serial_port {
	struct uart_port port;
	struct device *dev;
	unsigned int old_status;
	int tx_irq;
	int rx_irq;
	int status_irq;
	unsigned int lsr;
	/* DMA specific fields */
	int tx_done;
	int tx_count;
	dma_addr_t tx_dma_phy;
	struct scatterlist tx_sgl;
	struct circ_buf rx_dma_buf;
	dma_addr_t rx_dma_phy;
	dma_cookie_t rx_cookie;
	struct timer_list rx_dma_timer;
	spinlock_t rx_lock;
	struct dma_chan *tx_dma_channel;
	struct dma_chan *rx_dma_channel;
	/* Hardware flow control specific fields */
	unsigned int hwflow_mode;
	struct gpio_desc *hwflow_en_pin;
	bool hwflow_en;
	/* Use enable-divide-by-one in divisor? */
	bool edbo;
	struct clk *clk;
};

struct adi_uart4_serial_port *to_adi_serial_port(struct uart_port *port)
{
	return container_of(port, struct adi_uart4_serial_port, port);
}

#define ADI_UART_NO_HWFLOW	0
#define ADI_UART_HWFLOW_PERI	1

#define ADI_UART_NR_PORTS 4
static struct adi_uart4_serial_port *adi_uart4_serial_ports[ADI_UART_NR_PORTS];

/* UART_CTL Masks */
#define UCEN                     0x1  /* Enable UARTx Clocks */
#define LOOP_ENA                 0x2  /* Loopback Mode Enable */
#define UMOD_MDB                 0x10  /* Enable MDB Mode */
#define UMOD_IRDA                0x20  /* Enable IrDA Mode */
#define UMOD_MASK                0x30  /* Uart Mode Mask */
#define WLS(x)                   (((x-5) & 0x03) << 8)  /* Word Length Select */
#define WLS_MASK                 0x300  /* Word length Select Mask */
#define WLS_OFFSET               8      /* Word length Select Offset */
#define STB                      0x1000  /* Stop Bits */
#define STBH                     0x2000  /* Half Stop Bits */
#define PEN                      0x4000  /* Parity Enable */
#define EPS                      0x8000  /* Even Parity Select */
#define STP                      0x10000  /* Stick Parity */
#define FPE                      0x20000  /* Force Parity Error On Transmit */
#define FFE                      0x40000  /* Force Framing Error On Transmit */
#define SB                       0x80000  /* Set Break */
#define LCR_MASK		 (SB | STP | EPS | PEN | STB | WLS_MASK)
#define FCPOL                    0x400000  /* Flow Control Pin Polarity */
#define RPOLC                    0x800000  /* IrDA RX Polarity Change */
#define TPOLC                    0x1000000  /* IrDA TX Polarity Change */
#define MRTS                     0x2000000  /* Manual Request To Send */
#define XOFF                     0x4000000  /* Transmitter Off */
#define ARTS                     0x8000000  /* Automatic Request To Send */
#define ACTS                     0x10000000  /* Automatic Clear To Send */
#define RFIT                     0x20000000  /* Receive FIFO IRQ Threshold */
#define RFRT                     0x40000000  /* Receive FIFO RTS Threshold */

/* UART_STAT Masks */
#define DR                       0x01  /* Data Ready */
#define OE                       0x02  /* Overrun Error */
#define PE                       0x04  /* Parity Error */
#define FE                       0x08  /* Framing Error */
#define BI                       0x10  /* Break Interrupt */
#define THRE                     0x20  /* THR Empty */
#define TEMT                     0x80  /* TSR and UART_THR Empty */
#define TFI                      0x100  /* Transmission Finished Indicator */

#define ASTKY                    0x200  /* Address Sticky */
#define ADDR                     0x400  /* Address bit status */
#define RO			 0x800  /* Reception Ongoing */
#define SCTS                     0x1000  /* Sticky CTS */
#define CTS                      0x10000  /* Clear To Send */
#define RFCS                     0x20000  /* Receive FIFO Count Status */

/* UART_CLOCK Masks */
#define EDBO                     0x80000000 /* Enable Devide by One */

/* UART_IER Masks */
#define ERBFI                    0x01  /* Enable Receive Buffer Full Interrupt */
#define ETBEI                    0x02  /* Enable Transmit Buffer Empty Interrupt */
#define ELSI                     0x04  /* Enable RX Status Interrupt */
#define EDSSI                    0x08  /* Enable Modem Status Interrupt */
#define EDTPTI                   0x10  /* Enable DMA Transmit PIRQ Interrupt */
#define ETFI                     0x20  /* Enable Transmission Finished Interrupt */
#define ERFCI                    0x40  /* Enable Receive FIFO Count Interrupt */

# define OFFSET_REDIV            0x00  /* Version ID Register             */
# define OFFSET_CTL              0x04  /* Control Register                */
# define OFFSET_STAT             0x08  /* Status Register                 */
# define OFFSET_SCR              0x0C  /* SCR Scratch Register            */
# define OFFSET_CLK              0x10  /* Clock Rate Register             */
# define OFFSET_IER              0x14  /* Interrupt Enable Register       */
# define OFFSET_IER_SET          0x18  /* Set Interrupt Enable Register   */
# define OFFSET_IER_CLEAR        0x1C  /* Clear Interrupt Enable Register */
# define OFFSET_RBR              0x20  /* Receive Buffer register         */
# define OFFSET_THR              0x24  /* Transmit Holding register       */

#define UART_GET_CHAR(p)      readl(p->port.membase + OFFSET_RBR)
#define UART_GET_CLK(p)       readl(p->port.membase + OFFSET_CLK)
#define UART_GET_CTL(p)       readl(p->port.membase + OFFSET_CTL)
#define UART_GET_GCTL(p)      UART_GET_CTL(p)
#define UART_GET_LCR(p)       UART_GET_CTL(p)
#define UART_GET_MCR(p)       UART_GET_CTL(p)
#define UART_GET_STAT(p)      readl(p->port.membase + OFFSET_STAT)
#define UART_GET_MSR(p)       UART_GET_STAT(p)

#define UART_PUT_CHAR(p, v)   writel(v, p->port.membase + OFFSET_THR)
#define UART_PUT_CLK(p, v)    writel(v, p->port.membase + OFFSET_CLK)
#define UART_PUT_CTL(p, v)    writel(v, p->port.membase + OFFSET_CTL)
#define UART_PUT_GCTL(p, v)   UART_PUT_CTL(p, v)
#define UART_PUT_LCR(p, v)    UART_PUT_CTL(p, v)
#define UART_PUT_MCR(p, v)    UART_PUT_CTL(p, v)
#define UART_PUT_STAT(p, v)   writel(v, p->port.membase + OFFSET_STAT)

#define UART_CLEAR_IER(p, v)  writel(v, p->port.membase + OFFSET_IER_CLEAR)
#define UART_GET_IER(p)       readl(p->port.membase + OFFSET_IER)
#define UART_SET_IER(p, v)    writel(v, p->port.membase + OFFSET_IER_SET)

#define UART_CLEAR_LSR(p)     UART_PUT_STAT(p, -1)
#define UART_GET_LSR(p)       UART_GET_STAT(p)
#define UART_PUT_LSR(p, v)    UART_PUT_STAT(p, v)

/* This handles hard CTS/RTS */
#define UART_CLEAR_SCTS(p)      UART_PUT_STAT(p, SCTS)
#define UART_GET_CTS(x)         (UART_GET_MSR(x) & CTS)
#define UART_DISABLE_RTS(x)     UART_PUT_MCR(x, UART_GET_MCR(x) & ~(ARTS | MRTS))
#define UART_ENABLE_RTS(x)      UART_PUT_MCR(x, UART_GET_MCR(x) | MRTS | ARTS)
#define UART_ENABLE_INTS(x, v)  UART_SET_IER(x, v)
#define UART_DISABLE_INTS(x)    UART_CLEAR_IER(x, 0xF)

#define DMA_RX_XCOUNT		512
#define DMA_RX_YCOUNT		(PAGE_SIZE / DMA_RX_XCOUNT)

#define DMA_RX_FLUSH_JIFFIES	(msecs_to_jiffies(50))

#define uart_circ_chars_pending(circ)	\
	(CIRC_CNT((circ)->head, (circ)->tail, UART_XMIT_SIZE))

#define uart_circ_empty(circ)		((circ)->head == (circ)->tail)

static void adi_uart4_serial_dma_tx_chars(struct adi_uart4_serial_port *uart);
static void adi_uart4_serial_dma_tx(void *data);
static void adi_uart4_serial_tx_chars(struct adi_uart4_serial_port *uart);
static void adi_uart4_serial_reset_irda(struct uart_port *port);

static unsigned int adi_uart4_serial_get_mctrl(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);

	if (!uart->hwflow_mode || !uart->hwflow_en)
		return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;

	/* CTS PIN is negative assertive. */
	if (UART_GET_CTS(uart))
		return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
	else
		return TIOCM_DSR | TIOCM_CAR;
}

static void adi_uart4_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);

	if (!uart->hwflow_mode || !uart->hwflow_en)
		return;

	/* RTS PIN is negative assertive. */
	if (mctrl & TIOCM_RTS)
		UART_ENABLE_RTS(uart);
	else
		UART_DISABLE_RTS(uart);
}

/*
 * Handle any change of modem status signal.
 */
static irqreturn_t adi_uart4_serial_mctrl_cts_int(int irq, void *dev_id)
{
	struct adi_uart4_serial_port *uart = dev_id;
	unsigned int status = adi_uart4_serial_get_mctrl(&uart->port);
	struct tty_struct *tty = uart->port.state->port.tty;

	if (uart->hwflow_mode == ADI_UART_HWFLOW_PERI) {
		UART_CLEAR_SCTS(uart);
		if (tty->hw_stopped) {
			if (status) {
				tty->hw_stopped = 0;
				uart_write_wakeup(&uart->port);
			}
		} else {
			if (!status)
				tty->hw_stopped = 1;
		}
	}

	uart_handle_cts_change(&uart->port, status & TIOCM_CTS);

	return IRQ_HANDLED;
}

/*
 * interrupts are disabled on entry
 */
static void adi_uart4_serial_stop_tx(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);

	while (!(UART_GET_LSR(uart) & TEMT))
		cpu_relax();

	if (uart->tx_dma_channel) {
		if (!uart->tx_done)
			dma_unmap_sg(uart->dev, &uart->tx_sgl, 1, DMA_TO_DEVICE);
		dmaengine_terminate_sync(uart->tx_dma_channel);
		uart->port.icount.tx += uart->tx_count;
		uart->tx_count = 0;
		uart->tx_done = 1;
	} else {
		/* Clear TFI bit */
		UART_PUT_LSR(uart, TFI);
		UART_CLEAR_IER(uart, ETBEI);
	}
}

/*
 * port is locked and interrupts are disabled
 */
static void adi_uart4_serial_start_tx(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);
	struct tty_struct *tty = uart->port.state->port.tty;

	/*
	 * To avoid losting RX interrupt, we reset IR function
	 * before sending data.
	 */
	if (tty->termios.c_line == N_IRDA)
		adi_uart4_serial_reset_irda(port);

	if (uart->tx_dma_channel) {
		if (uart->tx_done)
			adi_uart4_serial_dma_tx_chars(uart);
	} else {
		UART_SET_IER(uart, ETBEI);
		adi_uart4_serial_tx_chars(uart);
	}
}

/*
 * Interrupts are enabled
 */
static void adi_uart4_serial_stop_rx(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);

	UART_CLEAR_IER(uart, ERBFI);
}

/*
 * Set the modem control timer to fire immediately.
 */
static void adi_uart4_serial_enable_ms(struct uart_port *port)
{
}

static void adi_uart4_serial_rx_chars(struct adi_uart4_serial_port *uart)
{
	unsigned int status, ch, flg;

	status = UART_GET_LSR(uart);
	UART_CLEAR_LSR(uart);

	ch = UART_GET_CHAR(uart);
	uart->port.icount.rx++;

	if (status & BI) {
		uart->port.icount.brk++;
		if (uart_handle_break(&uart->port))
			goto ignore_char;
		status &= ~(PE | FE);
	}
	if (status & PE)
		uart->port.icount.parity++;
	if (status & OE)
		uart->port.icount.overrun++;
	if (status & FE)
		uart->port.icount.frame++;

	status &= uart->port.read_status_mask;

	if (status & BI)
		flg = TTY_BREAK;
	else if (status & PE)
		flg = TTY_PARITY;
	else if (status & FE)
		flg = TTY_FRAME;
	else
		flg = TTY_NORMAL;

	if (uart_handle_sysrq_char(&uart->port, ch))
		goto ignore_char;

	uart_insert_char(&uart->port, status, OE, ch, flg);

 ignore_char:
	tty_flip_buffer_push(&uart->port.state->port);
}

static void adi_uart4_serial_tx_chars(struct adi_uart4_serial_port *uart)
{
	struct tty_port *tport = &uart->port.state->port;
	unsigned char c;

	if (kfifo_is_empty(&tport->xmit_fifo) || uart_tx_stopped(&uart->port)) {
		/* Clear TFI bit */
		UART_PUT_LSR(uart, TFI);
		/* Anomaly notes:
		 *  05000215 -	we always clear ETBEI within last UART TX
		 *		interrupt to end a string. It is always set
		 *		when start a new tx.
		 */
		UART_CLEAR_IER(uart, ETBEI);
		return;
	}

	if (uart->port.x_char) {
		UART_PUT_CHAR(uart, uart->port.x_char);
		uart->port.icount.tx++;
		uart->port.x_char = 0;
	}

	if (UART_GET_LSR(uart) & THRE) {
		/*
		 * pop data from fifo
		 * */
		if (!kfifo_get(&tport->xmit_fifo, &c))
			return;
		UART_PUT_CHAR(uart, c);
		uart->port.icount.tx++;
		if (kfifo_len(&tport->xmit_fifo) < WAKEUP_CHARS)
			uart_write_wakeup(&uart->port);
	}


}

static irqreturn_t adi_uart4_serial_rx_int(int irq, void *dev_id)
{
	struct adi_uart4_serial_port *uart = dev_id;

	while (UART_GET_LSR(uart) & DR)
		adi_uart4_serial_rx_chars(uart);

	return IRQ_HANDLED;
}

static irqreturn_t adi_uart4_serial_tx_int(int irq, void *dev_id)
{
	struct adi_uart4_serial_port *uart = dev_id;

	spin_lock(&uart->port.lock);
	if (UART_GET_LSR(uart) & THRE)
		adi_uart4_serial_tx_chars(uart);
	spin_unlock(&uart->port.lock);

	return IRQ_HANDLED;
}

static void adi_uart4_serial_dma_tx_chars(struct adi_uart4_serial_port *uart)
{
	struct dma_async_tx_descriptor *desc;
	struct tty_port *tport = &uart->port.state->port;
	int i, cnt, ret;

	uart->tx_done = 0;

	if (kfifo_is_empty(&tport->xmit_fifo) || uart_tx_stopped(&uart->port)) {
		uart->tx_count = 0;
		uart->tx_done = 1;
		return;
	}

	if (uart->port.x_char) {
		UART_PUT_CHAR(uart, uart->port.x_char);
		uart->port.icount.tx++;
		uart->port.x_char = 0;
	}

	/*
	 * Limit size to UART_XMIT_SIZE
	 * */
//	if (uart->tx_count > (UART_XMIT_SIZE))
//		uart->tx_count = UART_XMIT_SIZE;

	/*
	 * Prepare dma transfer
	 * */

//	for (i=0; i<uart->tx_count;) {
//		cnt = kfifo_get(&tport->xmit_fifo, &tport->xmit_buf[i]);
//		if (!cnt)
//			return;
//		i += cnt;
//	}


	sg_init_table(&uart->tx_sgl, 1);
	ret = kfifo_dma_out_prepare_mapped(&tport->xmit_fifo, &uart->tx_sgl, 1,
			UART_XMIT_SIZE, uart->tx_dma_phy);
	if (ret != 1)
		return;
//	sg_init_one(&uart->tx_sgl, &tport->xmit_buf, uart->tx_count);
//	dma_map_sg(uart->dev, &uart->tx_sgl, 1, DMA_TO_DEVICE);
//
	desc = dmaengine_prep_slave_sg(uart->tx_dma_channel, &uart->tx_sgl, 1,
		DMA_MEM_TO_DEV, 0);
	if (!desc) {
		dma_unmap_sg(uart->dev, &uart->tx_sgl, 1, DMA_TO_DEVICE);
		return;
	}

	uart->tx_count = sg_dma_len(&uart->tx_sgl);

	desc->callback = adi_uart4_serial_dma_tx;
	desc->callback_param = uart;

	/*
	 * Queue DMA transfer
	 * */
	dmaengine_submit(desc);
	dma_sync_single_for_device(uart->dev, uart->tx_dma_phy,
				   UART_XMIT_SIZE, DMA_TO_DEVICE);
	dma_async_issue_pending(uart->tx_dma_channel);
	UART_SET_IER(uart, ETBEI);
}

static void adi_uart4_serial_dma_rx_chars(struct adi_uart4_serial_port *uart)
{
	int i, flg, status;

	mod_timer(&uart->rx_dma_timer, jiffies + DMA_RX_FLUSH_JIFFIES);

	status = UART_GET_LSR(uart);
	UART_CLEAR_LSR(uart);

	uart->port.icount.rx +=
		CIRC_CNT(uart->rx_dma_buf.head, uart->rx_dma_buf.tail,
		UART_XMIT_SIZE);

	if (status & BI) {
		uart->port.icount.brk++;
		if (uart_handle_break(&uart->port))
			goto dma_ignore_char;
		status &= ~(PE | FE);
	}
	if (status & PE)
		uart->port.icount.parity++;
	if (status & OE)
		uart->port.icount.overrun++;
	if (status & FE)
		uart->port.icount.frame++;

	status &= uart->port.read_status_mask;

	if (status & BI)
		flg = TTY_BREAK;
	else if (status & PE)
		flg = TTY_PARITY;
	else if (status & FE)
		flg = TTY_FRAME;
	else
		flg = TTY_NORMAL;

	for (i = uart->rx_dma_buf.tail; ; i++) {
		if (i >= UART_XMIT_SIZE)
			i = 0;
		if (i == uart->rx_dma_buf.head)
			break;
		if (!uart_handle_sysrq_char(&uart->port, uart->rx_dma_buf.buf[i]))
			uart_insert_char(&uart->port, status, OE,
				uart->rx_dma_buf.buf[i], flg);
	}

 dma_ignore_char:
	tty_flip_buffer_push(&uart->port.state->port);
}

void adi_uart4_serial_rx_dma_timeout(struct timer_list *list)
{
	struct adi_uart4_serial_port *uart =
		container_of(list, struct adi_uart4_serial_port, rx_dma_timer);
	struct dma_tx_state state;
	enum dma_status status;
	unsigned long flags;

	dmaengine_pause(uart->rx_dma_channel);
	spin_lock_irqsave(&uart->rx_lock, flags);

	status = dmaengine_tx_status(uart->rx_dma_channel, uart->rx_cookie, &state);

	if (status == DMA_ERROR) {
		dev_err(uart->dev, "Error in RX DMA\n");
		goto exit;
	}

	// Because resume will reset us to the start of the buffer, reset the tail
	// pointer to 0 after, but use the previous tail for an offset buffer slice
	// that timed out
	uart->rx_dma_buf.head = UART_XMIT_SIZE - state.residue;
	adi_uart4_serial_dma_rx_chars(uart);
	uart->rx_dma_buf.tail = 0;

exit:
	spin_unlock_irqrestore(&uart->rx_lock, flags);
	dmaengine_resume(uart->rx_dma_channel);
}

static void adi_uart4_serial_dma_tx(void *data)
{
	struct adi_uart4_serial_port *uart = data;
	struct tty_port *tport = &uart->port.state->port;
	unsigned long flags;

	dma_unmap_sg(uart->dev, &uart->tx_sgl, 1, DMA_TO_DEVICE);

	spin_lock_irqsave(&uart->port.lock, flags);
	/* Anomaly notes:
	 *  05000215 -	we always clear ETBEI within last UART TX
	 *		interrupt to end a string. It is always set
	 *		when start a new tx.
	 */
	UART_CLEAR_IER(uart, ETBEI);
	uart->port.icount.tx += uart->tx_count;
	//kfifo_skip_count(&tport->xmit_fifo, uart->tx_count);

	/*
	 * If kfifo is not empty, wakeup and write
	 * */
	if (!kfifo_is_empty(&tport->xmit_fifo)) {
		if (kfifo_len(&tport->xmit_fifo) < WAKEUP_CHARS)
			uart_write_wakeup(&uart->port);
	}
	
	uart_xmit_advance(&uart->port, uart->tx_count);
	adi_uart4_serial_dma_tx_chars(uart);
	spin_unlock_irqrestore(&uart->port.lock, flags);
}

static void adi_uart4_serial_dma_rx(void *data)
{
	struct adi_uart4_serial_port *uart = data;
	struct dma_tx_state state;
	enum dma_status status;
	unsigned long flags;
	int pos;

	spin_lock_irqsave(&uart->rx_lock, flags);

	status = dmaengine_tx_status(uart->rx_dma_channel, uart->rx_cookie, &state);

	if (status == DMA_ERROR) {
		dev_err(uart->dev, "Error in RX DMA\n");
		spin_unlock_irqrestore(&uart->rx_lock, flags);
		return;
	}

	// Update tail to start of the current block, so that we can receive multiple
	// full blocks or a partial block not at the start of the buffer in event of
	// a timeout
	uart->rx_dma_buf.head = UART_XMIT_SIZE - state.residue;
	pos = (uart->rx_dma_buf.head - 1) & (UART_XMIT_SIZE - 1);
	pos = (pos / DMA_RX_XCOUNT) * DMA_RX_XCOUNT;
	uart->rx_dma_buf.tail = pos;
	adi_uart4_serial_dma_rx_chars(uart);

	spin_unlock_irqrestore(&uart->rx_lock, flags);
}

/*
 * Return TIOCSER_TEMT when transmitter is not busy.
 */
static unsigned int adi_uart4_serial_tx_empty(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);
	unsigned int lsr;

	lsr = UART_GET_LSR(uart);
	if (lsr & TEMT)
		return TIOCSER_TEMT;
	else
		return 0;
}

static void adi_uart4_serial_break_ctl(struct uart_port *port, int break_state)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);
	u32 lcr = UART_GET_LCR(uart);

	if (break_state)
		lcr |= SB;
	else
		lcr &= ~SB;
	UART_PUT_LCR(uart, lcr);
}

static int adi_uart4_serial_startup(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);
	struct tty_port *tport = &uart->port.state->port;
	struct dma_slave_config dma_config = {0};
	struct dma_async_tx_descriptor *desc;
	int ret;

	uart->tx_done = 1;

	ret = clk_prepare_enable(uart->clk);
	if (ret)
		return ret;

	if (uart->tx_dma_channel) {
		/*
		 * Setup rx dma buf
		 * */
		uart->rx_dma_buf.buf = dmam_alloc_coherent(uart->dev, UART_XMIT_SIZE,
			&uart->rx_dma_phy, GFP_KERNEL);
		if (!uart->rx_dma_buf.buf)
			return -ENOMEM;

		uart->rx_dma_buf.head = 0;
		uart->rx_dma_buf.tail = 0;

		dma_config.direction = DMA_DEV_TO_MEM;
		dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		// src_addr is not configured because we're attached to peripheral
		dma_config.src_maxburst = 1;
		dma_config.dst_maxburst = 1;

		ret = dmaengine_slave_config(uart->rx_dma_channel, &dma_config);
		if (ret) {
			dev_err(uart->dev, "Error configuring RX DMA channel\n");
			return -EINVAL;
		}

		desc = dmaengine_prep_dma_cyclic(uart->rx_dma_channel, uart->rx_dma_phy,
			UART_XMIT_SIZE, DMA_RX_XCOUNT, DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
		desc->callback = adi_uart4_serial_dma_rx;
		desc->callback_param = uart;

		uart->rx_cookie = dmaengine_submit(desc);
		dma_async_issue_pending(uart->rx_dma_channel);

		timer_setup(&uart->rx_dma_timer, adi_uart4_serial_rx_dma_timeout, 0);
		mod_timer(&uart->rx_dma_timer, jiffies + DMA_RX_FLUSH_JIFFIES);

		// TX channel
		/*
		 * Setup tx dma buf
		 * */
		uart->tx_dma_phy = dma_map_single(uart->dev,
				tport->xmit_buf,
				UART_XMIT_SIZE,
				DMA_TO_DEVICE);
		if (dma_mapping_error(uart->dev, uart->tx_dma_phy))
			return -ENOMEM;

		uart->port.fifosize = UART_XMIT_SIZE;

		dma_config = (struct dma_slave_config) {0};
		dma_config.direction = DMA_MEM_TO_DEV;
		dma_config.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		dma_config.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		// dst_addr is not configured because we're attached to peripheral
		dma_config.dst_maxburst = 1;
		dma_config.src_maxburst = 1;

		ret = dmaengine_slave_config(uart->tx_dma_channel, &dma_config);
		if (ret) {
			dev_err(uart->dev, "Error configuring TX DMA channel\n");
			return -EINVAL;
		}
	}

	if (uart->hwflow_mode == ADI_UART_HWFLOW_PERI) {
		/* CTS RTS PINs are negative assertive. */
		UART_PUT_MCR(uart, UART_GET_MCR(uart) | ACTS);
		UART_SET_IER(uart, EDSSI);
	}

	UART_SET_IER(uart, ERBFI);
	return 0;
}

static void adi_uart4_serial_shutdown(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);

	dev_dbg(uart->dev, "in serial_shutdown\n");

	if (uart->tx_dma_channel) {
		dmaengine_terminate_sync(uart->tx_dma_channel);
		dmaengine_terminate_sync(uart->rx_dma_channel);
		dma_unmap_single(uart->dev, uart->tx_dma_phy, UART_XMIT_SIZE, DMA_TO_DEVICE);
		dmam_free_coherent(uart->dev, UART_XMIT_SIZE, uart->rx_dma_buf.buf,
			uart->rx_dma_phy);
		del_timer(&uart->rx_dma_timer);
	}

	clk_disable_unprepare(uart->clk);
}

static void adi_uart4_serial_set_termios(struct uart_port *port,
		struct ktermios *termios, const struct ktermios *old)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);
	unsigned long flags;
	unsigned int baud, quot;
	unsigned int ier, lcr = 0;
	unsigned long timeout;

	if (uart->hwflow_mode == ADI_UART_HWFLOW_PERI)
		termios->c_cflag |= CRTSCTS;

	switch (termios->c_cflag & CSIZE) {
	case CS8:
		lcr = WLS(8);
		break;
	case CS7:
		lcr = WLS(7);
		break;
	case CS6:
		lcr = WLS(6);
		break;
	case CS5:
		lcr = WLS(5);
		break;
	default:
		dev_err(port->dev, "%s: word length not supported\n",
			__func__);
	}

	if (termios->c_cflag & CSTOPB)
		lcr |= STB;
	if (termios->c_cflag & PARENB)
		lcr |= PEN;
	if (!(termios->c_cflag & PARODD))
		lcr |= EPS;
	if (termios->c_cflag & CMSPAR)
		lcr |= STP;
	if (termios->c_cflag & CRTSCTS)
		uart->hwflow_en = true;
	else
		uart->hwflow_en = false;

	spin_lock_irqsave(&uart->port.lock, flags);

	port->read_status_mask = OE;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= (FE | PE);
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= BI;

	/*
	 * Characters to ignore
	 */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= FE | PE;
	if (termios->c_iflag & IGNBRK) {
		port->ignore_status_mask |= BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= OE;
	}

	/*
	 * uart_get_divisor has a hardcoded /16 factor that will cause integer
	 * round off errors if we're in divide-by-one mode
	 */
	if (uart->edbo) {
		baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk);
		quot = EDBO | DIV_ROUND_CLOSEST(port->uartclk, baud);
	} else {
		baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
		quot = uart_get_divisor(port, baud);
	}

	/* Wait till the transfer buffer is empty */
	timeout = jiffies + msecs_to_jiffies(10);
	while (UART_GET_GCTL(uart) & UCEN && !(UART_GET_LSR(uart) & TEMT))
		if (time_after(jiffies, timeout)) {
			dev_warn(port->dev,
				"timeout waiting for TX buffer empty\n");
			break;
		}

	/* Wait till the transfer buffer is empty */
	timeout = jiffies + msecs_to_jiffies(10);
	while (UART_GET_GCTL(uart) & UCEN && !(UART_GET_LSR(uart) & TEMT))
		if (time_after(jiffies, timeout)) {
			dev_warn(port->dev,
				"timeout waiting for TX buffer empty\n");
			break;
		}

	/* Disable UART */
	ier = UART_GET_IER(uart);
	UART_PUT_GCTL(uart, UART_GET_GCTL(uart) & ~UCEN);
	UART_DISABLE_INTS(uart);

	UART_PUT_CLK(uart, quot);

	UART_PUT_LCR(uart, (UART_GET_LCR(uart) & ~LCR_MASK) | lcr);

	/* Enable UART */
	UART_ENABLE_INTS(uart, ier);
	UART_PUT_GCTL(uart, UART_GET_GCTL(uart) | UCEN);

	/* Port speed changed, update the per-port timeout. */
	uart_update_timeout(port, termios->c_cflag, baud);

	spin_unlock_irqrestore(&uart->port.lock, flags);
}

static const char *adi_uart4_serial_type(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);

	return uart->port.type == PORT_BFIN ? "ADI-UART4" : NULL;
}

/*
 * Release the memory region(s) being used by 'port'.
 */
static void adi_uart4_serial_release_port(struct uart_port *port)
{
}

/*
 * Request the memory region(s) being used by 'port'.
 */
static int adi_uart4_serial_request_port(struct uart_port *port)
{
	return 0;
}

/*
 * Configure/autoconfigure the port.
 */
static void adi_uart4_serial_config_port(struct uart_port *port, int flags)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);

	if (flags & UART_CONFIG_TYPE &&
	    adi_uart4_serial_request_port(&uart->port) == 0)
		uart->port.type = PORT_BFIN;
}

/*
 * Verify the new serial_struct (for TIOCSSERIAL).
 * The only change we allow are to the flags and type, and
 * even then only between PORT_BFIN and PORT_UNKNOWN
 */
static int
adi_uart4_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	return 0;
}

/*
 * Enable the IrDA function if tty->ldisc.num is N_IRDA.
 * In other cases, disable IrDA function.
 */
static void adi_uart4_serial_set_ldisc(struct uart_port *port, struct ktermios *termios)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);
	unsigned int val;

	switch (termios->c_line) {
	case N_IRDA:
		val = UART_GET_GCTL(uart);
		val |= (UMOD_IRDA | RPOLC);
		UART_PUT_GCTL(uart, val);
		break;
	default:
		val = UART_GET_GCTL(uart);
		val &= ~(UMOD_MASK | RPOLC);
		UART_PUT_GCTL(uart, val);
	}
}

static void adi_uart4_serial_reset_irda(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);
	unsigned int val;

	val = UART_GET_GCTL(uart);
	val &= ~(UMOD_MASK | RPOLC);
	UART_PUT_GCTL(uart, val);
	val |= (UMOD_IRDA | RPOLC);
	UART_PUT_GCTL(uart, val);
}

#ifdef CONFIG_CONSOLE_POLL
static void adi_uart4_serial_poll_put_char(struct uart_port *port, unsigned char chr)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);

	while (!(UART_GET_LSR(uart) & THRE))
		cpu_relax();

	UART_PUT_CHAR(uart, (unsigned char)chr);
}

static int adi_uart4_serial_poll_get_char(struct uart_port *port)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);
	unsigned char chr;

	while (!(UART_GET_LSR(uart) & DR))
		cpu_relax();

	chr = UART_GET_CHAR(uart);

	return chr;
}
#endif


static const struct uart_ops adi_uart4_serial_pops = {
	.tx_empty	= adi_uart4_serial_tx_empty,
	.set_mctrl	= adi_uart4_serial_set_mctrl,
	.get_mctrl	= adi_uart4_serial_get_mctrl,
	.stop_tx	= adi_uart4_serial_stop_tx,
	.start_tx	= adi_uart4_serial_start_tx,
	.stop_rx	= adi_uart4_serial_stop_rx,
	.enable_ms	= adi_uart4_serial_enable_ms,
	.break_ctl	= adi_uart4_serial_break_ctl,
	.startup	= adi_uart4_serial_startup,
	.shutdown	= adi_uart4_serial_shutdown,
	.set_termios	= adi_uart4_serial_set_termios,
	.set_ldisc	= adi_uart4_serial_set_ldisc,
	.type		= adi_uart4_serial_type,
	.release_port	= adi_uart4_serial_release_port,
	.request_port	= adi_uart4_serial_request_port,
	.config_port	= adi_uart4_serial_config_port,
	.verify_port	= adi_uart4_serial_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_put_char	= adi_uart4_serial_poll_put_char,
	.poll_get_char	= adi_uart4_serial_poll_get_char,
#endif
};

#ifdef CONFIG_SERIAL_ADI_UART4_CONSOLE
static void adi_uart4_serial_console_putchar(struct uart_port *port, unsigned char ch)
{
	struct adi_uart4_serial_port *uart = to_adi_serial_port(port);

	while (!(UART_GET_LSR(uart) & THRE))
		barrier();
	UART_PUT_CHAR(uart, ch);
}

static void __init
adi_uart4_serial_console_get_options(struct adi_uart4_serial_port *uart, int *baud,
			   int *parity, int *bits)
{
	unsigned int status;

	status = UART_GET_IER(uart) & (ERBFI | ETBEI);
	if (status == (ERBFI | ETBEI)) {
		/* ok, the port was enabled */
		u32 lcr, clk;

		lcr = UART_GET_LCR(uart);

		*parity = 'n';
		if (lcr & PEN) {
			if (lcr & EPS)
				*parity = 'e';
			else
				*parity = 'o';
		}
		*bits = ((lcr & WLS_MASK) >> WLS_OFFSET) + 5;

		clk = UART_GET_CLK(uart);

		/* Only the lowest 16 bits are the divisor */
		if (clk & EDBO)
			*baud = uart->port.uartclk / (clk & 0xffff);
		else
			*baud = uart->port.uartclk / (16*clk);
	}
	pr_debug("%s:baud = %d, parity = %c, bits= %d\n", __func__, *baud, *parity, *bits);
}
static void
adi_uart4_serial_console_write(struct console *co, const char *s, unsigned int count)
{
	struct adi_uart4_serial_port *uart = adi_uart4_serial_ports[co->index];
	unsigned long flags;

	spin_lock_irqsave(&uart->port.lock, flags);
	uart_console_write(&uart->port, s, count, adi_uart4_serial_console_putchar);
	spin_unlock_irqrestore(&uart->port.lock, flags);

}

static int __init
adi_uart4_serial_console_setup(struct console *co, char *options)
{
	struct adi_uart4_serial_port *uart;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (co->index < 0 || co->index >= ADI_UART_NR_PORTS)
		return -ENODEV;

	uart = adi_uart4_serial_ports[co->index];
	if (!uart)
		return -ENODEV;

	if (uart->hwflow_mode == ADI_UART_HWFLOW_PERI)
		flow = 'r';

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);
	else
		adi_uart4_serial_console_get_options(uart, &baud, &parity, &bits);

	return uart_set_options(&uart->port, co, baud, parity, bits, flow);
}

static struct uart_driver adi_uart4_serial_reg;

static struct console adi_uart4_serial_console = {
	.name		= "ttySC",
	.write		= adi_uart4_serial_console_write,
	.device		= uart_console_device,
	.setup		= adi_uart4_serial_console_setup,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.data		= &adi_uart4_serial_reg,
};


#define ADI_SERIAL_UART4_CONSOLE (&adi_uart4_serial_console)
#else
#define ADI_SERIAL_UART4_CONSOLE NULL
#endif

static struct uart_driver adi_uart4_serial_reg = {
	.owner			= THIS_MODULE,
	.driver_name		= DRIVER_NAME,
	.dev_name		= "ttySC",
	.major			= TTY_MAJOR,
#ifdef CONFIG_ARCH_SC59X_64
	// Other serial drivers are using 64 --
	// Can probably disable in the future and set this back to 64
	.minor			= 74,
#else
	.minor			= 64,
#endif
	.nr			= ADI_UART_NR_PORTS,
	.cons			= ADI_SERIAL_UART4_CONSOLE,
};

static int adi_uart4_serial_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct adi_uart4_serial_port *uart = platform_get_drvdata(pdev);

	clk_disable(uart->clk);
	return uart_suspend_port(&adi_uart4_serial_reg, &uart->port);
}

static int adi_uart4_serial_resume(struct platform_device *pdev)
{
	struct adi_uart4_serial_port *uart = platform_get_drvdata(pdev);
	int ret;

	ret = clk_enable(uart->clk);
	if (ret)
		return ret;

	return uart_resume_port(&adi_uart4_serial_reg, &uart->port);
}

static int adi_uart4_serial_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;
	struct adi_uart4_serial_port *uart = NULL;
	int ret = 0;
	int uartid;
	struct dma_chan *tx_dma_channel = NULL;
	struct dma_chan *rx_dma_channel = NULL;

	dev_info(dev, "Serial probe\n");

	uartid = of_alias_get_id(np, "serial");
	tx_dma_channel = dma_request_chan(dev, "tx");
	rx_dma_channel = dma_request_chan(dev, "rx");

	if (uartid < 0) {
		dev_err(&pdev->dev, "failed to get alias/pdev id, errno %d\n",
				uartid);
		ret = -ENODEV;
		return ret;
	}

	if (adi_uart4_serial_ports[uartid] == NULL) {
		uart = kzalloc(sizeof(*uart), GFP_KERNEL);
		if (!uart)
			return -ENOMEM;

		adi_uart4_serial_ports[uartid] = uart;
		uart->dev = &pdev->dev;

		uart->clk = devm_clk_get(dev, "sclk0");
		if (IS_ERR(uart->clk))
			return -ENODEV;

		spin_lock_init(&uart->port.lock);
		uart->port.uartclk   = clk_get_rate(uart->clk);
		uart->port.fifosize  = 8;
		uart->port.ops       = &adi_uart4_serial_pops;
		uart->port.line      = uartid;
		uart->port.iotype    = UPIO_MEM;
		uart->port.flags     = UPF_BOOT_AUTOCONF;

		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (res == NULL) {
			dev_err(&pdev->dev, "Cannot get IORESOURCE_MEM\n");
			ret = -ENOENT;
			goto out_error_unmap;
		}

		uart->port.membase = devm_ioremap(&pdev->dev, res->start,
						resource_size(res));
		if (!uart->port.membase) {
			dev_err(&pdev->dev, "Cannot map uart IO\n");
			return -ENXIO;
		}

		uart->tx_dma_channel = tx_dma_channel;
		uart->rx_dma_channel = rx_dma_channel;
		spin_lock_init(&uart->rx_lock);
		uart->tx_done	    = 1;
		uart->tx_count	    = 0;

		if (!tx_dma_channel) {
			uart->tx_irq = platform_get_irq_byname(pdev, "tx");
			uart->rx_irq = platform_get_irq_byname(pdev, "rx");
			uart->status_irq = platform_get_irq_byname(pdev, "status");
			uart->port.irq = uart->rx_irq;

			ret = devm_request_threaded_irq(dev, uart->rx_irq,
				adi_uart4_serial_rx_int, NULL, 0, "ADI UART RX", uart);
			if (ret) {
				dev_err(dev, "Unable to attach UART RX int\n");
				return ret;
			}

			ret = devm_request_threaded_irq(dev, uart->tx_irq,
				adi_uart4_serial_tx_int, NULL, 0, "ADI UART TX", uart);
			if (ret) {
				dev_err(dev, "Unable to attach UART TX int\n");
				return ret;
			}
		}

		/* adi,uart-has-rtscts is deprecated */
		if (of_property_read_bool(np, "uart-has-rtscts") ||
		    of_property_read_bool(np, "adi,uart-has-rtscts")) {
			uart->hwflow_mode = ADI_UART_HWFLOW_PERI;
			ret = devm_request_threaded_irq(dev, uart->status_irq,
				adi_uart4_serial_mctrl_cts_int, NULL, 0, "ADI UART Modem Status",
				uart);
			if (ret) {
				uart->hwflow_mode = ADI_UART_NO_HWFLOW;
				dev_info(dev, "Unable to attach UART Modem Status int.\n");
			}
		} else
			uart->hwflow_mode = ADI_UART_NO_HWFLOW;

		uart->edbo = false;
		if (of_property_read_bool(np, "adi,use-edbo"))
			uart->edbo = true;

		if (uart->hwflow_mode == ADI_UART_HWFLOW_PERI) {
			uart->hwflow_en_pin = devm_gpiod_get(dev, "hwflow-en", GPIOD_OUT_HIGH);
			if (IS_ERR(uart->hwflow_en_pin)) {
				dev_err(dev, "hwflow-en required in peripheral hwflow mode\n");
				return PTR_ERR(uart->hwflow_en_pin);
			}
		}
	}

	uart = adi_uart4_serial_ports[uartid];
	uart->port.dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, uart);

	ret = uart_add_one_port(&adi_uart4_serial_reg, &uart->port);
	if (!ret)
		return 0;

	if (uart) {
out_error_unmap:
		adi_uart4_serial_ports[uartid] = NULL;
		kfree(uart);
	}

	return ret;
}

static int adi_uart4_serial_remove(struct platform_device *pdev)
{
	struct adi_uart4_serial_port *uart = platform_get_drvdata(pdev);

	dev_set_drvdata(&pdev->dev, NULL);

	if (uart) {
		uart_remove_one_port(&adi_uart4_serial_reg, &uart->port);
		dma_release_channel(uart->tx_dma_channel);
		dma_release_channel(uart->rx_dma_channel);
		adi_uart4_serial_ports[uart->port.line] = NULL;
		kfree(uart);
	}

	return 0;
}

static const struct of_device_id adi_uart_dt_match[] = {
	{ .compatible = "adi,uart4"},
	{},
};
MODULE_DEVICE_TABLE(of, adi_uart_dt_match);

static struct platform_driver adi_uart4_serial_driver = {
	.probe		= adi_uart4_serial_probe,
	.remove		= adi_uart4_serial_remove,
	.suspend	= adi_uart4_serial_suspend,
	.resume		= adi_uart4_serial_resume,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= adi_uart_dt_match,
	},
};

static int __init adi_uart4_serial_init(void)
{
	int ret;

	pr_info("ADI serial driver\n");

	ret = uart_register_driver(&adi_uart4_serial_reg);
	if (ret) {
		pr_err("failed to register %s:%d\n",
			adi_uart4_serial_reg.driver_name, ret);
	}

	ret = platform_driver_register(&adi_uart4_serial_driver);
	if (ret) {
		pr_err("fail to register ADI uart\n");
		uart_unregister_driver(&adi_uart4_serial_reg);
	}

	return ret;
}

static void __exit adi_uart4_serial_exit(void)
{
	platform_driver_unregister(&adi_uart4_serial_driver);
	uart_unregister_driver(&adi_uart4_serial_reg);
}

module_init(adi_uart4_serial_init);
module_exit(adi_uart4_serial_exit);

/* Early Console Support */
static inline u32 adi_uart_read(struct uart_port *port, u32 off)
{
	return readl(port->membase + off);
}

static inline void adi_uart_write(struct uart_port *port, u32 val,
				  u32 off)
{
	writel(val, port->membase + off);
}


static void adi_uart_wait_bit_set(struct uart_port *port, unsigned int offset,
				  u32 bit)
{
	while (!(adi_uart_read(port, offset) & bit))
		cpu_relax();
}


static void adi_uart_console_putchar(struct uart_port *port, unsigned char ch)
{
	/* wait for the hardware fifo to clear up */
	adi_uart_wait_bit_set(port, OFFSET_STAT, THRE);

	/* queue the character for transmission */
	adi_uart_write(port, ch, OFFSET_THR);
}


static void adi_uart_early_write(struct console *con, const char *s, unsigned int n)
{
	struct earlycon_device *dev = con->data;

	uart_console_write(&dev->port, s, n, adi_uart_console_putchar);
}


static int __init adi_uart_early_console_setup(struct earlycon_device *device,
					  const char *opt)
{
	if (!device->port.membase)
		return -ENODEV;

	device->con->write = adi_uart_early_write;
	return 0;
}

EARLYCON_DECLARE(adi_uart, adi_uart_early_console_setup);
