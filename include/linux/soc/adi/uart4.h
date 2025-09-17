/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef ADI_UART4_H
#define ADI_UART4_H

void adi_uart4_serial_rx_dma_timeout(struct timer_list *list);
struct adi_uart4_serial_port *to_adi_serial_port(struct uart_port *port);

#endif
