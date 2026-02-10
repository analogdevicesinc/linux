#define UART0_TRANSMIT_HOLD_REG 0x31003024
#define UART0_STATUS_REG 0x31003008

#define CGU0_DIV_REG 0x3108D00C
#define GCU0_CONTROL_REG 0x3108D000
#define CGU0_STAT_REG 0x3108D008

void my_putc(char c) {
        while ((*((volatile unsigned int *)UART0_STATUS_REG) & 0x20) == 0);
        *((volatile char *)UART0_TRANSMIT_HOLD_REG) = c;
}

void delay() {
        for (volatile int i = 0; i < 5000000; i++) {
                __asm__ volatile ("nop");
        }
}

void print_dots() {
        for (int i = 0; i < 50; i++) {
                my_putc('.');
                delay();
        }
        my_putc('\r');
        my_putc('\n');
}

void change_freq() {
        while (((*(volatile unsigned int *)CGU0_STAT_REG) & 0x08) != 0);
        unsigned int value = *((volatile unsigned int *)CGU0_DIV_REG);
        value &= ~0x1F;
        value |= 8;
        value |= (1 << 30);
        *((volatile unsigned int *)CGU0_DIV_REG) = value;
}

void main() {

    my_putc('\r');
    my_putc('\n');
    print_dots();
    my_putc('\r');
    my_putc('\n');
    change_freq();
    my_putc('\r');
    my_putc('\n');
    print_dots();
    my_putc('\r');
    my_putc('\n'); 
    while(1) {
    }
}
