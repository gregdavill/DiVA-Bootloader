#include <stdio.h>
#include <irq.h>
#include <uart.h>
#include <usb.h>
#include <time.h>
#include <dfu.h>
#include <rgb.h>
#include <spi.h>
#include <generated/csr.h>
#include <generated/mem.h>

void isr(void)
{
    unsigned int irqs;

    irqs = irq_pending() & irq_getmask();

    if (irqs & (1 << USB_INTERRUPT))
        usb_isr();
    
}

static int button_pressed(void){
    return button_i_read() != 1;
}

void reboot(void) {
    irq_setie(0);
    irq_setmask(0);

    // Free the SPI controller, which returns it to memory-mapped mode.
    spiFree();

    // Issue a reboot
    while(1){
        reboot_ctrl_write(0xac); 
    }

    __builtin_unreachable();
}

static void init(void)
{
    rgb_init();
    usb_init();

    spiInit();

    // Check for QE bit set. If not set, enable it.
    spiSetQE();
    if(!button_pressed()){
        spiFree();
        reboot();
    }


#ifdef CSR_UART_BASE
    init_printf(NULL, rv_putchar);
#endif

    irq_setmask(0);
    irq_setie(1);

    time_init();
    dfu_init();
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    init();
    usb_connect();
    
    while (1)
    {
        usb_poll();
        dfu_poll();
    }
    return 0;
}
