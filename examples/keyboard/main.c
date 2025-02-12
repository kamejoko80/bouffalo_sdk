#include "bflb_mtimer.h"
#include "board.h"

#define DBG_TAG "MAIN"
#include "log.h"

extern void hid_keyboard_init(void);
extern void hid_keyboard_test(void);

int main(void)
{
    board_init();
    hid_keyboard_init();

/*     while (1) {
        LOG_F("hello world fatal\r\n");
        LOG_E("hello world error\r\n");
        LOG_W("hello world warning\r\n");
        LOG_I("hello world information\r\n");
        LOG_D("hello world debug\r\n");
        LOG_T("hello world trace\r\n");
        LOG_RF("hello world fatal raw\r\n");
        LOG_RE("hello world error raw\r\n");
        LOG_RW("hello world warning raw\r\n");
        LOG_RI("hello world information raw\r\n");
        LOG_RD("hello world debug raw\r\n");
        LOG_RT("hello world trace raw\r\n");
        bflb_mtimer_delay_ms(1000);
    } */

    bflb_mtimer_delay_ms(5000);
    LOG_F("start to press\r\n");

    while (1) {
        hid_keyboard_test();
        bflb_mtimer_delay_ms(50);
    }
    
}
