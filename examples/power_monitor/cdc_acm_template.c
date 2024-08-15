#include <stdarg.h>  // For handling variable arguments
#include <stdio.h>   // For vsnprintf
#include <stdint.h>  // For uint8_t and uint16_t types
#include <string.h>  // For strlen

#include "usbd_core.h"
#include "usbd_cdc.h"
#include "cmd.h"
#include "ina229.h"

/*!< endpoint address */
#define CDC1_IN_EP  0x81  // IN Endpoint for CDC1
#define CDC1_OUT_EP 0x02  // OUT Endpoint for CDC1
#define CDC1_INT_EP 0x83  // Interrupt IN Endpoint for CDC1

#define CDC2_IN_EP  0x84  // IN Endpoint for CDC2
#define CDC2_OUT_EP 0x05  // OUT Endpoint for CDC2
#define CDC2_INT_EP 0x86  // Interrupt IN Endpoint for CDC2

#define USBD_VID           0x0815
#define USBD_PID           0x2024
#define USBD_MAX_POWER     100
#define USBD_LANGID_STRING 1033

/*!< config descriptor size */
#define USB_CONFIG_SIZE (9 + CDC_ACM_DESCRIPTOR_LEN + CDC_ACM_DESCRIPTOR_LEN)

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

/*!< global descriptor */
static const uint8_t cdc_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0xEF, 0x02, 0x01, USBD_VID, USBD_PID, 0x0100, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x04, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),

    // First CDC ACM Interface
    CDC_ACM_DESCRIPTOR_INIT(0x00, CDC1_INT_EP, CDC1_OUT_EP, CDC1_IN_EP, CDC_MAX_MPS, 0x02),

    // Second CDC ACM Interface
    CDC_ACM_DESCRIPTOR_INIT(0x02, CDC2_INT_EP, CDC2_OUT_EP, CDC2_IN_EP, CDC_MAX_MPS, 0x02),

    ///////////////////////////////////////
    /// string0 descriptor
    ///////////////////////////////////////
    USB_LANGID_INIT(USBD_LANGID_STRING),
    ///////////////////////////////////////
    /// string1 descriptor
    ///////////////////////////////////////
    0x14,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'C', 0x00,                  /* wcChar0 */
    'h', 0x00,                  /* wcChar1 */
    'e', 0x00,                  /* wcChar2 */
    'r', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    'y', 0x00,                  /* wcChar5 */
    'U', 0x00,                  /* wcChar6 */
    'S', 0x00,                  /* wcChar7 */
    'B', 0x00,                  /* wcChar8 */
    ///////////////////////////////////////
    /// string2 descriptor
    ///////////////////////////////////////
    0x26,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'p', 0x00,                  /* wcChar0 */
    'o', 0x00,                  /* wcChar1 */
    'w', 0x00,                  /* wcChar2 */
    'e', 0x00,                  /* wcChar3 */
    'r', 0x00,                  /* wcChar4 */
    '_', 0x00,                  /* wcChar5 */
    'm', 0x00,                  /* wcChar6 */
    'o', 0x00,                  /* wcChar7 */
    'n', 0x00,                  /* wcChar8 */
    'i', 0x00,                  /* wcChar9 */
    't', 0x00,                  /* wcChar10 */
    'o', 0x00,                  /* wcChar11 */
    'r', 0x00,                  /* wcChar12 */
    '_', 0x00,                  /* wcChar13 */
    '2', 0x00,                  /* wcChar14 */
    '0', 0x00,                  /* wcChar15 */
    '2', 0x00,                  /* wcChar16 */
    '4', 0x00,                  /* wcChar17 */
    ///////////////////////////////////////
    /// string3 descriptor
    ///////////////////////////////////////
    0x08,                       /* bLength */
    USB_DESCRIPTOR_TYPE_STRING, /* bDescriptorType */
    'c', 0x00,                  /* wcChar0 */
    'o', 0x00,                  /* wcChar1 */
    'm', 0x00,                  /* wcChar2 */
#ifdef CONFIG_USB_HS
    ///////////////////////////////////////
    /// device qualifier descriptor
    ///////////////////////////////////////
    0x0a,
    USB_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    0x00,
    0x02,
    0x02,
    0x02,
    0x01,
    0x40,
    0x01,
    0x00,
#endif
    0x00
};

#define WR_BUFF_SIZE       (sizeof(response_t))
#define DATA_RPT_BUFF_SIZE (sizeof(ina229_data_report_t))
#define RD_BUFF_SIZE       (256)

USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t read_buffer1[RD_BUFF_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t write_buffer1[WR_BUFF_SIZE];
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t data_rpt_buffer[DATA_RPT_BUFF_SIZE];

volatile uint8_t *p_wr_buf = &write_buffer1[0];
volatile uint8_t *p_data_rpt_buf = &data_rpt_buffer[0];
volatile bool ep_tx_busy_flag = false;

#ifdef CONFIG_USB_HS
#define CDC_MAX_MPS 512
#else
#define CDC_MAX_MPS 64
#endif

void usbd_event_handler(uint8_t event)
{
    switch (event) {
        case USBD_EVENT_RESET:
            break;
        case USBD_EVENT_CONNECTED:
            break;
        case USBD_EVENT_DISCONNECTED:
            break;
        case USBD_EVENT_RESUME:
            break;
        case USBD_EVENT_SUSPEND:
            break;
        case USBD_EVENT_CONFIGURED:
            /* setup first out ep read transfer */
            usbd_ep_start_read(CDC1_OUT_EP, read_buffer1, RD_BUFF_SIZE);
            break;
        case USBD_EVENT_SET_REMOTE_WAKEUP:
            break;
        case USBD_EVENT_CLR_REMOTE_WAKEUP:
            break;

        default:
            break;
    }
}

void usbd_cdc1_acm_bulk_in(uint8_t ep, uint32_t nbytes)
{
    if ((nbytes % CDC_MAX_MPS) == 0 && nbytes) {
        usbd_ep_start_write(CDC1_IN_EP, NULL, 0);  // Send ZLP if required
    } else {
        ep_tx_busy_flag = false;
    }
}

void usbd_cdc2_acm_bulk_in(uint8_t ep, uint32_t nbytes)
{
    if ((nbytes % CDC_MAX_MPS) == 0 && nbytes) {
        usbd_ep_start_write(CDC2_IN_EP, NULL, 0);  // Send ZLP if required
    } else {
        ep_tx_busy_flag = false;
    }
}

void usbd_cdc1_acm_bulk_out(uint8_t ep, uint32_t nbytes)
{
    cmd_process(read_buffer1, nbytes);
    usbd_ep_start_read(CDC1_OUT_EP, read_buffer1, RD_BUFF_SIZE);
}

void usbd_cdc2_acm_bulk_out(uint8_t ep, uint32_t nbytes)
{
    /* not used */
}

/*!< endpoint call back */
struct usbd_endpoint cdc1_in_ep = {
    .ep_addr = CDC1_IN_EP,
    .ep_cb = usbd_cdc1_acm_bulk_in
};

struct usbd_endpoint cdc1_out_ep = {
    .ep_addr = CDC1_OUT_EP,
    .ep_cb = usbd_cdc1_acm_bulk_out
};

struct usbd_endpoint cdc2_in_ep = {
    .ep_addr = CDC2_IN_EP,
    .ep_cb = usbd_cdc2_acm_bulk_in
};

struct usbd_endpoint cdc2_out_ep = {
    .ep_addr = CDC2_OUT_EP,
    .ep_cb = usbd_cdc2_acm_bulk_out
};

struct usbd_interface intf11;
struct usbd_interface intf12;
struct usbd_interface intf21;
struct usbd_interface intf22;

void cdc_acm_init(void)
{
    memset(&write_buffer1[0], 0, WR_BUFF_SIZE);
    memset(&read_buffer1[0],  0, RD_BUFF_SIZE);

    usbd_desc_register(cdc_descriptor);

    // Register the first CDC interface
    usbd_add_interface(usbd_cdc_acm_init_intf(&intf11));
    usbd_add_interface(usbd_cdc_acm_init_intf(&intf12));
    usbd_add_endpoint(&cdc1_in_ep);
    usbd_add_endpoint(&cdc1_out_ep);

    // Register the second CDC interface
    usbd_add_interface(usbd_cdc_acm_init_intf(&intf21));
    usbd_add_interface(usbd_cdc_acm_init_intf(&intf22));
    usbd_add_endpoint(&cdc2_in_ep);
    usbd_add_endpoint(&cdc2_out_ep);

    usbd_initialize();
}

volatile uint8_t dtr_enable = 0;

void usbd_cdc_acm_set_dtr(uint8_t intf, bool dtr)
{
    if (dtr) {
        dtr_enable = 1;
    } else {
        dtr_enable = 0;
    }
}

void cdc_acm_data_send_with_dtr_test(void)
{
    if (dtr_enable) {
        ep_tx_busy_flag = true;
        usbd_ep_start_write(CDC1_IN_EP, write_buffer1, WR_BUFF_SIZE);
        while (ep_tx_busy_flag) {
        }
    }
}

void cdc_acm_cmd_response_send(void)
{
    usbd_ep_start_write(CDC1_IN_EP, write_buffer1, WR_BUFF_SIZE);
}

void cdc_acm_data_rpt_send(void)
{
    usbd_ep_start_write(CDC2_IN_EP, data_rpt_buffer, DATA_RPT_BUFF_SIZE);
}

void cdc_acm_prints(char *str)
{
    uint16_t len = strlen(str);

    if (dtr_enable) {
        ep_tx_busy_flag = true;
        usbd_ep_start_write(CDC1_IN_EP, (uint8_t*)str, len);
        while (ep_tx_busy_flag) {
        }
    }
}

/* Must define global buffer */
USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX char buffer[256];

void cdc_acm_printf(const char *format, ...)
{
    va_list args;
    va_start (args, format);
    vsnprintf (buffer, 255, format, args);
    cdc_acm_prints(buffer);
    va_end (args);
}
