                                                                                                                                                                                                                                                                                                                                                              
                                                                                                                                                                                                                                                                                                                                                                                                                                                                          

#define ECHI_OK    0
#define ECHI_ERROR 1

#define ECHI_MODE_HOST   3
#define ECHI_MODE_DEVICE 2

#define EHCI_SPEED_FULL 0
#define EHCI_SPEED_LOW  1
#define EHCI_SPEED_HIGH 2

typedef struct
{
    USB_Type * usb;
    uint32_t   addr;
    uint32_t   ep;
    uint32_t   speed;

    uint8_t  * buf_setup;
    uint32_t   len_setup;

    uint8_t  * buf_in;
    uint32_t   len_in;

    uint8_t  * buf_out;
    uint32_t   len_out;
} ehci_control_transfer_t;

typedef struct
{
    // app
    USB_Type    * usb;
    USBPHY_Type * phy;
    
    // internal
    void * qh_hs00;
    
    void * pfl_table;
    void * pfl_head;
} ehci_handle_t;

/*
    Configuration
*/
#define usb_printf PRINTF

#define CONFIG_QH_CNT     3
#define CONFIG_QTD_CNT    3
#define CONFIG_4K_BUF_CNT 4
#define CONFIG_PFL_SIZE   32 // Element size, it decide the maximum periodic interval



/*
    CALL BACK EVENT(cbe)
*/
typedef enum
{
    event_none,
    event_port_attach,
    event_port_detach,
    event_transfer_done,
}ehci_callback_event_t;

typedef void (*usb_callback_t)(ehci_callback_event_t event);


/*
    API
*/
void ehci_init(ehci_handle_t * handle, uint32_t mode);
void ehci_reset(ehci_handle_t * handle); 
uint32_t ehci_get_speed(ehci_handle_t * handle);
int ehci_control_in(ehci_handle_t * handle, ehci_control_transfer_t * transfer);
void ehci_isr(ehci_handle_t * handle, usb_callback_t callback);










