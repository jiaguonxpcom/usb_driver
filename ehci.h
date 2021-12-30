                                                                                                                                                                                                                                                                                                                                                              
                                                                                                                                                                                                                                                                                                                                                                                                                                                                          

#define ECHI_OK    0
#define ECHI_ERROR 1

#define ECHI_MODE_HOST   3
#define ECHI_MODE_DEVICE 2

typedef void (*usb_callback)(uint32_t );

typedef enum _ehci_speed
{
    ehci_speed_full = 0,
    ehci_speed_low  = 1,
    ehci_speed_high = 2
}ehci_speed_t;

typedef enum _ep_type
{
    control,
    bulk_out,
    bulk_in,
    int_out,
    int_in,
    iso_out,
    iso_in    
} ep_type_t;

typedef struct _ehci_control_transfer
{
    USB_Type * usb;
    uint32_t   addr;
    uint32_t   ep;
    ehci_speed_t speed;

    uint8_t  * buf_setup;
    uint32_t   len_setup;

    uint8_t  * buf_in;
    uint32_t   len_in;

    uint8_t  * buf_out;
    uint32_t   len_out;
} ehci_control_transfer_t;

typedef struct _ehci_handle
{
    // app
    USB_Type    * usb;
    USBPHY_Type * phy;
    
    // data below is a not care for app, it is used by driver itself only
    void * qh_hs00;
    
    void * pfl_table;
    void * pfl_head;
} ehci_handle_t;

typedef void * handle_t;

/*
    Configuration
*/
#define usb_printf PRINTF



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
handle_t echi_create_iso_ep(
    ehci_handle_t * handle,
    uint32_t     addr, 
    uint32_t     ep, 
    ep_type_t    ep_type,
    ehci_speed_t speed,
    uint32_t     max_pack_size,
    uint32_t     multi,
    uint32_t     buf_4k_num);
uint32_t ehci_write_ep(void * handle, void * buf, uint32_t len, usb_callback call_back);









