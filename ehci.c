

#include "MIMXRT1176_cm7.h"
#include "fsl_debug_console.h"
#include "ehci.h"
#include "string.h"


#define ECHI_T   1
#define ECHI_T_0 0
#define ECHI_T_1 1

#define ECHI_BUF_ADDR_MASK (~0xfff)

#define ECHI_HLINK_ADDR_MASK (~0x1f)
#define ECHI_HLINK_TYPE_MASK (0x6)
#define ECHI_HLINK_TYPE_ITD  (0<<1)
#define ECHI_HLINK_TYPE_QH   (1<<1)
#define ECHI_HLINK_TYPE_SITD (2<<1)
#define ECHI_HLINK_TYPE_FSTN (3<<1)

#define ECHI_QH_ECHAR_NAK_RL(n)      (n<<28)
#define ECHI_QH_ECHAR_NAK_RL(n)      (n<<28)
#define ECHI_QH_ECHAR_C              (1<<27)
#define ECHI_QH_ECHAR_MAX_PACKAGE(n) (n<<16)
#define ECHI_QH_ECHAR_HEAD           (1<<15)
#define ECHI_QH_ECHAR_TOGGLE_KEEP    (0<<14)
#define ECHI_QH_ECHAR_TOGGLE_BY_QTD  (1<<14)
#define ECHI_QH_ECHAR_SPEED(n)       (n<<12)
#define ECHI_QH_ECHAR_EP(n)          (n<<8)
#define ECHI_QH_ECHAR_INACTIVE       (1<<7)
#define ECHI_QH_ECHAR_ADDR(n)        (n<<0)

#define ECHI_QH_ECAPA_MULTI(n)       (n<<30)
#define ECHI_QH_ECAPA_PORT(n)        (n<<23)
#define ECHI_QH_ECAPA_HUB_ADDR(n)    (n<<16)
#define ECHI_QH_ECAPA_C_MASK(n)      (n<<8)
#define ECHI_QH_ECAPA_S_MASK(n)      (n<<0)

#define PID_OUT   0
#define PID_IN    1
#define PID_SETUP 2
#define STATUS_ACTIVE 0x80

#define ECHI_QTD_ADDR_MASK           (~0x1f)
#define EHCI_QTD_TOKEN_DT0           (0<<31)
#define EHCI_QTD_TOKEN_DT1           (1<<31)
#define EHCI_QTD_TOKEN_TOTLE_BYTE(n) (n<<16)
#define EHCI_QTD_TOKEN_IOC           (1<<15)
#define EHCI_QTD_TOKEN_NO_IOC        (0<<15)
#define EHCI_QTD_TOKEN_C_PAGE(n)     (n<<12)
#define EHCI_QTD_TOKEN_CERR(n)       (n<<10)
#define EHCI_QTD_TOKEN_PID(n)        (n<<8)
#define EHCI_QTD_TOKEN_STATUS(n)     (n<<0)

#define EHCI_QTD_TOKEN_GET_PID(n)         (n>>8)&0x3
#define EHCI_QTD_TOKEN_GET_STATUS(n)      (n>>0)&0xff
#define EHCI_QTD_TOKEN_GET_TOTAL_BYTES(n) (n>>16)&0xff



#define EHCI_BOUNDARY_ALIGNMENT 32
#define ALLOCATED   1
#define UNALLOCATED 0


#define LOG_ENABLE 1



// qTD
typedef struct _ECHI_QTD
{
    uint32_t next_qtd;   // 1
    uint32_t alt_qtd;    // 2
    uint32_t token;      // 3
    uint32_t buf[5];     // 4 to 8
} ECHI_QTD_t;

// qH
typedef struct _ECHI_QH
{
    uint32_t hlink;   // 1 word
    uint32_t echar;   // 1 word
    uint32_t ecapa;   // 1 word
    uint32_t cur_qtd; // 1 word
    ECHI_QTD_t qtd;   // 8 word

    // non-ehci definition
    // this is a special design, place some information here, it can be used in ISR callback.
    ECHI_QTD_t * first_qtd;  // 1 word
    uint8_t *    buf_in;     // 1 word
    uint32_t     len_in;     // 1 word
    uint32_t     interval;   // 1 word, for int EP

    // 4 + 8 + 4 = 16 words
    // Total size should be 32 byte align = 8 words align
} ECHI_QH_t;






/*
    --------------- Cache maintenance --------------
*/
static void d_cache_flush(void * addr, uint32_t len)
{
    SCB_CleanDCache_by_Addr((uint32_t *)addr, len);
}
static void d_cache_invalidate(void * addr, uint32_t len)
{
    SCB_InvalidateDCache_by_Addr((uint32_t *)addr, len);
}
static bool hlink_last(uint32_t hlink)
{
    if(hlink & ECHI_T)
    {
        return true;
    }
    else
    {
        return false;
    }
}
static uint32_t * hlink_next(uint32_t * hlink)
{
    return (uint32_t *)(*hlink & ECHI_HLINK_ADDR_MASK);
}
static bool qh_qtd_last(uint32_t qh_qtd)
{
    if(qh_qtd & ECHI_T)
    {
        return true;
    }
    else
    {
        return false;
    }
}
static uint32_t make_hlink(void * addr, uint32_t type)
{
    uint32_t r;
    r = (uint32_t)addr & ECHI_HLINK_ADDR_MASK;
    r |= type;
    return r;
}
/*
    *hlink point to new_addr
*/
static void update_hlink_address(uint32_t * hlink, uint32_t new_addr)
{
    new_addr &= ECHI_HLINK_ADDR_MASK;
    *hlink   &= (~ECHI_HLINK_ADDR_MASK);
    // clear ECHI_T
    *hlink   &=  ~ECHI_T;
    *hlink   |= new_addr;
}





#include "ehci_config.h"
#include "ehci_memory.h"
#include "ehci_async.cxx"
#include "ehci_period.cxx"
#include "ehci_memory.cxx"
#include "ehci_api.cxx"

/*
    --------------- External API --------------
*/
void ehci_init(ehci_handle_t * handle, uint32_t mode)
{
    mem_manage_init();
    buf_4k_init();
    
    if(mode == ECHI_MODE_HOST)
    {
        handle->usb->USBMODE  = ECHI_MODE_HOST;
        handle->usb->USBCMD   = USB_USBCMD_ITC(1);   // If set ITC to be 0, possible to trigger one interrupt twice
        handle->usb->PORTSC1 |= USB_PORTSC1_PP_MASK; // Power device
        handle->usb->USBCMD  |= USB_USBCMD_RS_MASK;
        
        
        handle->usb->USBINTR |= USB_USBINTR_UE_MASK  | // IOC
                                USB_USBINTR_UEE_MASK | // IOC error
                                USB_USBINTR_PCE_MASK | // Port change
                                USB_USBINTR_SEE_MASK | // System error
                                //USB_USBINTR_AAE_MASK |
                                //USB_USBINTR_SRE_MASK | // SOF
                                0;
    }
    
    init_async_schecule(handle);
    init_periodic_schecule(handle);
}

void ehci_reset(ehci_handle_t * handle)
{
    ehci_speed_t speed;
    usb_printf("--->ehci_reset.\n");
    
    handle->usb->PORTSC1 |= USB_PORTSC1_PR_MASK;
    __DSB();
    while (handle->usb->PORTSC1 & USB_PORTSC1_PR_MASK)
        ;
    
    speed = ehci_get_speed(handle);
    if(speed == ehci_speed_high)
    {
        handle->phy->CTRL |= USBPHY_CTRL_ENHOSTDISCONDETECT_MASK;
    }
}

uint32_t ehci_get_speed(ehci_handle_t * handle)
{
	ehci_speed_t speed;
	speed = (handle->usb->PORTSC1 >> 26) & 3;

    if(speed == ehci_speed_high)
    {
        usb_printf("high speed \r\n");
    }
    else if(speed == ehci_speed_full)
    {
        usb_printf("full speed \r\n");
    }
    else if(speed == ehci_speed_low)
    {
        usb_printf("low speed \r\n");
    }
    return speed;
}

/*
    qH --> qTD[0](Setup) --> qTD[1](In) --> qTD[2](Out)
*/
int ehci_control_in(ehci_handle_t * handle, ehci_control_transfer_t * transfer)
{
    ECHI_QH_t  * qh;
    ECHI_QTD_t * qtd[3];
    char     * buf_4k[2];
    uint32_t package_len_max = 0;

    qtd[0] = qtd_malloc();
    qtd[1] = qtd_malloc();
    qtd[2] = qtd_malloc();
    buf_4k[0] = buf_4k_malloc();
    buf_4k[1] = buf_4k_malloc();

    if((qtd[0]    == NULL) || 
       (qtd[1]    == NULL) || 
       (qtd[2]    == NULL) ||
       (buf_4k[0] == NULL) ||
       (buf_4k[1] == NULL) )
    {
        qtd_free(qtd[0]);
        qtd_free(qtd[1]);
        qtd_free(qtd[2]);
        buf_4k_free(buf_4k[0]);
        buf_4k_free(buf_4k[1]);
        return ECHI_ERROR;
    }
    
    // Init SETUP qtd
    qtd[0]->next_qtd = (uint32_t)qtd[1];
    qtd[0]->alt_qtd  = ECHI_T;
    qtd[0]->token    = EHCI_QTD_TOKEN_DT0                                 |
                       EHCI_QTD_TOKEN_TOTLE_BYTE(transfer->len_setup)     |
                       EHCI_QTD_TOKEN_NO_IOC                              |
                       EHCI_QTD_TOKEN_C_PAGE(0) /*C Page point to buf n*/ |
                       EHCI_QTD_TOKEN_CERR(3)                             |
                       EHCI_QTD_TOKEN_PID(PID_SETUP)                      |
                       EHCI_QTD_TOKEN_STATUS(STATUS_ACTIVE);

    // Any random buffer may cross 4K boundary, so need a copy here.
    memcpy(buf_4k[0], transfer->buf_setup, transfer->len_setup);
    qtd[0]->buf[0] = (uint32_t)buf_4k[0];
    
    // Init IN qtd
    qtd[1]->next_qtd = (uint32_t)qtd[2];
    qtd[1]->alt_qtd  = ECHI_T;
    qtd[1]->token    = EHCI_QTD_TOKEN_DT1                                 |
                       EHCI_QTD_TOKEN_TOTLE_BYTE(transfer->len_in)        |
                       EHCI_QTD_TOKEN_NO_IOC                              |
                       EHCI_QTD_TOKEN_C_PAGE(0) /*C Page point to buf n*/ |
                       EHCI_QTD_TOKEN_CERR(3)                             |
                       EHCI_QTD_TOKEN_PID(PID_IN)                         |
                       EHCI_QTD_TOKEN_STATUS(STATUS_ACTIVE);

    qtd[1]->buf[0] = (uint32_t)buf_4k[1];
    
    // Init OUT qtd
    qtd[2]->next_qtd = ECHI_T;
    qtd[2]->alt_qtd  = ECHI_T;
    qtd[2]->token    = EHCI_QTD_TOKEN_DT1                                 |
                       EHCI_QTD_TOKEN_TOTLE_BYTE(0)                       |
                       EHCI_QTD_TOKEN_IOC                                 |
                       EHCI_QTD_TOKEN_C_PAGE(0) /*C Page point to buf n*/ |
                       EHCI_QTD_TOKEN_CERR(3)                             |
                       EHCI_QTD_TOKEN_PID(PID_OUT)                        |
                       EHCI_QTD_TOKEN_STATUS(STATUS_ACTIVE);
    qtd[2]->buf[0] = 0;
    

    if(transfer->speed == ehci_speed_high) 
    {
        qh = handle->qh_hs00;
    }
    else
    {
        // fs ls 00
        qh = (ECHI_QH_t  *)(((ECHI_QH_t  *)(handle->qh_hs00))->hlink & ECHI_HLINK_ADDR_MASK);
    }
    usb_printf("qh = %x\r\n", qh);

    qh->buf_in       = transfer->buf_in;
    qh->len_in       = transfer->len_in;
    qh->qtd.next_qtd = (uint32_t)qtd[0];
    qh->first_qtd    = qtd[0];
    
    // Flush
    d_cache_flush(qh,     sizeof(ECHI_QH_t));
    d_cache_flush(qtd[0], sizeof(ECHI_QTD_t));
    d_cache_flush(qtd[1], sizeof(ECHI_QTD_t));
    d_cache_flush(qtd[2], sizeof(ECHI_QTD_t));
    d_cache_flush(buf_4k[0], transfer->len_setup);
    
    // Attach qh and enable async schedule
    return ECHI_OK;
}
int ehci_control_out(ehci_control_transfer_t * transfer)
{
    
}
int ehci_bulk_in(ehci_control_transfer_t * transfer)
{
    
}
int ehci_bulk_out(ehci_control_transfer_t * transfer)
{
    
}
int ehci_int_in(ehci_control_transfer_t * transfer)
{
    
}
int ehci_int_out(ehci_control_transfer_t * transfer)
{
    
}
int ehci_iso_in(ehci_control_transfer_t * transfer)
{
    
}
int ehci_iso_out(ehci_control_transfer_t * transfer)
{
    
}

static void free_qtd_in_qh(ECHI_QH_t  * qh)
{
    ECHI_QTD_t * qtd;
    ECHI_QTD_t * qtd_next;

    qtd = (ECHI_QTD_t *)(qh->first_qtd);
    while(qtd != NULL)
    {
        buf_4k_free((void *)(qtd->buf[0]));
        qtd_next = (ECHI_QTD_t *)(qtd->next_qtd);
        qtd_free(qtd);

        if(qh_qtd_last((uint32_t)qtd_next))
        {
            qtd = NULL;
        }
        else
        {
            qtd = qtd_next;
        }
    }
}
/*
    --------------- ISR --------------
*/
#define MAX_QTD_CNT_FROM_ONE_QH 3
void mem_dump_8(void * addr, uint32_t len);
static void isr_ioc(ehci_handle_t * handle, usb_callback_t callback)
{
    ECHI_QH_t  * qh;
    ECHI_QH_t  * qh_prev;
    ECHI_QTD_t * qtd;
    uint32_t   pid[MAX_QTD_CNT_FROM_ONE_QH]       = {0};
    uint32_t   status[MAX_QTD_CNT_FROM_ONE_QH]    = {0};
    uint32_t   total_len[MAX_QTD_CNT_FROM_ONE_QH] = {0};
    uint32_t   buf[MAX_QTD_CNT_FROM_ONE_QH]       = {0};

    uint32_t   index = 0;
    uint32_t   len_rx;
    uint32_t   err = ECHI_OK;
    uint32_t   busy;
    USB_Type * usb = handle->usb;
    
    // Go through each qh.
    qh = handle->qh_hs00;
    do
    {
    	d_cache_invalidate(qh, sizeof(ECHI_QH_t));

    	// Go through each qTD
        qtd = (ECHI_QTD_t *)(qh->first_qtd);
        
        if(qtd != NULL)
        {
            busy = false;
            while(qtd != NULL)
            {
                usb_printf("qtd->token = %x\r\n", qtd->token);
                
                d_cache_invalidate(qtd, sizeof(ECHI_QTD_t));
                pid[index]       = EHCI_QTD_TOKEN_GET_PID(qtd->token);
                status[index]    = EHCI_QTD_TOKEN_GET_STATUS(qtd->token);
                total_len[index] = EHCI_QTD_TOKEN_GET_TOTAL_BYTES(qtd->token);
                buf[index]       = qtd->buf[0] & ECHI_BUF_ADDR_MASK;
                
                usb_printf("pid[%d] = %x\r\n",    index, pid[index]);
                usb_printf("status[%d] = %x\r\n", index, status[index]);
                usb_printf("total_len[%d] = %x\r\n", index, total_len[index]);
                
                if(status[index] & STATUS_ACTIVE)
                {
                    // qtd busy
                    busy = true;
                    break;
                }
                else
                {
                    if(status[index])
                    {
                        err = ECHI_ERROR;
                        usb_printf("error happen \r\n");
                    }
                    
                    index++;

                    if(qh_qtd_last(qtd->next_qtd))
                    {
                        qtd = NULL;
                    }
                    else
                    {
                        qtd = (ECHI_QTD_t *)(qtd->next_qtd);
                    }                    
                }
            } // while(qtd != NULL)
            
            if(busy)
            {
                // do nothing
            }
            else
            {
                // transfer done.
                if(err == ECHI_OK)
                {
                    // process this transfer
                    
                    // Setup --> IN --> OUT(0)
                    if((pid[0] == PID_SETUP) && (pid[1] == PID_IN))
                    {
                        usb_printf(" SETUP IN transfer done. \r\n");
                        len_rx = qh->len_in - total_len[1];
                        usb_printf("len_rx = %d \r\n", len_rx);
                        d_cache_invalidate((void*)buf[1], len_rx);
                        memcpy((void*)(qh->buf_in), (void*)buf[1], len_rx);
                        mem_dump_8((void*)(qh->buf_in), len_rx);
                        callback(event_transfer_done);
                    }
                }
                else
                {
                    // process error
                }
                
                // clear this qh
                free_qtd_in_qh(qh);
                qh->first_qtd = NULL;
                qh->buf_in    = NULL;
                qh->len_in    = 0;
            }
        } // if(qtd != NULL)

        // process next qh
        qh = (ECHI_QH_t  *)(qh->hlink & ECHI_HLINK_ADDR_MASK);
    } while(qh != handle->qh_hs00);
}

void ehci_isr(ehci_handle_t * handle, usb_callback_t callback)
{
    uint32_t status;
    USB_Type * usb = handle->usb;
    
    status = usb->USBSTS;
    usb->USBSTS = 0xFFFFFFFF;
    __DSB();


    usb_printf("\r\n\r\n\r\n\r\n----------- ehci_isr -----------\r\n");
    usb_printf("status = %x\r\n",  status);
    usb_printf("USBSTS = %x\r\n",  usb->USBSTS);
    usb_printf("PORTSC1 = %x\r\n", usb->PORTSC1);
    usb_printf("USBCMD = %x\r\n",  usb->USBCMD);
    usb_printf("USBINTR = %x\r\n", usb->USBINTR);

    if(status & USB_USBSTS_UEI_MASK)
    {
        usb_printf("!!!IOC err\r\n");
        while(1);
    }
    if(status & USB_USBSTS_SEI_MASK)
    {
        usb_printf("!!!System error\n");
        while(1);
    }
    if(status & USB_USBSTS_PCI_MASK)
    {
        usb_printf("Port change\r\n");
        if(usb->PORTSC1 & USB_PORTSC1_CCS_MASK)
        {
            callback(event_port_attach);
        }
        else
        {
            callback(event_port_detach);
            handle->phy->CTRL &= ~USBPHY_CTRL_ENHOSTDISCONDETECT_MASK;
        }
    }
    if(status & USB_USBSTS_UI_MASK)
    {
        usb_printf("IOC \r\n");
        isr_ioc(handle, callback);
    }    

    usb_printf("*exit: usb->USBSTS = %x \r\n\r\n\r\n\r\n", usb->USBSTS);
    __DSB();
}

/*
    --------------- Advanced API --------------
*/
// Expose these buffer for possible acceleration
void * echi_buf_4k_allocate(void)
{
    return buf_4k_malloc();
}
void echi_buf_4k_free(void * p)
{
    buf_4k_free(p);
}






