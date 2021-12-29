

#define ECHI_ITD_ADDR_MASK        (~0x1f)
#define ECHI_ITD_BP0_EP(n)        (n<<9)
#define ECHI_ITD_BP0_ADDR(n)      (addr<<0)
#define ECHI_ITD_BP1_IO(n)        (n<<11)
#define ECHI_ITD_BP1_MAX_PACK(n)  (n<<0)
#define ECHI_ITD_BP2_MULTI(n)     (n<<0)



// ITD
typedef struct _ECHI_ITD
{
    uint32_t hlink;    // 1 word
    uint32_t TSC[8];   // 8 word
    uint32_t BP[7];    // 7 word

    // non-ehci definition
    // this is a special design, place some information here, it can be used in ISR callback.
    uint8_t * buf_in;        // 1 word
    uint32_t  len_in;        // 1 word
    uint32_t  interval;      // 1 word
    uint32_t  revserved[5];  // 5 word

    // 16 + 8 = 24 words
    // Total size should be 32 byte align = 8 words align
} ECHI_ITD_t;

typedef struct _ECHI_PFL
{
    void * frame[CONFIG_PFL_SIZE];
} ECHI_PFL_t; //Period Frame List

static void echi_periodic_enable(USB_Type * usb)
{
    usb->USBCMD |= USB_USBCMD_PSE_MASK;
}
static void echi_periodic_disable(USB_Type * usb)
{
    usb->USBCMD &= ~USB_USBCMD_PSE_MASK;
}
static void dump_periodic_queue(ehci_handle_t * handle)
{

}
static uint32_t period_get_interval(void * element)
{
    uint32_t * hlink;
    uint32_t type;
    hlink = element;
    type = ((*hlink) & ECHI_HLINK_TYPE_MASK);

    switch(type)
    {
        case ECHI_HLINK_TYPE_QH:
            return ((ECHI_QH_t*)element)->interval;
            break;
        case ECHI_HLINK_TYPE_ITD:
            break;
    }
}

/*
    hilink: new element to be install.
*/
static void install_periodic_element(ehci_handle_t * handle, uint32_t * hlink)
{
    uint32_t  * hlink_cur;
    uint32_t  * hlink_prev;
    
    if( (uint32_t)(handle->pfl_head) == ECHI_T)
    {
        // 0 element in queue, add to pfl_head.
        handle->pfl_head = hlink;
    }
    else
    {
        hlink_prev = NULL;
        hlink_cur  = handle->pfl_head;

        while(1)
        {
            if(period_get_interval(hlink) >= period_get_interval(hlink_cur))
            {
                // add to left of qh_cur
                // 1.new node point to hlink_cur
                *hlink = update_hlink_address(hlink, (uint32_t)hlink_cur);

                // 2.hlink_prev point to new
                if(hlink_prev == NULL)
                {
                    handle->pfl_head = hlink;
                }
                else
                {
                    *hlink_prev = update_hlink_address(hlink_prev, (uint32_t)hlink);
                }

                usb_printf("period element added at left. \r\n");
                break;
            }
            else
            {
                // check if it is the last element in queue
                if(hlink_last((uint32_t)hlink_cur))
                {
                    // new element is minimum interval and should be placed at the end
                    // hlink_cur point new element
                    update_hlink_address(hlink_cur, (uint32_t)hlink);
                    *hlink |= ECHI_T;
                    usb_printf("period element added at right. \r\n");
                    break;
                }
            }

            // move to next qh
            hlink_prev = hlink_cur;
            hlink_cur  = (uint32_t *)(*hlink_cur);
        }
    }
}

/*
    Period qh is for USB interrupt transaction.
*/
static void install_periodic_qh(ehci_handle_t * handle, 
                                uint32_t addr, 
                                uint32_t ep,
                                ehci_speed_t speed,
                                uint32_t     interval)
{
    ECHI_QH_t  * qh;
    uint32_t package_len_max = 0;

    qh = qh_malloc();    
    if(qh == NULL)
    {
        usb_printf("***install_periodic_qh fail. \r\n");
        return;
    }
    
    if(speed == ehci_speed_high)
    {
        package_len_max = 1024;
    }
    else
    {
        package_len_max = 64; // interrupt transaction
    }
    
    qh->echar  = ECHI_QH_ECHAR_NAK_RL(15)                   |
                 ECHI_QH_ECHAR_MAX_PACKAGE(package_len_max) |
                 ECHI_QH_ECHAR_TOGGLE_BY_QTD                |
                 ECHI_QH_ECHAR_SPEED(speed)                 |
                 ECHI_QH_ECHAR_EP(ep)                       |
                 ECHI_QH_ECHAR_ADDR(addr);

    qh->ecapa = ECHI_QH_ECAPA_MULTI(3);
    qh->qtd.next_qtd = ECHI_T;
    
    qh->interval = interval;

    

    install_periodic_element(handle, &(qh->hlink));

    usb_printf("\r\n\r\n\r\n\r\n");
}

static void uninstall_periodic_qh(ehci_handle_t * handle, uint32_t addr, uint32_t ep)
{
    
}

#define EHCI_ITD_IN  1
#define EHCI_ITD_OUT 0
static ECHI_ITD_t * install_periodic_itd(ehci_handle_t * handle, 
                                         int32_t addr, 
                                         int32_t ep, 
                                         int32_t io, 
                                         int32_t max_pack_size,
                                         int32_t multi)
{
    ECHI_ITD_t * itd;
    itd = itd_malloc();
    if(itd == NULL)
    {
        usb_printf("***install_periodic_itd - memory allocate - fail. \r\n");
        return NULL;
    }
    itd->hlink = ECHI_HLINK_TYPE_ITD;
    itd->BP[0] = ECHI_ITD_BP0_EP(ep) | ECHI_ITD_BP0_ADDR(addr);
    itd->BP[1] = ECHI_ITD_BP1_IO(io) | ECHI_ITD_BP1_MAX_PACK(max_pack_size);
    itd->BP[2] = ECHI_ITD_BP2_MULTI(multi);

    install_periodic_element(handle, &(itd->hlink));
    return itd;
}
static void uninstall_periodic_itd(ehci_handle_t * handle, uint32_t addr, uint32_t ep)
{
    
}

static ECHI_QH_t * install_periodic_sitd(ehci_handle_t * handle,
                                         uint32_t addr,
                                         uint32_t ep, 
                                         uint32_t speed,
                                         uint32_t ep_type)
{
    
    return NULL;
}
static void uninstall_periodic_sitd(ehci_handle_t * handle, uint32_t addr, uint32_t ep)
{
    
}
static void init_periodic_schecule(ehci_handle_t * handle)
{
    ECHI_PFL_t * pfl;
    int i;
    
    pfl = pfl_malloc();
    if(pfl == NULL)
    {
        usb_printf("pfl_malloc fail. \r\n");
        return;
    }
    handle->pfl_table = pfl;
    handle->pfl_head = (void *)ECHI_T;

    for(i=0; i<CONFIG_PFL_SIZE; i++)
    {
        pfl->frame[i] = (void *)ECHI_T;
    }
    
    d_cache_flush(&pfl->frame[0], sizeof(ECHI_PFL_t));

    handle->usb->PERIODICLISTBASE = (uint32_t)(handle->pfl_table);
    usb_printf("PERIODICLISTBASE = %x. \r\n", handle->usb->PERIODICLISTBASE);
    
    handle->usb->USBCMD &= ~0x800C;

    switch(CONFIG_PFL_SIZE)
    {
        case 8:    handle->usb->USBCMD |= (1<<15) | (3<<2);  break;
        case 16:   handle->usb->USBCMD |= (1<<15) | (2<<2);  break;
        case 32:   handle->usb->USBCMD |= (1<<15) | (1<<2);  break;
        case 64:   handle->usb->USBCMD |= (1<<15) | (0<<2);  break;
        case 128:  handle->usb->USBCMD |= (0<<15) | (3<<2);  break;
        case 256:  handle->usb->USBCMD |= (0<<15) | (2<<2);  break;
        case 512:  handle->usb->USBCMD |= (0<<15) | (1<<2);  break;
        case 1024: handle->usb->USBCMD |= (0<<15) | (0<<2);  break;
    }
    
    echi_periodic_enable(handle->usb);
}


