

#define ITD_ADDR_MASK        (~0x1f)
#define ITD_BP0_EP(n)        (n<<8)
#define ITD_BP0_ADDR(n)      (addr<<0)
#define ITD_BP1_IO(n)        (n<<11)
#define ITD_BP1_MAX_PACK(n)  (n<<0)
#define ITD_BP2_MULTI(n)     (n<<0)

#define ITD_SLOT_BYTES_MAX (1024 * 3)
#define ITD_SLOT_CNT 8
#define ITD_SLOT_PG_BYTES_MAX (1024 * 4)

#define ITD_TSC_STATUS_ACTIVE (1<<31)
#define ITD_TSC_TRANS_LEN(n)  (n<<16)
#define ITD_TSC_IOC           (1<<15)
#define ITD_TSC_PAGE(n)       (n<<12)
#define ITD_TSC_OFFSET(n)     (n<<0)


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
    usb_callback callback;
    uint32_t  callback_para;
    uint32_t  revserved[3];  // 5 word

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
static void update_pfl_table(ehci_handle_t * handle)
{
    ECHI_PFL_t * pfl_table;
    uint32_t   * hlink;

    pfl_table = handle->pfl_table;
    hlink     = handle->pfl_head;

    if(hlink == NULL)
    {
        return;
    }
    
    pfl_table->frame[0] = hlink;
}
static void dump_pfl_queue(ehci_handle_t * handle)
{
    uint32_t * hlink;
    uint32_t hlink_type;
    hlink = handle->pfl_head;

    usb_printf("dump_pfl_queue \r\n");

    if(hlink == NULL)
    {
        return;
    }

    while(1)
    {
        hlink_type = *hlink & ECHI_HLINK_TYPE_MASK;
        switch(hlink_type)
        {
            case ECHI_HLINK_TYPE_ITD:
                usb_printf("-->itd \r\n");
                break;
            case ECHI_HLINK_TYPE_QH:
                usb_printf("-->qh \r\n");
                break;
            case ECHI_HLINK_TYPE_SITD:
                usb_printf("-->sitd \r\n");
                break;
            case ECHI_HLINK_TYPE_FSTN:
                usb_printf("-->fstn \r\n");
                break;
        }

        if(*hlink & ECHI_T)
        {
            break;
        }
        else
        {
            hlink = hlink_next(hlink);
        }
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
        *hlink |= ECHI_T;
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
                update_hlink_address(hlink, (uint32_t)hlink_cur);

                // 2.hlink_prev point to new
                if(hlink_prev == NULL)
                {
                    handle->pfl_head = hlink;
                }
                else
                {
                    update_hlink_address(hlink_prev, (uint32_t)hlink);
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

    dump_pfl_queue(handle);
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


static void itd_load_slot(ECHI_ITD_t * itd, uint32_t slot, void * buf, uint32_t size)
{
    uint32_t page;
    uint32_t offset;

    //usb_printf("itd_load_slot %d \r\n", slot);
    page   = slot * ITD_SLOT_BYTES_MAX/ITD_SLOT_PG_BYTES_MAX;
    offset = slot * ITD_SLOT_BYTES_MAX % ITD_SLOT_PG_BYTES_MAX;
    //usb_printf("page = %d, offset = %d \r\n", page, offset);

    // copy data to 4k buf.

    itd->TSC[slot] = ITD_TSC_STATUS_ACTIVE   | 
                     ITD_TSC_TRANS_LEN(size) |
                     ITD_TSC_PAGE(page)      |
                     ITD_TSC_OFFSET(offset) ;
}


#define EHCI_ITD_IN  1
#define EHCI_ITD_OUT 0
#define EHCI_BUF_4K_NUM_MAX 6
static ECHI_ITD_t * install_periodic_itd(ehci_handle_t * handle, 
                                         uint32_t addr, 
                                         uint32_t ep, 
                                         uint32_t io, 
                                         uint32_t max_pack_size,
                                         uint32_t multi,
                                         uint32_t buf_4k_num)
{
    ECHI_ITD_t * itd;
    uint32_t i = 0;

    if(buf_4k_num > EHCI_BUF_4K_NUM_MAX)
    {
        return NULL;
    }

    itd = itd_malloc();
    if(itd == NULL)
    {
        usb_printf("***install_periodic_itd - itd memory allocate - fail. \r\n");
        return NULL;
    }

    for(i = 0; i < buf_4k_num; i++)
    {
        itd->BP[i] = (uint32_t)buf_4k_malloc();
        if(itd->BP[i] == (uint32_t)NULL)
        {
            usb_printf("***install_periodic_itd - 4k memory allocate - fail. \r\n");
            while(i > 0)
            {
                i--;
                buf_4k_free((void*)(itd->BP[i]));
                if(i == 0)
                {
                    itd_free(itd);
                    break;
                }
            }
            return NULL;
        }
        usb_printf("bp[%d] installed. \r\n", i);
    }

    itd->hlink = ECHI_HLINK_TYPE_ITD;
    itd->BP[0] |= ITD_BP0_EP(ep) | ITD_BP0_ADDR(addr);
    itd->BP[1] |= ITD_BP1_IO(io) | ITD_BP1_MAX_PACK(max_pack_size);
    itd->BP[2] |= ITD_BP2_MULTI(multi);

    install_periodic_element(handle, &(itd->hlink));
    update_pfl_table(handle);
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


