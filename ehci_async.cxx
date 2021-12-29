


static void echi_async_enable(USB_Type * usb)
{
    usb->USBCMD |= USB_USBCMD_ASE_MASK;
}
static void echi_async_disable(USB_Type * usb)
{
    usb->USBCMD &= ~USB_USBCMD_ASE_MASK;
}



static ECHI_QH_t  * install_async_qh(ehci_handle_t * handle, 
                                   uint32_t addr, 
                                   uint32_t ep, 
                                   ehci_speed_t speed,
                                   ep_type_t    ep_type)
{
    ECHI_QH_t  * qh;
    ECHI_QH_t  * qh_loop;
    uint32_t package_len_max = 0;

    qh = qh_malloc();    
    if(qh == NULL)
    {
        usb_printf("***install_async_qh fail. \r\n");
        return NULL;
    }
    
    if(ep_type == control)
    {
        package_len_max = 64;
    }
    
    qh->echar  = ECHI_QH_ECHAR_NAK_RL(15)                   |
                 ECHI_QH_ECHAR_MAX_PACKAGE(package_len_max) |
                 ECHI_QH_ECHAR_TOGGLE_BY_QTD                |
                 ECHI_QH_ECHAR_SPEED(speed)                 |
                 ECHI_QH_ECHAR_EP(ep)                       |
                 ECHI_QH_ECHAR_ADDR(addr);

    /*
    Control Endpoint Flag (C). 
    If the QH.EPS field indicates the endpoint is not a high-speed device, 
    and the endpoint is a control endpoint, then software must set this bit to a one. 
    Otherwise, it should always set this bit to zero.
    */
    if( (ep_type == control) && 
        ((speed == ehci_speed_full) || (speed == ehci_speed_low)) )
    {
        qh->echar |= ECHI_QH_ECHAR_C;
    }
    qh->ecapa = ECHI_QH_ECAPA_MULTI(3);
    qh->qtd.next_qtd = ECHI_T;


    if(handle->usb->ASYNCLISTADDR == 0)
    {
        // 1st qh, qh_hs(0, 0)
        qh->echar |= ECHI_QH_ECHAR_HEAD;
        qh->hlink  = (uint32_t)qh | ECHI_HLINK_TYPE_QH;
        handle->usb->ASYNCLISTADDR = (uint32_t)qh;
        handle->qh_hs00 = qh;
        usb_printf("qh header added. \r\n");
    }
    else
    {
        // add to the last of the queue
        qh->hlink = (uint32_t)handle->qh_hs00 | ECHI_HLINK_TYPE_QH;

        // go to last qh
        qh_loop = handle->qh_hs00;
        while((qh_loop->hlink & ECHI_HLINK_ADDR_MASK) != (uint32_t)(handle->qh_hs00) )
        {
            qh_loop = (ECHI_QH_t  *)(qh_loop->hlink & ECHI_HLINK_ADDR_MASK);
        }
        qh_loop->hlink = (uint32_t)qh | ECHI_HLINK_TYPE_QH;
        
        if(handle->usb->USBCMD & USB_USBCMD_ASE_MASK)
        {
            // wait until current qh is accessed by USB controller.
            while( (uint32_t)(handle->usb->ASYNCLISTADDR & ECHI_HLINK_ADDR_MASK) != ((uint32_t)handle->qh_hs00 & 0xFFFFFFE0) )
                ;
            
            usb_printf("new qh added dynamically\r\n");            
        }
        else
        {
            usb_printf("new qh added statically\r\n");    
        }
    }

    usb_printf("\r\n\r\n\r\n\r\n");
    return qh;
}
static void uninstall_async_qh(ehci_handle_t * handle, uint32_t addr, uint32_t ep)
{
    
}
#define ADDR0 0
#define EP0   0
static void init_async_schecule(ehci_handle_t * handle)
{
    install_async_qh(handle, ADDR0, EP0, ehci_speed_high, control);
    install_async_qh(handle, ADDR0, EP0, ehci_speed_full, control);
    echi_async_enable(handle->usb);
}
