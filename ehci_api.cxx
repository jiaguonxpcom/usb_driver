


static ECHI_ITD_t * create_iso_ep_hs(
    ehci_handle_t * handle,
    uint32_t     addr,
    uint32_t     ep,
    ep_type_t    ep_type,
    uint32_t     max_pack_size,
    uint32_t     multi,
    uint32_t     buf_4k_num)
{
    int32_t io;
    if(ep_type == iso_out)
    {
        io = EHCI_ITD_OUT;
    }
    else if(ep_type == iso_in)
    {
        io = EHCI_ITD_IN;
    }
    else
    {
        return NULL;
    }

    return install_periodic_itd(handle, addr, ep, io, max_pack_size, multi, buf_4k_num);
}

handle_t echi_create_iso_ep(
    ehci_handle_t * handle,
    uint32_t     addr, 
    uint32_t     ep, 
    ep_type_t    ep_type,
    ehci_speed_t speed,
    uint32_t     max_pack_size,
    uint32_t     multi,
    uint32_t     buf_4k_num)
{
    if(speed == ehci_speed_high)
    {
        return create_iso_ep_hs(handle, addr, ep, ep_type, max_pack_size, multi, buf_4k_num);
    }
    else
    {   

    }
}


static uint32_t write_iso_ep(void * handle, void * buf, uint32_t len, usb_callback call_back)
{
    ECHI_ITD_t * itd;
    int i;
    if((len == 0) || (len > ITD_SLOT_BYTES_MAX * ITD_SLOT_CNT))
    {
        return ECHI_ERROR;
    }

    itd = (ECHI_ITD_t *)handle;

    i = 0;
    while(len >= ITD_SLOT_BYTES_MAX)
    {
        itd_load_slot(itd, i, buf + i*ITD_SLOT_BYTES_MAX, ITD_SLOT_BYTES_MAX);
        i++;
        len -= ITD_SLOT_BYTES_MAX;
    }

    if(len > 0)
    {
        itd_load_slot(itd, i, buf + i*ITD_SLOT_BYTES_MAX, len);
    }

    return ECHI_OK;
}

uint32_t ehci_write_ep(void * handle, void * buf, uint32_t len, usb_callback call_back)
{
    uint32_t hlink;
    hlink = *(uint32_t *)handle;
    switch(hlink & ECHI_HLINK_TYPE_MASK)
    {
        case ECHI_HLINK_TYPE_ITD:
            write_iso_ep(handle, buf, len, call_back);
            break;
    }
}

uint32_t ehci_read_ep(void * handle, void * buf, uint32_t len, usb_callback call_back)
{
    ECHI_ITD_t * itd = handle;
}



