


static void * create_iso_ep_hs(
    ehci_handle_t * handle,
    uint32_t     addr, 
    uint32_t     ep_number, 
    ep_type_t    ep_type,
    int32_t      max_pack_size,
    int32_t      multi)
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
}

void * echi_create_iso_ep(
    ehci_handle_t * handle,
    uint32_t     addr, 
    uint32_t     ep_number, 
    ep_type_t    ep_type,
    ehci_speed_t speed,
    int32_t      max_pack_size,
    int32_t      multi)
{
    if(speed == ehci_speed_high)
    {
        create_iso_ep_hs(handle, addr, ep_number, ep_type, max_pack_size, multi);
    }
    else
    {

    }
}



