





static void * x_malloc(void * mem_pool, uint32_t size, uint32_t cnt, uint8_t * allocated)
{
    int i;
    void * p = NULL;

    for(i=0; i<cnt; i++)
    {
        if(allocated[i] == UNALLOCATED)
        {
            allocated[i] = ALLOCATED;
            p = (uint8_t *)mem_pool + size * i;
            memset(p, 0, sizeof(ECHI_QH_t));
            break;
        }
    }
    
    return p; 
}

static void x_free(void * mem_pool, void * p_free, uint32_t size, uint32_t cnt, uint8_t * allocated)
{
    int i;
    for(i=0; i<cnt; i++)
    {
        if(p_free == (uint8_t *)mem_pool + size * i)
        {
            allocated[i] = UNALLOCATED;
            break;
        }
    }
}

typedef struct _MEM_Manage
{
    ECHI_QH_t  qh[CONFIG_QH_CNT];
    ECHI_QTD_t qtd[CONFIG_QTD_CNT];
    ECHI_ITD_t itd[CONFIG_ITD_CNT];
    uint8_t  qh_allocated[CONFIG_QH_CNT];
    uint8_t  qtd_allocated[CONFIG_QTD_CNT];
    uint8_t  itd_allocated[CONFIG_ITD_CNT];
    uint8_t  alignment[EHCI_BOUNDARY_ALIGNMENT];
} MEM_Manage_t;

static MEM_Manage_t mem_manage;
static MEM_Manage_t * p_mem_manage;

static void mem_manage_init(void)
{
    uint32_t addr;
    int i;
    memset(&mem_manage, 0, sizeof(MEM_Manage_t));
    
    addr  = (uint32_t)(&mem_manage);
    addr += EHCI_BOUNDARY_ALIGNMENT;
    addr &=  ~(EHCI_BOUNDARY_ALIGNMENT-1);
    p_mem_manage = (MEM_Manage_t *)addr;
}

static void * qh_malloc(void)
{   
    return x_malloc(&(p_mem_manage->qh[0]), sizeof(ECHI_QH_t), CONFIG_QH_CNT, &(p_mem_manage->qh_allocated[0])); 
}

static void qh_free(void * qh)
{
    x_free(&(p_mem_manage->qh[0]), qh, sizeof(ECHI_QH_t), CONFIG_QH_CNT, &(p_mem_manage->qh_allocated[0])); 
}
static void * qtd_malloc(void)
{
    return  x_malloc(&(p_mem_manage->qtd[0]), sizeof(ECHI_QTD_t), CONFIG_QTD_CNT, &(p_mem_manage->qtd_allocated[0])); ;
}

static void qtd_free(void * qtd)
{
    x_free(&(p_mem_manage->qtd[0]), qtd, sizeof(ECHI_QTD_t), CONFIG_QTD_CNT, &(p_mem_manage->qtd_allocated[0]));
}
static void * itd_malloc(void)
{   
    return  x_malloc(&(p_mem_manage->itd[0]), sizeof(ECHI_ITD_t), CONFIG_ITD_CNT, &(p_mem_manage->itd_allocated[0])); ;
}
static void itd_free(void * itd)
{
    x_free(&(p_mem_manage->itd[0]), itd, sizeof(ECHI_ITD_t), CONFIG_ITD_CNT, &(p_mem_manage->itd_allocated[0]));
}

/*
    --------------- 4K buf  --------------
*/
static char buf_4k[4096 * CONFIG_4K_BUF_CNT + 4096];
static char * p_buf_4k;
static char buf_4k_busy[CONFIG_4K_BUF_CNT];

static void buf_4k_init(void)
{
    uint32_t addr;
    addr = (uint32_t)buf_4k;
    addr += 4096;
    addr &= ~0xFFF;
    p_buf_4k = (char *)addr;
    memset(buf_4k_busy, 0, sizeof(buf_4k_busy));
}
static void * buf_4k_malloc(void)
{
    uint32_t i;
    void * r;
    for(i=0; i<CONFIG_4K_BUF_CNT; i++)
    {
        if(buf_4k_busy[i] == 0)
        {
            buf_4k_busy[i] = 1;
            r = p_buf_4k + i * 4096;
            memset(r, 0, 4096);
            return r;
        }
    }
    return NULL;
}
static void buf_4k_free(void * p)
{
    uint32_t i;
    
    p = (void *)((uint32_t)p & 0xfffff000);
    
    if(p == NULL)
        return;
    
    for(i=0; i<CONFIG_4K_BUF_CNT; i++)
    {
        if((p_buf_4k + i*4096) == p)
        {
            usb_printf("buf_4k_free ok, i = %d\r\n", i);
            buf_4k_busy[i] = 0;
            return;
        }
    }
}

static void * pfl_malloc(void)
{
    return buf_4k_malloc();
}

static void pfl_free(void * pfl)
{
    buf_4k_free(pfl);
}

