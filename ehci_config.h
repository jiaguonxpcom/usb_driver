


#define CONFIG_QH_CNT     3
#define CONFIG_QTD_CNT    3
#define CONFIG_ITD_CNT    5
#define CONFIG_ITD_BUF_4K 6 // each ITD use n 4K_buf, maximum value is 6
#define CONFIG_4K_BUF_CNT (4 + (CONFIG_ITD_CNT * CONFIG_ITD_BUF_4K))



/*
    Element size, it decide the maximum periodic interval
*/
#define CONFIG_PFL_SIZE   32


