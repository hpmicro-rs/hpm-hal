MEMORY
{
    XPI0_HEADER : ORIGIN = 0x80000000, LENGTH = 0x3000 /* bootheader */
    XPI0_APP    : ORIGIN = 0x80003000, LENGTH = 1024K - 0x3000 /* app firmware */
    DLM0        : ORIGIN = 0x00200000, LENGTH =  256K /* data local memory */
    ILM0        : ORIGIN = 0x00000000, LENGTH =  256K /* instruction local memory */
    AAXI_SRAM   : ORIGIN = 0x01200000, LENGTH = 1M
}
REGION_ALIAS("REGION_TEXT", XPI0_APP);
REGION_ALIAS("REGION_RODATA", XPI0_APP);
REGION_ALIAS("REGION_DATA", DLM0);
REGION_ALIAS("REGION_BSS", DLM0)
REGION_ALIAS("REGION_HEAP", DLM0);
REGION_ALIAS("REGION_STACK", DLM0);
REGION_ALIAS("REGION_FASTTEXT", ILM0);


/*
SECTIONS
{
    .fast : ALIGN(4)
    {
        . = ALIGN(8);
         __fast_load_addr__ = LOADADDR(.fast);
        __fast_start_addr__ = .;
        PROVIDE(__ramfunc_start__ = .);
        *(.fast)
        *(.fast.*)
        . = ALIGN(8);
        PROVIDE(__ramfunc_end__ = .);
        __fast_end_addr__ = .;
    } > ILM
}

*/
