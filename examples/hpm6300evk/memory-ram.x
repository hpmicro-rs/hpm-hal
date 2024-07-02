MEMORY
{
    ILM0       : ORIGIN = 0x00000000, LENGTH =  128K
    DLM0       : ORIGIN = 0x00080000, LENGTH =  128K /* data local memory */

    AXI_SRAM    : ORIGIN = 0x01200000, LENGTH = 512K
    AHB_SRAM    : ORIGIN = 0xF0300000, LENGTH = 32K
}
REGION_ALIAS("REGION_TEXT", ILM0);
REGION_ALIAS("REGION_RODATA", ILM0);
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