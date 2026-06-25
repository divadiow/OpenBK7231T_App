/* OpenOPL1000 split-M3 linker script.
 *
 * Normal M3 patch image remains in the standard APS patch window:
 *   0x004164A0 .. 0x00440000
 *
 * A second explicit output section is placed at the high end of the 16 KB M3
 * shared-memory window used by the vendor Expand_M3_RAM demo.  v29b proved code at 0x80000000 executes but the Wi-Fi connect path resets afterwards.
 * v29c proved the upper 1 KB survives Wi-Fi/DHCP; v30 proved 2 KB; v31 proved 3 KB; v33 proved 10 KB; v34 proved 14 KB; v35 proved 15 KB.
 * v35 proved the upper 15 KB tail; v36 proved selected micro-HTTP helpers can execute from it; v37 also moves the micro-UI strings and live HTTP buffers there:
 *   0x80000400 .. 0x80004000
 *
 * The build emits two M3 binaries from the same ELF:
 *   opl1000_app_m3_main.bin -> loaded at 0x004164A0
 *   opl1000_app_m3_shm.bin  -> loaded at 0x80000400
 */

MEMORY
{
    IRAM1 (rwx) : ORIGIN = 0x004164a0, LENGTH = 0x00029B60
    SHM   (rwx) : ORIGIN = 0x80000400, LENGTH = 0x00003C00
}

SECTIONS
{
    .entry ORIGIN(IRAM1) :
    {
        KEEP(*(ENTRY_POINT))
    } > IRAM1

    .text ALIGN(4) :
    {
        __text_start__ = .;
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.v4_bx)
        *(.vfp11_veneer)
        *(.ARM.extab* .gnu.linkonce.armextab.*)
        __text_end__ = .;
    } > IRAM1

    .rodata ALIGN(4) :
    {
        __rodata_start__ = .;
        *(.rodata)
        *(.rodata*)
        __rodata_end__ = .;
    } > IRAM1

    .data ALIGN(4) :
    {
        __data_start__ = .;
        *(.data)
        *(.data*)
        __data_end__ = .;
    } > IRAM1

    .bss ALIGN(4) (NOLOAD) :
    {
        __bss_start__ = .;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    } > IRAM1

    /*
     * libnosys.a's _sbrk() references the plain symbol "end".  The stock
     * Opulinks linker script provides it, and the split-M3 script must do the
     * same even though the runtime allocator is supplied by the SDK/CMSIS layer.
     */
    PROVIDE(end = __bss_end__);
    PROVIDE(_end = __bss_end__);
    PROVIDE(__end__ = __bss_end__);

    ASSERT(__bss_end__ <= ORIGIN(IRAM1) + LENGTH(IRAM1), "Error: IRAM1 patch window overflow")

    .shm_region ORIGIN(SHM) :
    {
        __shm_region_start__ = .;
        KEEP(*(SHM_REGION))
        KEEP(*(.SHM_REGION))
        KEEP(*(.SHM_REGION*))
        . = ALIGN(4);
        __shm_region_end__ = .;
    } > SHM

    ASSERT(__shm_region_end__ <= ORIGIN(SHM) + LENGTH(SHM), "Error: SHM split-M3 region overflow")

    /DISCARD/ :
    {
        *(.comment)
        *(.note*)
        *(.ARM.attributes)
    }
}
