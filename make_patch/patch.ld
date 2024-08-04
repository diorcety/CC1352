MEMORY
{
	FLASH (rx) : ORIGIN = 0x2100404C, LENGTH = 8K
}

SECTIONS
{
    .patch_vector :
    {
        *(.PROP_PATCH_VEC)
    } > FLASH

    .text :
    {
        . = ALIGN(4);
    } > FLASH
}
