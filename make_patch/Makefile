%.o: %.c
	arm-none-eabi-gcc -mcpu=cortex-m0 -mthumb -c $?

patch.elf: patch.ld patch.o
	arm-none-eabi-ld -o $@ -T $^

patch.hex: patch.elf
	objcopy -O ihex $? $@

patch_array.c: patch.hex
	srec_cat patch.hex -intel -o $@ -C-Array
