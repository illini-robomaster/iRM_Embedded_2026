#!/bin/bash
#
# Post-CubeMX generation script to fix linker script for DMA compatibility
# Run this after regenerating code with CubeMX
#
# Problem: STM32H7 DMA cannot access DTCM RAM (0x20000000)
# Solution: Move heap to AXI SRAM (0x24000000)
#

LINKER_FILE="stm32h723vgtx_flash.ld"
CUSTOM_LINKER="cmake/stm32h723vgtx_flash_custom.ld"

cd "$(dirname "$0")"

if [ ! -f "$LINKER_FILE" ]; then
    echo "Error: $LINKER_FILE not found"
    exit 1
fi

echo "Updating custom linker script from CubeMX generated version..."

# Copy the CubeMX generated file as base
cp "$LINKER_FILE" "$CUSTOM_LINKER"

# Fix 1: Change stack pointer to use AXI RAM instead of DTCM
sed -i '' 's/_estack = ORIGIN(DTCMRAM) + LENGTH(DTCMRAM);/_estack = ORIGIN(RAM) + LENGTH(RAM);    \/* end of AXI SRAM - DMA accessible *\//' "$CUSTOM_LINKER"

# Fix 2: Increase heap size for DMA buffers
sed -i '' 's/_Min_Heap_Size = 0x200;/_Min_Heap_Size = 0x4000;      \/* 16KB heap for DMA buffers *\//' "$CUSTOM_LINKER"

# Fix 3: Increase stack size
sed -i '' 's/_Min_Stack_Size = 0x400;/_Min_Stack_Size = 0x1000;     \/* 4KB stack *\//' "$CUSTOM_LINKER"
sed -i '' 's/_Min_Stack_Size = 0x800;/_Min_Stack_Size = 0x1000;     \/* 4KB stack *\//' "$CUSTOM_LINKER"

# Fix 4: Move .bss section to AXI RAM (for DMA buffers allocated statically)
sed -i '' 's/} >DTCMRAM$/} >RAM/' "$CUSTOM_LINKER"

# Fix 5: Add header comment
sed -i '' '1i\
/*\
 * Custom linker script for STM32H723VGTx - DMA Compatible\
 * Auto-generated from CubeMX linker script by fix_linker_script.sh\
 *\
 * IMPORTANT: STM32H7 DMA1/DMA2 cannot access DTCM RAM (0x20000000).\
 * Heap and stack are placed in AXI SRAM (0x24000000) for DMA compatibility.\
 */\
' "$CUSTOM_LINKER"

echo "Done! Custom linker script updated: $CUSTOM_LINKER"
echo ""
echo "Changes made:"
echo "  - Stack pointer moved to AXI SRAM"
echo "  - Heap size increased to 16KB"
echo "  - Stack size set to 4KB"  
echo "  - .bss section moved to AXI SRAM"
