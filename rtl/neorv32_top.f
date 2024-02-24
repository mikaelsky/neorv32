+incdir+core/
-y core/


// Packages
// Compile the definitions file
./core/neorv32_package.vhd

// Pick up the core files
// Note: -F is used to allow relative paths in the included .f file!!

// Include all the RISCV core source files
-F ./neorv32_core.f

// Include all the core specific peripherals
-F ./neorv32_core_peripherals.f

// Include the core debug interface
-F ./neorv32_debug.f

// Include the core memory definitions
-F ./neorv32_memories.f
-F ./neorv32_peripherals.f

// Memory architectures replace content of files to use real memories
// The memories need to be byte addressable and byte maskable.
// ./core/mem/neorv32_dmem.default.vhd
// ./core/mem/neorv32_imem.default.vhd

// Not needed, these are legacy usage memories
// ./core/mem/neorv32_dmem.legacy.vhd
// ./core/mem/neorv32_imem.legacy.vhd

// content of the boot ROM
 ./core/neorv32_bootloader_image.vhd

// The top level file that instantiates the core and all its peripherals
 ./core/neorv32_top.vhd

// The top view of the RISCV processor using std logic instead of defined packaging
// This should be replaced with a conversion to verilog 
// Modified!
// ./system_integration/neorv32_ProcessorTop_stdlogic.vhd

