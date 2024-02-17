-- #################################################################################################
-- # << NEORV32 - CPU RVVI Package >>                                                              #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2024, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32       (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

package neorv32_cpu_rvvi_package is

-- ****************************************************************************************************************************
-- Declare a component for the trace module to allow synthesis to not read in the trace file
-- ****************************************************************************************************************************
	component neorv32_rvvi_trace is
	  generic (
	    -- General --
	    HART_ID                    : std_ulogic_vector(31 downto 0); -- hardware thread ID
	    VENDOR_ID                  : std_ulogic_vector(31 downto 0); -- vendor's JEDEC ID
	    CPU_BOOT_ADDR              : std_ulogic_vector(31 downto 0); -- cpu boot address
	    CPU_DEBUG_PARK_ADDR        : std_ulogic_vector(31 downto 0); -- cpu debug mode parking loop entry address, 4-byte aligned
	    CPU_DEBUG_EXC_ADDR         : std_ulogic_vector(31 downto 0); -- cpu debug mode exception entry address, 4-byte aligned
	    -- RISC-V CPU Extensions --
	    CPU_EXTENSION_RISCV_A      : boolean; -- implement atomic memory operations extension?
	    CPU_EXTENSION_RISCV_B      : boolean; -- implement bit-manipulation extension?
	    CPU_EXTENSION_RISCV_C      : boolean; -- implement compressed extension?
	    CPU_EXTENSION_RISCV_E      : boolean; -- implement embedded RF extension?
	    CPU_EXTENSION_RISCV_M      : boolean; -- implement mul/div extension?
	    CPU_EXTENSION_RISCV_U      : boolean; -- implement user mode extension?
	    CPU_EXTENSION_RISCV_Zfinx  : boolean; -- implement 32-bit floating-point extension (using INT regs)
	    CPU_EXTENSION_RISCV_Zicntr : boolean; -- implement base counters?
	    CPU_EXTENSION_RISCV_Zicond : boolean; -- implement integer conditional operations?
	    CPU_EXTENSION_RISCV_Zihpm  : boolean; -- implement hardware performance monitors?
	    CPU_EXTENSION_RISCV_Zmmul  : boolean; -- implement multiply-only M sub-extension?
	    CPU_EXTENSION_RISCV_Zxcfu  : boolean; -- implement custom (instr.) functions unit?
	    CPU_EXTENSION_RISCV_Sdext  : boolean; -- implement external debug mode extension?
	    CPU_EXTENSION_RISCV_Sdtrig : boolean; -- implement trigger module extension?
	    CPU_EXTENSION_RISCV_Smpmp  : boolean; -- implement physical memory protection?
	    -- Tuning Options --
	    FAST_MUL_EN                : boolean; -- use DSPs for M extension's multiplier
	    FAST_SHIFT_EN              : boolean; -- use barrel shifter for shift operations
	    REGFILE_HW_RST             : boolean; -- implement full hardware reset for register file
	    BOOT_ADDR_INPUT_EN         : boolean; -- use dedicated boot address input instead of parameter
	    -- Hardware Performance Monitors (HPM) --
	    HPM_NUM_CNTS               : natural range 0 to 13; -- number of implemented HPM counters (0..13)
	    HPM_CNT_WIDTH              : natural range 0 to 64;  -- total size of HPM counters (0..64)
	    RUNNING_IN_SIM             : boolean;
	    RUNNING_IN_FPGA            : boolean;
	    RVE_EN                     : boolean -- implement embedded RF extension    
	  );
	  port (
	    -- global control --
	    clk_i          : in  std_ulogic; -- global clock, rising edge
	    rstn_i         : in  std_ulogic; -- global reset, low-active, async
	    -- RVVI-Trace
	    rvvi_trace_valid_o      : out std_ulogic;   -- Retired instruction
	    rvvi_trace_insn_o       : out std_ulogic_vector(31 downto 0);   -- Instruction bit pattern
	    rvvi_trace_insn_valid_o : out std_ulogic; -- Instruction vector is valid
	    rvvi_trace_trap_o       : out std_ulogic;   -- Trapped instruction (External to Core, eg Memory Subsystem)
	    rvvi_trace_halt_o       : out std_ulogic;   -- Halted  instruction
	    rvvi_trace_intr_o       : out std_ulogic;   -- (RVFI Legacy) Flag first instruction of trap handler
	    rvvi_trace_mode_o       : out std_ulogic_vector(1 downto 0);   -- Privilege mode of operation
	    rvvi_trace_ixl_o        : out std_ulogic_vector(1 downto 0);   -- XLEN mode 32/64 bit
	    rvvi_trace_pc_rdata_o   : out std_ulogic_vector(31 downto 0);   -- PC of insn
	    rvvi_trace_pc_wdata_o   : out std_ulogic_vector(31 downto 0);   -- PC of next instruction
	    -- Processor registers
	    rvvi_trace_x_wdata_o    : out rvvi_xregfile_t;   -- X data value
	    rvvi_trace_x_wb_o       : out std_ulogic_vector((XLEN-1) downto 0);   -- X data writeback (change) flag
	    rvvi_trace_x_rf_wb_en_o : out std_ulogic; -- X data write back timing indicator
	    -- Control & State Registers
	    rvvi_trace_csr_o          : out rvvi_csr_t;   -- Full CSR Address range
	    rvvi_trace_csr_wb_o       : out std_ulogic_vector(4095 downto 0);   -- CSR writeback (change) flag
	    rvvi_trace_lrsc_cancel_o  : out std_ulogic   -- Implementation defined cancel
	  );
	end component;

end neorv32_cpu_rvvi_package;
