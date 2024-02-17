-- #################################################################################################
-- # << NEORV32 CPU - RVVI Trace >>                                                                #
-- # ********************************************************************************************* #
-- #                                                                                               #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
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
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;
use neorv32.neorv32_cpu_control_package.all;
use neorv32.neorv32_cpu_regfile_package.all;

entity neorv32_rvvi_trace is
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
end neorv32_rvvi_trace;

architecture neorv32_rvvi_trace_rtl of neorv32_rvvi_trace is

  -- Signal replication types

  -- Control types
  -- HPM counter auto-configuration --
  constant hpm_num_c          : natural := cond_sel_natural_f(CPU_EXTENSION_RISCV_Zihpm, HPM_NUM_CNTS, 0);

  -- counter CSRs --
  type temp_rvvi_cnt_dat_t is array (0 to 2+hpm_num_c) of std_ulogic_vector(XLEN-1 downto 0);
--  signal cnt_lo_rd, cnt_hi_rd : temp_rvvi_cnt_dat_t;

  -- Regfile section
  -- auto-configuration --
  constant addr_bits_c : natural := cond_sel_natural_f(RVE_EN, 4, 5); -- address width
--  alias addr_bits_c is <<constant ^.^.neorv32_cpu_regfile_inst.addr_bits_c : natural>>;
  -- register file --
  type   rvvi_reg_file_t is array (31 downto 0) of std_ulogic_vector(XLEN-1 downto 0);
--  signal reg_file : rvvi_reg_file_t;

  -- RVVI Trace signals
  signal rvvi_trace_x_wb        : std_ulogic_vector(31 downto 0); -- Sample hold for the WB vector
  signal rvvi_trace_x_wb_en     : std_ulogic;                     -- Sample hold for the WB enable


  -- RVVI Trace --
  type rvvi_cnt_dat_t is array (0 to 31) of std_ulogic_vector(XLEN-1 downto 0);

  signal rvvi_cnt_lo_rd : rvvi_cnt_dat_t;
  signal rvvi_cnt_hi_rd : rvvi_cnt_dat_t;

  -- RVVI hold registers
  signal rvvi_trace_csr_wb            : std_ulogic_vector(4095 downto 0);
  signal rvvi_trace_valid             : std_ulogic;
  signal trap_ctrl_cause_5            : std_ulogic;
  signal debug_ctrl_running           : std_ulogic;
  signal trap_ctrl_env_enter          : std_ulogic;
  signal trap_ctrl_env_exit           : std_ulogic;
  signal rvvi_ignore_pc_we            : std_ulogic;
  signal rvvi_trace_insn              : std_ulogic_vector(XLEN-1 downto 0);
  signal rvvi_trace_insn_dly          : std_ulogic_vector(XLEN-1 downto 0);
  signal rvvi_trace_trap              : std_ulogic;
  signal rvvi_trace_halt              : std_ulogic;
  signal rvvi_trace_pc_rdata          : std_ulogic_vector(XLEN-1 downto 0);
  signal rvvi_trace_pc_wdata          : std_ulogic_vector(XLEN-1 downto 0);

  -- signal replication via alias --
  -- -------------------------------------------------------------------------------------------
  alias csr            is <<signal ^.^.neorv32_cpu_control_inst.csr : csr_t>>;
  alias execute_engine is <<signal ^.^.neorv32_cpu_control_inst.execute_engine : execute_engine_t>>;
  alias debug_ctrl     is <<signal ^.^.neorv32_cpu_control_inst.debug_ctrl : debug_ctrl_t>>;
  alias issue_engine   is <<signal ^.^.neorv32_cpu_control_inst.issue_engine : issue_engine_t>>;
  alias trap_ctrl      is <<signal ^.^.neorv32_cpu_control_inst.trap_ctrl : trap_ctrl_t>>;
  alias cnt_lo_rd      is <<signal ^.^.neorv32_cpu_control_inst.cnt_lo_rd : cnt_dat_t>>;
  alias cnt_hi_rd      is <<signal ^.^.neorv32_cpu_control_inst.cnt_hi_rd : cnt_dat_t>>;
  alias hpmevent_rd    is <<signal ^.^.neorv32_cpu_control_inst.hpmevent_rd : hpmevent_rd_t>>;

  -- Because Zfinx is an option extensions we cannot utilize alias
--    alias fflags_wb      is <<signal ^.^.neorv32_cpu_alu_inst.neorv32_cpu_cp_fpu_inst_true.neorv32_cpu_cp_fpu_inst.valid_o : std_ulogic>>;
--    alias fflags         is <<signal ^.^.neorv32_cpu_alu_inst.neorv32_cpu_cp_fpu_inst_true.neorv32_cpu_cp_fpu_inst.csr_fflags : std_ulogic_vector(4 downto 0)>>;
--    alias frm            is <<signal ^.^.neorv32_cpu_alu_inst.neorv32_cpu_cp_fpu_inst_true.neorv32_cpu_cp_fpu_inst.csr_frm : std_ulogic_vector(2 downto 0)>>;
  -- Instead we need to generate local signals and optionally path them based on the Zfinx option
  signal fflags    : std_ulogic_vector(4 downto 0);
  signal fflags_wb : std_ulogic;
  signal frm       : std_ulogic_vector(2 downto 0);

  -- Regfile aliases
-- Pathing: neorv32_cpu_inst:neorv32_cpu_regfile_inst:reg_file
-- Pathing: neorv32_cpu_inst:neorv32_rvvi_trace_inst_true:neorv32_rvvi_trace_inst:cnt_hi_rd
-- To get to cpu_inst level ^.^. to go up two levels
-- To go down to reg_file   neorv32_cpu_regfile_inst to go down 1 level into the regfile
-- Path should be: ^.^.neorv32_cpu_regfile_inst
  alias reg_file       is <<signal ^.^.neorv32_cpu_regfile_inst.reg_file : reg_file_t>>;
  alias rf_we_sel      is <<signal ^.^.neorv32_cpu_regfile_inst.rf_we_sel : std_ulogic_vector(31 downto 0)>>;
  alias ctrl_i         is <<signal ^.^.neorv32_cpu_regfile_inst.ctrl_i : ctrl_bus_t>>;

  -- Because PMP is an option extensions we cannot utilize alias
  --alias rvvi_pmp_cfg   is <<signal ^.^.rvvi_pmp_cfg  : rvvi_pmp_cfg_t>>;
  --alias rvvi_pmp_addr  is <<signal ^.^.rvvi_pmp_addr : rvvi_pmp_addr_t>>;
  signal rvvi_pmp_cfg  : rvvi_pmp_cfg_t;
  signal rvvi_pmp_addr : rvvi_pmp_addr_t;

begin

-- ****************************************************************************************************************************
-- RVVI Trace port
-- ****************************************************************************************************************************

  -- Trace port assignment --
  -- -------------------------------------------------------------------------------------------
  -- Only path to the FPU if Zfinx is enabled  
  rvvi_zfinx_true:
    if (CPU_EXTENSION_RISCV_Zfinx) generate
      -- add additional path up as we are inside a generate statement
      fflags_wb  <= <<signal ^.^.^.neorv32_cpu_alu_inst.neorv32_cpu_cp_fpu_inst_true.neorv32_cpu_cp_fpu_inst.valid_o : std_ulogic>>;
      fflags     <= <<signal ^.^.^.neorv32_cpu_alu_inst.neorv32_cpu_cp_fpu_inst_true.neorv32_cpu_cp_fpu_inst.csr_fflags : std_ulogic_vector(4 downto 0)>>;
      frm        <= <<signal ^.^.^.neorv32_cpu_alu_inst.neorv32_cpu_cp_fpu_inst_true.neorv32_cpu_cp_fpu_inst.csr_frm : std_ulogic_vector(2 downto 0)>>;
    end generate;

  rvvi_zfinx_false:
    if (not CPU_EXTENSION_RISCV_Zfinx) generate
      -- if Zfinx is disabled these will always be 0.
      fflags_wb  <= '0';
      fflags     <= (others => '0');
      frm        <= (others => '0');
    end generate;

  -- Only path to the PMP if PMP is enabled  
    rvvi_pmp_true:
      if (CPU_EXTENSION_RISCV_Smpmp) generate
        rvvi_pmp_cfg  <= <<signal ^.^.^.pmp_inst_true.neorv32_cpu_pmp_inst.rvvi_pmp_cfg  : rvvi_pmp_cfg_t>>;
        rvvi_pmp_addr <= <<signal ^.^.^.pmp_inst_true.neorv32_cpu_pmp_inst.rvvi_pmp_addr : rvvi_pmp_addr_t>>;
      end generate;

    rvvi_pmp_false:
      if (not CPU_EXTENSION_RISCV_Smpmp) generate
        rvvi_pmp_cfg  <= (others => (others => '0'));
        rvvi_pmp_addr <= (others => (others => '0'));
      end generate;

  -- These RVVI trace signals are constance for this specific core
  rvvi_trace_intr_o  <= '0';
  rvvi_trace_mode_o  <= (csr.privilege,csr.privilege);
  rvvi_trace_ixl_o   <= "00"; -- we are 32-bit only

  -- Non RVVI trace standard signal.
  -- We issue this signal to indicate that an instructions has been loaded into the pipeline
  -- this helps with debugging instructions that do not result in a valid out.
  rvvi_trace_insn_valid_o <= issue_engine.ack;

  -- Delay the valid signal by 1 clock cycle to ensure we can capture the output
  -- off all the registers being updated at the end of an instruction
  -- Other signals in this process are used to capture various states and delay by 1 clock to realign with valid.
  rvvi_trace_update_delay: process(clk_i, rstn_i)
  begin
    if (rstn_i = '0') then
      rvvi_trace_valid    <= '0';
      rvvi_trace_insn     <= (others => '0');
      rvvi_trace_trap     <= '0';
      rvvi_trace_halt     <= '0';
      rvvi_trace_pc_rdata <= (others => '0');
      rvvi_trace_pc_wdata <= (others => '0');
      rvvi_trace_insn_dly <= (others => '0');
    elsif rising_edge(clk_i) then 
      -- Generate the valid signal based on updating the program counter
      -- This as we need to grab signals at the end of an instruction not at the begining
      rvvi_trace_valid    <= execute_engine.pc_we when (rvvi_ignore_pc_we='1') else '0'; -- block first pc_we to prevent imperas compare being out of sync.

      -- Capture and hold the instructions when the issue engine ACKs
      -- Introduce a pipeline delay to align insn with the delayed valid signal.
      rvvi_trace_insn_dly     <= rvvi_trace_insn;
      if (issue_engine.ack = '1') then 
        rvvi_trace_insn     <= issue_engine.data(31 downto 0) when issue_engine.valid = "11" else ("0000000000000000" & issue_engine.ci_i16);
      end if;

      -- Capture a trap occured.
      rvvi_trace_trap     <= '1' when execute_engine.state = TRAP_ENTER else '0'; -- Note can also be trap-execute

      -- Indicate a halt occured if debugging is enabled.
      rvvi_trace_halt     <= '1' when ((trap_ctrl.cause(5) = '1') and (CPU_EXTENSION_RISCV_Sdext = true) and (execute_engine.state = TRAP_ENTER)) else '0';

      -- Capture and hold the current program counter
      rvvi_trace_pc_rdata <= execute_engine.pc;

      -- Capture and hold the next program counter
      rvvi_trace_pc_wdata <= execute_engine.next_pc;
    end if;
  end process rvvi_trace_update_delay;

  -- Assign generated signals to the RVVI output signals
  rvvi_trace_valid_o    <= rvvi_trace_valid;
  rvvi_trace_insn_o     <= rvvi_trace_insn_dly;
  rvvi_trace_trap_o     <= rvvi_trace_trap; -- Note can also be trap-execute
  rvvi_trace_halt_o     <= rvvi_trace_halt;
  rvvi_trace_pc_rdata_o <= rvvi_trace_pc_rdata;
  rvvi_trace_pc_wdata_o <= rvvi_trace_pc_wdata;

  -- The signal set here is to capture various sub-states inside the controller.
  -- Using the information capture we can update our RVVI states correct.
  -- * rvvi_ignore_pc_we   is used to gate out a valid signal that occurs during resets and trap events
  -- * trap_ctrl_cause_5   is used to indicate that CSRs are getting updated during a trap
  -- * trap_ctrl_env_enter is used to know when we are entering a trap
  -- * trap_ctrl_env_exit  is used to know when we are leaving a trap
  rvvi_trace_csr_write_back_delay: process(clk_i, rstn_i)
  begin 
    if (rstn_i = '0') then 
      trap_ctrl_cause_5   <= '0';
      debug_ctrl_running  <= '0';
      trap_ctrl_env_enter <= '0';
      trap_ctrl_env_exit  <= '0';
      rvvi_ignore_pc_we   <= '0';
    elsif rising_edge(clk_i) then
      -- capture and hold signals
      -- could potentially be removed for now they exist
      trap_ctrl_cause_5   <= trap_ctrl.cause(5);
      debug_ctrl_running  <= debug_ctrl.running;
      trap_ctrl_env_enter <= trap_ctrl.env_enter;
      trap_ctrl_env_exit  <= trap_ctrl.env_exit;

      -- if we entered a trap ignore the next pc.we as the is the trap destination write to PC
      if (execute_engine.state = TRAP_ENTER) then
        rvvi_ignore_pc_we <= '0';
      end if;

      -- Ensure that we ignore the first PC update after reset
      if ((rvvi_ignore_pc_we = '0') and (execute_engine.pc_we = '1')) then
        rvvi_ignore_pc_we <= '1';
      end if;
    end if;
  end process;

  -- Generate a full set of HPM counters for easy RVVI CSR vector generation
  hpm_counter_generate: process(cnt_lo_rd, cnt_hi_rd)
    variable n : integer;
  begin
    for n in 0 to 31 loop
      if (n < 2+hpm_num_c) then
        rvvi_cnt_lo_rd(n) <= cnt_lo_rd(n);
        rvvi_cnt_hi_rd(n) <= cnt_hi_rd(n);
      else
        -- TODO add counters driven by HPM count
        rvvi_cnt_lo_rd(n) <= (others => '0');
        rvvi_cnt_hi_rd(n) <= (others => '0');
      end if;
    end loop;
  end process;


  -- Capture the content of the register file and place it on the trace port
  rvvi_trace_regfiles: process(reg_file, rstn_i) -- rstn_i included to trigger initial set of rvvi_trace
    variable i : integer;
  begin
    rvvi_trace_x_wdata_o(0) <= (others => '0');
    for i in 1 to 31 loop
      rvvi_trace_x_wdata_o(i) <= reg_file(i);
      if ((i > 15) and (RVE_EN = true)) then
        rvvi_trace_x_wdata_o(i) <= (others => '0');
      end if;
    end loop;       
  end process;
  
  -- As the WB signals are only asserted for 1 cycle we need to create a capture and hold circuit
  -- This takes the write signals and holds them until we see a valid.
  rvvi_trace_regfiles_wb_hold: process(clk_i, rstn_i)
  begin
    if (rstn_i = '0') then
      rvvi_trace_x_wb    <= (others => '0');
      rvvi_trace_x_wb_en <= '0';
    elsif (rising_edge(clk_i)) then
      -- Remember our write backs
      rvvi_trace_x_wb    <= rvvi_trace_x_wb;
      rvvi_trace_x_wb_en <= rvvi_trace_x_wb_en;
      -- If we haven't seen a register writeback allow delays to be updated
      if (rvvi_trace_x_wb_en = '0') then 
        if (RVE_EN = false) then
          rvvi_trace_x_wb <= rf_we_sel;
        else
          rvvi_trace_x_wb(15 downto 0) <= rf_we_sel(15 downto 0);
        end if;
        rvvi_trace_x_wb_en             <= ctrl_i.rf_wb_en;
      end if;
      -- When an instruction is retired clear the writeback dly signals
      if (rvvi_trace_valid = '1') then
        rvvi_trace_x_wb                <= (others => '0');
        rvvi_trace_x_wb_en             <= '0';
      end if;
    end if;
  end process;
  -- We set the register write back outputs to the actual signal change
  rvvi_trace_x_wb_o       <= rvvi_trace_x_wb; -- rvvi_trace_x_wb OR 
  rvvi_trace_x_rf_wb_en_o <= rvvi_trace_x_wb_en; -- ctrl_i.rf_wb_en OR 


  -- Generate the RVVI trace CSR write back signal. This is a 1-hot vector that
  -- indicate which CSR registers got updated during the execution of this 
  -- instructions.
  rvvi_trace_csr_write_back: process(clk_i, rstn_i)
    variable i : integer;
  begin
    -- Clear the write back vector during reset
    if (rstn_i = '0') then 
        rvvi_trace_csr_wb <= (others => '0');
    elsif (rising_edge(clk_i)) then
      -- Clear the write back vector when a given instruction is retired.
      if (rvvi_trace_valid = '1') then
        rvvi_trace_csr_wb <= (others => '0');
      end if;

      -- capture floating point flag update
      -- indicate CSR update
      if (fflags_wb = '1') then
        rvvi_trace_csr_wb(1) <= '1';
        rvvi_trace_csr_wb(3) <= '1';
      end if;

      -- If a CSRW instruction is running set the appropriate bit in the WB vector to '1'
      if (csr.we = '1') then 
        rvvi_trace_csr_wb(to_integer(unsigned(csr.addr(11 downto 0)))) <= '1';
        -- if we are writing to any of the float CSRs (0x001 or 0x002) also indicate the combined float CSR is updated.
        if ((to_integer(unsigned(csr.addr(11 downto 0))) = 1) or (to_integer(unsigned(csr.addr(11 downto 0))) = 2)) then
          rvvi_trace_csr_wb(3) <= '1';
        end if;
      end if;

      -- If the address picked for CSR is the the combined float CSR register
      -- set the write back for the FRM and FFLAGS the csr.we
      if (to_integer(unsigned(csr.addr(11 downto 0))) = to_integer(unsigned(csr_fcsr_c))) then 
        rvvi_trace_csr_wb(to_integer(unsigned(csr_frm_c))) <= csr.we;
        rvvi_trace_csr_wb(to_integer(unsigned(csr_fflags_c))) <= csr.we;
      end if;

      -- If we enter a trap a group of CSR registers get updated, this basically replicated
      -- the trap controller conditions
      if (trap_ctrl_env_enter = '1') then          
        if (CPU_EXTENSION_RISCV_Sdext = false) or ((trap_ctrl_cause_5 = '0') and (debug_ctrl_running = '0')) then
          -- mepc   0x341
          rvvi_trace_csr_wb(833) <= '1'; -- set the MTVAL address as updated.
          -- mcause 0x342
          rvvi_trace_csr_wb(834) <= '1'; -- set the MTVAL address as updated.
          -- mtval  0x343
          rvvi_trace_csr_wb(835) <= '1'; -- set the MTVAL address as updated.
          -- privilege
          -- mstatus 0x300
          rvvi_trace_csr_wb(768) <= '1'; -- set the MSTATUS address as updated.  
          -- mtinst 0x34A
          rvvi_trace_csr_wb(842) <= '1'; -- set the MTINST address as updated.  
        end if;
        if ((CPU_EXTENSION_RISCV_Sdext = true) and (trap_ctrl_cause_5 = '1') and (debug_ctrl_running = '0')) then
          -- DCSR   0x7B0
          rvvi_trace_csr_wb(1968) <= '1'; -- set the DCSR address as updated.  
          -- DPC    0x7B1
          rvvi_trace_csr_wb(1969) <= '1'; -- set the DPC  address as updated.  
        end if;
      end if;

      -- when we exit a trap flag MSTATUS as being updated.        
      if (trap_ctrl_env_exit = '1') then 
        -- mstatus 0x300
        rvvi_trace_csr_wb(768) <= '1'; -- set the MSTATUS address as updated.  
      end if;
    end if;
  end process;

  -- If csr.we is asserted bypass the register. This is for CSRW instructions.
  -- This ensures that we assert CSR_WB as the write happens, which can co-incide with a valid assertion.
  rvvi_generate_csr_wb_output: process (rvvi_trace_csr_wb, csr.we, csr.addr) begin
    rvvi_trace_csr_wb_o <= rvvi_trace_csr_wb;
    if (csr.we = '1') then 
      rvvi_trace_csr_wb_o(to_integer(unsigned(csr.addr(11 downto 0)))) <= '1';
    end if;
  end process rvvi_generate_csr_wb_output;

  -- Control & State Registers
  -- dynamic assigns
  -- rstn_i is on the sensitivity list for force an update of static values when reset triggers.
  rvvi_generate_csr_vector: process(rstn_i, rvvi_cnt_lo_rd, rvvi_cnt_hi_rd, csr, hpmevent_rd, fflags, frm, trap_ctrl) begin
    -- default csr trace vector to 0
    rvvi_trace_csr_o <= (others => (others => '0'));
    -- Zero CSR
    -- Unprivileged Floating-Point CSR
    rvvi_trace_csr_o(to_integer(unsigned(csr_fflags_c)))        <= (31 downto 5 => '0') & fflags       when (CPU_EXTENSION_RISCV_Zfinx) else (others => '0'); -- fflags
    rvvi_trace_csr_o(to_integer(unsigned(csr_frm_c)))           <= (31 downto 3 => '0') & frm          when (CPU_EXTENSION_RISCV_Zfinx) else (others => '0'); -- frm
    rvvi_trace_csr_o(to_integer(unsigned(csr_fcsr_c)))          <= (31 downto 8 => '0') & frm & fflags when (CPU_EXTENSION_RISCV_Zfinx) else (others => '0'); -- fcsr
    -- Unprivleged Counter/Timers
    rvvi_trace_csr_o(to_integer(unsigned(csr_cycle_c)))         <= rvvi_cnt_lo_rd(00);
    rvvi_trace_csr_o(to_integer(unsigned(csr_time_c)))          <= (others => '0');  -- time
    rvvi_trace_csr_o(to_integer(unsigned(csr_instret_c)))       <= rvvi_cnt_lo_rd(02); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter3_c )))  <= rvvi_cnt_lo_rd(03); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter4_c )))  <= rvvi_cnt_lo_rd(04); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter5_c )))  <= rvvi_cnt_lo_rd(05); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter6_c )))  <= rvvi_cnt_lo_rd(06); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter7_c )))  <= rvvi_cnt_lo_rd(07); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter8_c )))  <= rvvi_cnt_lo_rd(08); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter9_c )))  <= rvvi_cnt_lo_rd(09); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter10_c)))  <= rvvi_cnt_lo_rd(10); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter11_c)))  <= rvvi_cnt_lo_rd(11); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter12_c)))  <= rvvi_cnt_lo_rd(12); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter13_c)))  <= rvvi_cnt_lo_rd(13); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter14_c)))  <= rvvi_cnt_lo_rd(14); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter15_c)))  <= rvvi_cnt_lo_rd(15); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_cycleh_c)))        <= rvvi_cnt_hi_rd(0);
    rvvi_trace_csr_o(to_integer(unsigned(csr_timeh_c)))         <= (others => '0');  -- timeh
    rvvi_trace_csr_o(to_integer(unsigned(csr_instret_c)))       <= rvvi_cnt_hi_rd(02); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter3h_c ))) <= rvvi_cnt_hi_rd(03); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter4h_c ))) <= rvvi_cnt_hi_rd(04); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter5h_c ))) <= rvvi_cnt_hi_rd(05); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter6h_c ))) <= rvvi_cnt_hi_rd(06); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter7h_c ))) <= rvvi_cnt_hi_rd(07); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter8h_c ))) <= rvvi_cnt_hi_rd(08); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter9h_c ))) <= rvvi_cnt_hi_rd(09); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter10h_c))) <= rvvi_cnt_hi_rd(10); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter11h_c))) <= rvvi_cnt_hi_rd(11); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter12h_c))) <= rvvi_cnt_hi_rd(12); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter13h_c))) <= rvvi_cnt_hi_rd(13); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter14h_c))) <= rvvi_cnt_hi_rd(14); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_hpmcounter15h_c))) <= rvvi_cnt_hi_rd(15); 
    -- Supervisor Trap Setup
    -- Supervisor Configuation
    -- Supervisor Trap Handling
    -- Supervisor Protection and Translation
    -- Debug/Trace register
    -- Hypervisor Trap Setup
    -- Hypervisor Trap Handling
    -- Hypervisor Configuation
    -- Supervisor Protection and Translation
    -- Debug/Trace register
    -- Hypervisor Counter/Timer virtulization registers
    -- Virtual Supervisor Registers
    -- Machine Information Registers
    rvvi_trace_csr_o(to_integer(unsigned(csr_mvendorid_c))) <= VENDOR_ID;
    rvvi_trace_csr_o(to_integer(unsigned(csr_marchid_c))  ) <= std_ulogic_vector(to_unsigned(archid_c,XLEN)); -- marchid
    rvvi_trace_csr_o(to_integer(unsigned(csr_mimpid_c))   ) <= hw_version_c; -- mimpid   
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhartid_c))  ) <= HART_ID; -- mhartid
    -- machine extended ISA extensions information --
    -- extended ISA (sub-)extensions --
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(31) <= (bool_to_ulogic_f(FAST_SHIFT_EN));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(30) <= (bool_to_ulogic_f(FAST_MUL_EN));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(30) <= (bool_to_ulogic_f(REGFILE_HW_RST));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(29 downto 22) <= (others => '0');
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(21) <= (bool_to_ulogic_f(RUNNING_IN_FPGA));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(20) <= (bool_to_ulogic_f(RUNNING_IN_SIM));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(19 downto 12) <= (others => '0');
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(11) <= (bool_to_ulogic_f(CPU_EXTENSION_RISCV_Sdtrig));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(10) <= (bool_to_ulogic_f(CPU_EXTENSION_RISCV_Sdext));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(09) <= (bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zihpm));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(08) <= (bool_to_ulogic_f(CPU_EXTENSION_RISCV_Smpmp));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(07) <= (bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicntr));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(06) <= (bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zicond));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(05) <= (bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zfinx));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(04) <= (bool_to_ulogic_f(CPU_EXTENSION_RISCV_U));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(03) <= (bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zxcfu));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(02) <= (bool_to_ulogic_f(CPU_EXTENSION_RISCV_Zmmul));
    rvvi_trace_csr_o(to_integer(unsigned(csr_mxisa_c)))(01 downto 00) <= (others => '0');

    rvvi_trace_csr_o(to_integer(unsigned(csr_mstatus_c)))   <= (31 downto 22 => '0') & (csr.mstatus_tw and bool_to_ulogic_f(CPU_EXTENSION_RISCV_U)) & "000" & csr.mstatus_mprv & "0000" & csr.mstatus_mpp & csr.mstatus_mpp & "000" & csr.mstatus_mpie & "000" & csr.mstatus_mie & "000"; -- mstatus
    rvvi_trace_csr_o(to_integer(unsigned(csr_misa_c))   )   <= ("01" & "000000" & "1" & "00" & bool_to_ulogic_f(CPU_EXTENSION_RISCV_U) & "0000000" & bool_to_ulogic_f(CPU_EXTENSION_RISCV_M) & "000" & not bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) & "000" & bool_to_ulogic_f(CPU_EXTENSION_RISCV_E) & "0" & bool_to_ulogic_f(CPU_EXTENSION_RISCV_C) & bool_to_ulogic_f(CPU_EXTENSION_RISCV_B) & "0"); -- misa
    rvvi_trace_csr_o(to_integer(unsigned(csr_mie_c))    )   <= (csr.mie_firq & "0000" & csr.mie_mei & "000" & csr.mie_mti & "000" & csr.mie_msi & "000"); -- mie
    -- Assume we are not updating MTVEC.
    rvvi_trace_csr_o(to_integer(unsigned(csr_mtvec_c))  )   <= csr.mtvec(XLEN-1 downto 0); -- mtvec
    rvvi_trace_csr_o(to_integer(unsigned(csr_mcounteren_c)))(2 downto 0)                     <= csr.mcounteren & '0' & csr.mcounteren when ((CPU_EXTENSION_RISCV_U = true) and (CPU_EXTENSION_RISCV_Zicntr = true)) else (others => '0'); -- mcounteren 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mcounteren_c)))((HPM_NUM_CNTS+3)-1 downto 3)    <= (others => csr.mcounteren) when ((CPU_EXTENSION_RISCV_U = true) and (CPU_EXTENSION_RISCV_Zihpm = true) and (HPM_NUM_CNTS > 0)) else (others => '0'); -- mcounteren 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mcounteren_c)))(XLEN-1 downto (HPM_NUM_CNTS+3)) <= (others => '0');
    -- Machine Trap Handling
    rvvi_trace_csr_o(to_integer(unsigned(csr_mscratch_c)))   <= csr.mscratch; -- mscratch  
    -- Trap handling
    rvvi_trace_csr_o(to_integer(unsigned(csr_mepc_c))    )   <= csr.mepc(XLEN-1 downto 1) & '0'; -- mepc      
    rvvi_trace_csr_o(to_integer(unsigned(csr_mcause_c))  )   <= csr.mcause(5) & (30 downto 5 => '0') & csr.mcause(4 downto 0); -- mcause
    rvvi_trace_csr_o(to_integer(unsigned(csr_mtval_c))   )   <= csr.mtval; -- mtval 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mtinst_c))  )   <= csr.mtinst; -- mtinst 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mip_c))     )   <= (trap_ctrl.irq_pnd(irq_firq_15_c downto irq_firq_0_c) & "0000" & trap_ctrl.irq_pnd(irq_mei_irq_c) & "000" & trap_ctrl.irq_pnd(irq_mti_irq_c) & "000" & trap_ctrl.irq_pnd(irq_msi_irq_c) & "000"); -- mip   
    -- Machine Configuration
    -- Machine Memory Protection
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpcfg0_c)))   <= rvvi_pmp_cfg(0);
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpcfg1_c)))   <= rvvi_pmp_cfg(1);
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpcfg2_c)))   <= rvvi_pmp_cfg(2);
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpcfg3_c)))   <= rvvi_pmp_cfg(3);
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr0_c ))) <= rvvi_pmp_addr( 0); -- pmpaddr0
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr1_c ))) <= rvvi_pmp_addr( 1); -- pmpaddr1
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr2_c ))) <= rvvi_pmp_addr( 2); -- pmpaddr2
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr3_c ))) <= rvvi_pmp_addr( 3); -- pmpaddr3
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr4_c ))) <= rvvi_pmp_addr( 4); -- pmpaddr4
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr5_c ))) <= rvvi_pmp_addr( 5); -- pmpaddr5
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr6_c ))) <= rvvi_pmp_addr( 6); -- pmpaddr6
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr7_c ))) <= rvvi_pmp_addr( 7); -- pmpaddr7
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr8_c ))) <= rvvi_pmp_addr( 8); -- pmpaddr8
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr9_c ))) <= rvvi_pmp_addr( 9); -- pmpaddr9
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr10_c))) <= rvvi_pmp_addr(10); -- pmpaddr10
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr11_c))) <= rvvi_pmp_addr(11); -- pmpaddr11
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr12_c))) <= rvvi_pmp_addr(12); -- pmpaddr12
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr13_c))) <= rvvi_pmp_addr(13); -- pmpaddr13
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr14_c))) <= rvvi_pmp_addr(14); -- pmpaddr14
    rvvi_trace_csr_o(to_integer(unsigned(csr_pmpaddr15_c))) <= rvvi_pmp_addr(15); -- pmpaddr15
    -- Machine Counter/Timers
    rvvi_trace_csr_o(to_integer(unsigned(csr_mcycle_c))  )       <= rvvi_cnt_lo_rd(00) when (CPU_EXTENSION_RISCV_Zicntr) else (others => '0');
    rvvi_trace_csr_o(to_integer(unsigned(csr_minstret_c)))       <= rvvi_cnt_lo_rd(02) when (CPU_EXTENSION_RISCV_Zicntr) else (others => '0');
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter3_c )))  <= rvvi_cnt_lo_rd(03); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter4_c )))  <= rvvi_cnt_lo_rd(04); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter5_c )))  <= rvvi_cnt_lo_rd(05); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter6_c )))  <= rvvi_cnt_lo_rd(06); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter7_c )))  <= rvvi_cnt_lo_rd(07); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter8_c )))  <= rvvi_cnt_lo_rd(08); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter9_c )))  <= rvvi_cnt_lo_rd(09); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter10_c)))  <= rvvi_cnt_lo_rd(10); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter11_c)))  <= rvvi_cnt_lo_rd(11); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter12_c)))  <= rvvi_cnt_lo_rd(12); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter13_c)))  <= rvvi_cnt_lo_rd(13); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter14_c)))  <= rvvi_cnt_lo_rd(14); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter15_c)))  <= rvvi_cnt_lo_rd(15); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mcycleh_c))  )      <= rvvi_cnt_hi_rd(00) when (CPU_EXTENSION_RISCV_Zicntr) else (others => '0'); -- mcycle       
    rvvi_trace_csr_o(to_integer(unsigned(csr_minstreth_c)))      <= rvvi_cnt_hi_rd(02) when (CPU_EXTENSION_RISCV_Zicntr) else (others => '0'); -- mcycle       
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter3h_c ))) <= rvvi_cnt_hi_rd(03); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter4h_c ))) <= rvvi_cnt_hi_rd(04); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter5h_c ))) <= rvvi_cnt_hi_rd(05); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter6h_c ))) <= rvvi_cnt_hi_rd(06); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter7h_c ))) <= rvvi_cnt_hi_rd(07); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter8h_c ))) <= rvvi_cnt_hi_rd(08); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter9h_c ))) <= rvvi_cnt_hi_rd(09); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter10h_c))) <= rvvi_cnt_hi_rd(10); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter11h_c))) <= rvvi_cnt_hi_rd(11); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter12h_c))) <= rvvi_cnt_hi_rd(12); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter13h_c))) <= rvvi_cnt_hi_rd(13); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter14h_c))) <= rvvi_cnt_hi_rd(14); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmcounter15h_c))) <= rvvi_cnt_hi_rd(15); 
    -- Machine Non-Maskable Interrupt Handling
    -- Machine Counter Setup
    rvvi_trace_csr_o(to_integer(unsigned(csr_mcountinhibit_c)))(2 downto 0)                     <= csr.mcountinhibit(2) & '0' & csr.mcountinhibit(0) when (CPU_EXTENSION_RISCV_U = true) else (others => '0'); -- mcounteren 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mcountinhibit_c)))((HPM_NUM_CNTS+3)-1 downto 3)    <= csr.mcountinhibit((HPM_NUM_CNTS+3)-1 downto 3) when ((CPU_EXTENSION_RISCV_U = true) and (CPU_EXTENSION_RISCV_Zihpm = true) and (HPM_NUM_CNTS > 0)) else (others => '0'); -- mcountinhibit
    rvvi_trace_csr_o(to_integer(unsigned(csr_mcountinhibit_c)))(XLEN-1 downto (HPM_NUM_CNTS+3)) <= (others => '0');
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmevent3_c ))) <= hpmevent_rd(03); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmevent4_c ))) <= hpmevent_rd(04); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmevent5_c ))) <= hpmevent_rd(05); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmevent6_c ))) <= hpmevent_rd(06); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmevent7_c ))) <= hpmevent_rd(07); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmevent8_c ))) <= hpmevent_rd(08); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmevent9_c ))) <= hpmevent_rd(09); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmevent10_c))) <= hpmevent_rd(10); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmevent11_c))) <= hpmevent_rd(11); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmevent12_c))) <= hpmevent_rd(12); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmevent13_c))) <= hpmevent_rd(13); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmevent14_c))) <= hpmevent_rd(14); 
    rvvi_trace_csr_o(to_integer(unsigned(csr_mhpmevent15_c))) <= hpmevent_rd(15); 
    -- Debug/Trace Registers (Shared with Debug Mode)
    rvvi_trace_csr_o(to_integer(unsigned(csr_tdata1_c)))   <= csr.tdata1_rd when (CPU_EXTENSION_RISCV_Sdtrig) else (others => '0'); -- tdata1  
    rvvi_trace_csr_o(to_integer(unsigned(csr_tdata2_c)))   <= csr.tdata2 when (CPU_EXTENSION_RISCV_Sdtrig) else (others => '0'); -- tdata2  
    rvvi_trace_csr_o(to_integer(unsigned(csr_tinfo_c)) )   <= x"00000004" when (CPU_EXTENSION_RISCV_Sdtrig) else (others => '0'); -- tinfo   
    -- Debug Mode Registers
    rvvi_trace_csr_o(to_integer(unsigned(csr_dcsr_c))     )   <= csr.dcsr_rd   when (CPU_EXTENSION_RISCV_Sdext) else (others => '0'); -- dcsr     
    rvvi_trace_csr_o(to_integer(unsigned(csr_dpc_c))      )   <= csr.dpc       when (CPU_EXTENSION_RISCV_Sdext) else (others => '0'); -- dpc     
    rvvi_trace_csr_o(to_integer(unsigned(csr_dscratch0_c)))   <= csr.dscratch0 when (CPU_EXTENSION_RISCV_Sdext) else (others => '0'); -- dscratch0     

  end process;
  rvvi_trace_lrsc_cancel_o <= '0'; -- Core doesn't support internal cancellation of instructions

end neorv32_rvvi_trace_rtl;