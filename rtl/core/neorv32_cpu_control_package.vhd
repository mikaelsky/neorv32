-- #################################################################################################
-- # << NEORV32 - CPU Controller Package >>                                                        #
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

package neorv32_cpu_control_package is

-- ****************************************************************************************************************************
-- Types, Structs and Constants
-- ****************************************************************************************************************************

  -- instruction issue engine --
  type issue_engine_t is record
    align     : std_ulogic;
    align_set : std_ulogic;
    align_clr : std_ulogic;
    ci_i16    : std_ulogic_vector(15 downto 0);
    ci_i32    : std_ulogic_vector(31 downto 0);
    data      : std_ulogic_vector((2+32)-1 downto 0); -- 2-bit status + 32-bit instruction
    valid     : std_ulogic_vector(1 downto 0); -- data word is valid
    ack       : std_ulogic;
  end record;

  -- instruction execution engine --
  -- make sure reset state is the first item in the list (discussion #415)
  type execute_engine_state_t is (DISPATCH, TRAP_ENTER, TRAP_EXIT, RESTART, FENCE, SLEEP,
                                  EXECUTE, ALU_WAIT, BRANCH, BRANCHED, SYSTEM, MEM_REQ, MEM_WAIT);
  type execute_engine_t is record
    state        : execute_engine_state_t;
    state_nxt    : execute_engine_state_t;
    state_prev   : execute_engine_state_t;
    state_prev2  : execute_engine_state_t;
    ir           : std_ulogic_vector(31 downto 0);
    ir_nxt       : std_ulogic_vector(31 downto 0);
    is_ci        : std_ulogic; -- current instruction is de-compressed instruction
    is_ci_nxt    : std_ulogic;
    branch_taken : std_ulogic; -- branch condition fulfilled
    pc           : std_ulogic_vector(XLEN-1 downto 0); -- actual PC, corresponding to current executed instruction
    pc_we        : std_ulogic; -- PC update enabled
    next_pc      : std_ulogic_vector(XLEN-1 downto 0); -- next PC, corresponding to next instruction to be executed
    next_pc_inc  : std_ulogic_vector(XLEN-1 downto 0); -- increment to get next PC
    link_pc      : std_ulogic_vector(XLEN-1 downto 0); -- next PC for linking (return address)
  end record;

  -- trap controller --
  type trap_ctrl_t is record
    exc_buf     : std_ulogic_vector(exc_width_c-1 downto 0); -- synchronous exception buffer (one bit per exception)
    exc_fire    : std_ulogic; -- set if there is a valid source in the exception buffer
    irq_pnd     : std_ulogic_vector(irq_width_c-1 downto 0); -- pending interrupt
    irq_buf     : std_ulogic_vector(irq_width_c-1 downto 0); -- asynchronous exception/interrupt buffer (one bit per interrupt source)
    irq_fire    : std_ulogic; -- set if an interrupt is actually kicking in
    cause       : std_ulogic_vector(6 downto 0); -- trap ID for mcause CSR & debug-mode entry identifier
    epc         : std_ulogic_vector(XLEN-1 downto 0); -- exception program counter
    --
    env_pending : std_ulogic; -- start of trap environment if pending
    env_enter   : std_ulogic; -- enter trap environment
    env_exit    : std_ulogic; -- leave trap environment
    wakeup      : std_ulogic; -- wakeup from sleep due to an enabled pending IRQ
    --
    instr_be    : std_ulogic; -- instruction fetch bus error
    instr_ma    : std_ulogic; -- instruction fetch misaligned address
    instr_il    : std_ulogic; -- illegal instruction
    ecall       : std_ulogic; -- ecall instruction
    ebreak      : std_ulogic; -- ebreak instruction
    hwtrig      : std_ulogic; -- hardware trigger module
  end record;

  -- control and status registers (CSRs) --
  type csr_t is record
    addr             : std_ulogic_vector(11 downto 0); -- csr address
    raddr            : std_ulogic_vector(11 downto 0); -- simplified csr read address
    we, we_nxt       : std_ulogic; -- csr write enable
    re, re_nxt       : std_ulogic; -- csr read enable
    wdata, rdata     : std_ulogic_vector(XLEN-1 downto 0); -- csr write/read data
    --
    mstatus_mie      : std_ulogic; -- machine-mode IRQ enable
    mstatus_mpie     : std_ulogic; -- previous machine-mode IRQ enable
    mstatus_mpp      : std_ulogic; -- machine previous privilege mode
    mstatus_mprv     : std_ulogic; -- effective privilege level for load/stores
    mstatus_tw       : std_ulogic; -- do not allow user mode to execute WFI instruction when set
    --
    mie_msi          : std_ulogic; -- machine software interrupt enable
    mie_mei          : std_ulogic; -- machine external interrupt enable
    mie_mti          : std_ulogic; -- machine timer interrupt enable
    mie_firq         : std_ulogic_vector(15 downto 0); -- fast interrupt enable
    mip_firq_nclr    : std_ulogic_vector(15 downto 0); -- clear pending FIRQ (active-low)
    --
    privilege        : std_ulogic; -- current privilege mode
    privilege_eff    : std_ulogic; -- current *effective* privilege mode
    --
    mepc             : std_ulogic_vector(XLEN-1 downto 0); -- machine exception PC
    mcause           : std_ulogic_vector(5 downto 0); -- machine trap cause
    mtvec            : std_ulogic_vector(XLEN-1 downto 0); -- machine trap-handler base address
    mtval            : std_ulogic_vector(XLEN-1 downto 0); -- machine bad address or instruction
    mtinst           : std_ulogic_vector(XLEN-1 downto 0); -- machine trap instruction
    mscratch         : std_ulogic_vector(XLEN-1 downto 0); -- machine scratch register
    mcounteren       : std_ulogic; -- machine counter access enable (from user-mode) for ALL counters
    mcountinhibit    : std_ulogic_vector(15 downto 0); -- inhibit counter auto-increment
    mcyclecfg_minh   : std_ulogic; -- inhibit cycle counter when in machine-mode
    mcyclecfg_uinh   : std_ulogic; -- inhibit cycle counter when in user-mode
    minstretcfg_minh : std_ulogic; -- inhibit instret counter when in machine-mode
    minstretcfg_uinh : std_ulogic; -- inhibit instret counter when in user-mode
    --
    dcsr_ebreakm     : std_ulogic; -- behavior of ebreak instruction in m-mode
    dcsr_ebreaku     : std_ulogic; -- behavior of ebreak instruction in u-mode
    dcsr_step        : std_ulogic; -- single-step mode
    dcsr_prv         : std_ulogic; -- current privilege level when entering debug mode
    dcsr_cause       : std_ulogic_vector(2 downto 0); -- why was debug mode entered
    dcsr_rd          : std_ulogic_vector(XLEN-1 downto 0); -- debug mode control and status register
    dpc              : std_ulogic_vector(XLEN-1 downto 0); -- mode program counter
    dscratch0        : std_ulogic_vector(XLEN-1 downto 0); -- debug mode scratch register 0
    --
    tdata1_hit_clr   : std_ulogic; -- set to manually clear mcontrol6.hit0
    tdata1_execute   : std_ulogic; -- enable instruction address match trigger
    tdata1_action    : std_ulogic; -- enter debug mode / ebreak exception when trigger fires
    tdata1_dmode     : std_ulogic; -- set to ignore tdata* CSR access from machine-mode
    tdata1_rd        : std_ulogic_vector(XLEN-1 downto 0); -- trigger register read-back
    tdata2           : std_ulogic_vector(XLEN-1 downto 0); -- address-match register
  end record;

  type hpmevent_rd_t  is array (3 to 15) of std_ulogic_vector(XLEN-1 downto 0);

  type cnt_dat_t is array (natural range <>) of std_ulogic_vector(XLEN-1 downto 0);


  -- debug mode controller --
  type debug_ctrl_t is record
    running      : std_ulogic; -- CPU is in debug mode
    trig_hw      : std_ulogic; -- hardware trigger
    trig_break   : std_ulogic; -- ebreak instruction trigger
    trig_halt    : std_ulogic; -- external request trigger
    trig_step    : std_ulogic; -- single-stepping mode trigger
    ext_halt_req : std_ulogic; -- external halt request buffer
  end record;


end neorv32_cpu_control_package;
