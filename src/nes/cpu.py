import numpy as np
from .console import Console
from enum import Enum

class CPU(object):
    def __init__(self, console: Console):

        self.console = console

        # data registers
        self.a_reg: np.uint8 = None  # accumulator register
        self.x_reg: np.uint8 = None  # index register x
        self.y_reg: np.uint8 = None  # index register y

        self.sp: np.uint8 = None  # stack pointer
        self.pc: np.uint16 = None  # program counter
        self.p: np.uint8 = None  # status register

        # status registers
        self.CFlag: np.uint8 = 0x01  # carry flag
        self.ZFlag: np.uint8 = 0x02  # zero flag
        self.IFlag: np.uint8 = 0x04  # interrupt control flag
        self.DFlag: np.uint8 = 0x08  # decimal mode flag
        self.BFlag: np.uint8 = 0x10  # break flag
        self.UFlag: np.uint8 = 0x20  # unused flag
        self.OFlag: np.uint8 = 0x40  # overflow flag
        self.NFlag: np.uint8 = 0x80  # negative bit / sign bit

        # interrupt type
        self.interrupt = None

        # currect opcode add
        self.op_address: np.uint16 = None
        self.cycles: np.uint64 = None
        self.additional_cycles: np.uint64 = None
        self.op_value: np.uint8 = None
        self.op_code = None

        self.cpu_reset()

    class Interrupt(Enum):
        NMI = 0
        IRQ = 1

    def cpu_reset(self):
        self.pc = self.console.mem.memory_read_word(0xFFFC)
        self.sp = 0xFD
        self.p = 0x24

        self.a_reg = 0
        self.x_reg = 0
        self.y_reg = 0

        self.interrupt = None

        self.op_address = 0
        self.cycles = 0
        self.additional_cycles = 0
        self.op_value = 0
        self.op_code = 0

    # addressing modes
    def mode_absolute(self):
        self.op_address = self.console.mem.memory_read_word(self.pc)
        self.op_value = self.console.mem.memory_read_byte(self.op_address)
        self.pc = self.pc + 2
        self.additional_cycles = 0

    def mode_absolute_x(self):
        self.op_address = self.console.mem.memory_read_word(self.pc)
        if (self.op_address & 0xFF00) != (self.op_address + self.x_reg) & 0xFF00:
            self.additional_cycles = 1
        else:
            self.additional_cycles = 0
        self.op_address += self.x_reg
        self.op_value = self.console.mem.memory_read_byte(self.op_address)
        self.pc += 2

    def mode_absolute_y(self):
        self.op_address = self.console.mem.memory_read_word(self.pc)
        if (self.op_address & 0xFF00) != (self.op_address + self.y_reg) & 0xFF00:
            self.additional_cycles = 1
        else:
            self.additional_cycles = 0
        self.op_address += self.y_reg
        self.op_value = self.console.mem.memory_read_byte(self.op_address)
        self.pc += 2

    def mode_accmulator(self):
        self. additional_cycles = 0

    def mode_immediate(self):
        self.op_address = self.pc
        self.op_value = self.console.mem.memory_read_byte(self.pc)
        self.pc += 1
        self.additional_cycles = 0

    def mode_implied(self):
        self.additional_cycles = 0

    def mode_indirect(self):
        addr : np.uint8 = self.console.mem.memory_read_word(self.pc)
        b = (addr & 0xFF00) | (addr + 1)
        lo = self.console.mem.memory_read(addr)
        hi = self.console.mem.memory_read(b)
        self.op_address = hi << 8 | lo
        self.pc += 2
        self.op_value = self.console.mem.memory_read_byte(self.op_address)
        self.additional_cycles = 0

    def mode_indirect_x(self):
        addr = self.console.mem.memory_read_byte(self.pc) + self.x_reg
        b =(addr & 0xFF00) | (addr + 1)
        lo = self.console.mem.memory_read_byte(addr)
        hi = self.console.mem.memory_read_byte(b)
        self.op_address = hi << 8 | lo
        self.op_value = self.console.mem.memory_read_byte(self.op_address)
        self.pc += 1
        self.additional_cycles = 0

    def mode_indirect_y(self):
        addr = self.console.mem.memory_read_byte(self.pc)
        b =(addr & 0xFF00) | (addr + 1)
        lo = self.console.mem.memory_read_byte(addr)
        hi = self.console.mem.memory_read_byte(b)
        self.op_address = hi << 8 | lo
        self.op_address += self.y_reg
        self.op_value = self.console.mem.memory_read_byte(self.op_address)
        self.pc += 1
        if ((self.op_address - self.y_reg) & 0xFF00) != (self.op_address & 0xFF00):
            self.additional_cycles = 1
        else:
            self.additional_cycles = 0

    def mode_relative(self):
        offest = self.console.mem.memory_read_byte(self.pc)
        if offest < 0x80:
            self.op_address = self.pc + 1 + offest
        else:
            self.op_address = self.pc + 1 + offest - 0x100
        self.pc += 1
        self.op_value = self.console.mem.memory_read_byte(self.op_address)
        if (self.op_address >> 8) != (self.pc >> 8):
            self.additional_cycles = 1
        else:
            self.additional_cycles = 0

    def mode_zeropage(self):
        self.op_address = self.console.mem.memory_read_byte(self.pc)
        self.op_value = self.console.mem.memory_read_byte(self.op_address)
        self.pc += 1
        self.additional_cycles = 0

    def mode_zeropage_x(self):
        self.op_address = (self.console.mem.memory_read_word(self.pc) + self.x_reg) & 0xff
        self.op_value = self.console.mem.memory_read_byte(self.op_address)
        self.pc += 1
        self.additional_cycles = 0

    def mode_zeropage_y(self):
        self.op_address = (self.console.mem.memory_read_word(self.pc) + self.y_reg) & 0xff
        self.op_value = self.console.mem.memory_read_byte(self.op_address)
        self.pc += 1
        self.additional_cycles = 0

    # flag operations
    def clear_flags(self, flags: np.uint8):
        self.p &= ~flags

    def set_flags(self, flags: np.uint8):
        self.p |= flags

    # write flags
    # if value == True, set flag to 1
    # if value == False, set flag to 0

    def write_c_flag(self, value: bool):
        if value:
            self.set_flags(self.CFlag)
        else:
            self.clear_flags(self.CFlag)

    def write_z_flag(self, value: bool):
        if value:
            self.set_flags(self.ZFlag)
        else:
            self.clear_flags(self.ZFlag)

    def write_i_flag(self, value: bool):
        if value:
            self.set_flags(self.IFlag)
        else:
            self.clear_flags(self.IFlag)

    def write_d_flag(self, value: bool):
        if value:
            self.set_flags(self.DFlag)
        else:
            self.clear_flags(self.DFlag)

    def write_b_flag(self, value: bool):
        if value:
            self.set_flags(self.BFlag)
        else:
            self.clear_flags(self.BFlag)

    def write_u_flag(self, value: bool):
        if value:
            self.set_flags(self.UFlag)
        else:
            self.clear_flags(self.UFlag)

    def write_o_flag(self, value: bool):
        if value:
            self.set_flags(self.OFlag)
        else:
            self.clear_flags(self.OFlag)

    def write_n_flag(self, value: bool):
        if value:
            self.set_flags(self.NFlag)
        else:
            self.clear_flags(self.NFlag)

    def check_flag(self, flag: np.uint8)->np.uint8:
        return (self.p & flag) == flag

    def set_zn_flags(self, value: np.uint8):
        self.write_z_flag((value >> 7) & 1)
        self.write_n_flag(value == 0)

    # Stack operations
    def push_byte(self, value:np.uint8):
        self.console.mem.memory_write(self.sp + 0x100, value)
        self.sp -= 1

    def pull_byte(self)-> np.uint8 :
        self.sp += 1
        return self.console.mem.memory_read(self.sp + 0x100)

    def push_word(self, value:np.uint16):
        hi = np.uint8(value >> 8)
        lo = np.uint8(value & 0xFF)
        self.push_byte(hi)
        self.push_byte(lo)

    def pull_word(self)-> np.uint16:
        lo = self.pull_byte()
        hi = self.pull_byte()
        return hi << 8 | lo

    def trigger_nmi(self):
        self.interrupt = self.Interrupt.NMI

    def trigger_irq(self):
        if self.check_flag(self.IFlag):
            self.interrupt = self.Interrupt.IRQ

    # instructions

    # ALU
    def cpu_ora(self):
        self.a_reg |= self.op_value
        self.set_zn_flags(self.a_reg)

    def cpu_and(self):
        self.a_reg &= self.op_value
        self.set_zn_flags(self.a_reg)

    def cpu_eor(self):
        self.a_reg ^= self.op_value
        self.set_zn_flags(self.a_reg)

    def cpu_asl(self):
        if self.addressing_mode_list[self.op_value].__name__ == "mode_accmulator":
            self.write_c_flag((self.a_reg >> 7) & 1)
            self.a_reg <<= 1
            self.set_zn_flags(self.a_reg)
        else:
            self.write_c_flag((self.op_value >> 7) & 1)
            self.op_value <<= 1
            self.console.mem.memory_write(self.op_address,self.op_value)
            self.set_zn_flags(self.op_value)

    def cpu_rol(self):
        if self.addressing_mode_list[self.op_value].__name__ == "mode_accmulator":
            c = self.CFlag
            self.write_c_flag((self.a_reg >> 7) & 1)
            self.a_reg = (self.a_reg << 1) | c
            self.set_zn_flags(self.a_reg)
        else:
            c = self.CFlag
            self.write_c_flag((self.op_value >> 7) & 1)
            self.op_value = ((self.op_value << 1) | c)
            self.console.mem.memory_write(self.op_address, self.op_value)
            self.set_zn_flags(self.op_value)

    def cpu_ror(self):
        if self.addressing_mode_list[self.op_value].__name__ == "mode_accmulator":
            c = self.CFlag
            self.write_c_flag(self.a_reg & 1)
            self.a_reg = (self.a_reg >> 1) | (c << 7)
            self.set_zn_flags(self.a_reg)
        else:
            c = self.CFlag
            self.write_c_flag(self.op_value & 1)
            self.op_value = (self.op_value >> 1) | (c << 7)
            self.console.mem.memory_write(self.op_address, self.op_value)
            self.set_zn_flags(self.op_value)

    def cpu_lsr(self):
        if self.addressing_mode_list[self.op_value].__name__ == "mode_accmulator":
            self.write_c_flag(self.a_reg & 1)
            self.a_reg >>= 1
            self.set_zn_flags(self.a_reg)
        else:
            self.write_c_flag(self.op_value & 1)
            self.op_value >>= 1
            self.console.mem.memory_write(self.op_address, self.op_value)
            self.set_zn_flags(self.op_value)

    def cpu_adc(self):
        result = self.op_value + self.a_reg + (1 if (self.p & self.CFlag) else 0)
        self.write_c_flag(result > 0xFF)
        self.write_o_flag((self.op_value ^ result) & (self.a_reg ^ result) & 0x80)
        self.a_reg = result
        self.set_zn_flags(self.a_reg)

    def cpu_sbc(self):
        result = self.a_reg - self.op_value - (1 - (1 if (self.p & self.CFlag) else 0))
        self.write_c_flag(result >= 0)
        self.write_o_flag((self.a_reg ^ self.op_value) & (self.a_reg ^ result) & 0x80)
        self.a_reg = result
        self.set_zn_flags(self.a_reg)

    # Branching
    def cpu_bmi(self):
        if self.p & self.NFlag:
            self.pc = self.op_address

    def cpu_bcs(self):
        if self.p & self.CFlag:
            self.pc = self.op_address

    def cpu_beq(self):
        if self.p & self.ZFlag:
            self.pc = self.op_address

    def cpu_bvs(self):
        if self.p & self.OFlag:
            self.pc = self.op_address

    def cpu_bpl(self):
        if not (self.p & self.NFlag):
            self.pc = self.op_address

    def cpu_bcc(self):
        if not(self.p & self.CFlag):
            self.pc = self.op_address

    def cpu_bne(self):
        if not(self.p & self.ZFlag):
            self.pc = self.op_address

    def cpu_bvc(self):
        if not(self.p & self.OFlag):
            self.pc = self.op_address

    #  Comapre

    def cpu_bit(self):
        self.write_o_flag(self.op_value & 0x40)
        self.write_n_flag(self.op_value & 0x80)
        self.write_z_flag(not (self.op_value & self.a_reg))

    def cpu_cmp(self):
        self.write_c_flag((self.a_reg - self.op_value) >= 0)
        self.set_zn_flags(self.a_reg - self.op_value)

    def cpu_cpx(self):
        self.write_c_flag((self.x_reg - self.op_value) >= 0)
        self.set_zn_flags(self.x_reg - self.op_value)

    def cpu_cpy(self):
        self.write_c_flag((self.y_reg - self.op_value) >= 0)
        self.set_zn_flags(self.y_reg - self.op_value)

    #  Flag Operations

    def cpu_clc(self):
        self.write_c_flag(False)

    def cpu_cli(self):
        self.write_i_flag(False)

    def cpu_cld(self):
        self.write_d_flag(False)

    def cpu_clv(self):
        self.write_o_flag(False)

    def cpu_sec(self):
        self.write_c_flag(True)

    def cpu_sei(self):
        self.write_i_flag(True)

    def cpu_sed(self):
        self.write_d_flag(True)

    # Inc and Dec

    def cpu_dec(self):
        self.console.mem.memory_write(self.op_address, self.op_value - 1)
        self.set_zn_flags(self.op_value - 1)

    def cpu_dex(self):
        self.x_reg -= 1
        self.set_zn_flags(self.x_reg)

    def cpu_dey(self):
        self.y_reg -= 1
        self.set_zn_flags(self.y_reg)

    def cpu_inc(self):
        self.console.mem.memory_write(self.op_address, self.op_value + 1)
        self.set_zn_flags(self.op_value + 1)

    def cpu_inx(self):
        self.x_reg += 1
        self.set_zn_flags(self.x_reg)

    def cpu_iny(self):
        self.y_reg += 1
        self.set_zn_flags(self.y_reg)

    #  Load and Store

    def cpu_lda(self):
        self.a_reg = self.op_value
        self.set_zn_flags(self.a_reg)

    def cpu_ldx(self):
        self.x_reg = self.op_value
        self.set_zn_flags(self.x_reg)

    def cpu_ldy(self):
        self.y_reg = self.op_value
        self.set_zn_flags(self.y_reg)

    def cpu_sta(self):
        self.console.mem.memory_write(self.op_address, self.a_reg)

    def cpu_stx(self):
        self.console.mem.memory_write(self.op_address, self.x_reg)

    def cpu_sty(self):
        self.console.mem.memory_write(self.op_address, self.y_reg)

    # NOP - No
    def cpu_nop(self):
        pass

    # Stack and Jump

    def cpu_pha(self):
        self.push_byte(self.a_reg)

    def cpu_php(self):
        self.push_byte(self.p | 0x30)

    def cpu_pla(self):
        self.a_reg = self.pull_byte()
        self.set_zn_flags(self.a_reg)

    def cpu_plp(self):
        self.p = self.pull_byte() & 0xEF | 0x20

    def cpu_rts(self):
        self.pc = self.pull_word() + 1

    def cpu_rti(self):
        self.p = self.pull_byte() & 0xEF | 0x20
        self.pc = self.pull_word()

    def cpu_jmp(self):
        self.pc = self.op_address

    def cpu_jsr(self):
        self.push_word(self.pc - 1)
        self.pc = self.op_address

    # def cpu_brk(self):
    #     self.push_word(self.pc - 1)
    #     self.push_byte(self.p)
    #     self.p = self.p | self.UFlag | self.BFlag
    #     self.pc = self.console.mem.memory_read_word(0xfffe)

    def cpu_brk(self):
        pass

    # Transfer Operations
    def cpu_tax(self):
        self.x_reg = self.a_reg
        self.set_zn_flags(self.x_reg)

    def cpu_tay(self):
        self.y_reg = self.a_reg
        self.set_zn_flags(self.y_reg)

    def cpu_txa(self):
        self.a_reg = self.x_reg
        self.set_zn_flags(self.a_reg)

    def cpu_tya(self):
        self.a_reg = self.y_reg
        self.set_zn_flags(self.a_reg)

    def cpu_tsx(self):
        self.x_reg = self.sp
        self.set_zn_flags(self.x_reg)

    def cpu_txs(self):
        self.sp = self.x_reg

    # Undocumemted
    def cpu_ahx(self):
        pass

    def cpu_alr(self):
        pass

    def cpu_anc(self):
        pass

    def cpu_arr(self):
        pass

    def cpu_axs(self):
        pass

    def cpu_dcp(self):
        pass

    def cpu_isc(self):
        pass

    def cpu_kil(self):
        pass

    def cpu_las(self):
        pass

    def cpu_lax(self):
        pass

    def cpu_rla(self):
        pass

    def cpu_rra(self):
        pass

    def cpu_sax(self):
        pass

    def cpu_shx(self):
        pass

    def cpu_shy(self):
        pass

    def cpu_slo(self):
        pass

    def cpu_sre(self):
        pass

    def cpu_tas(self):
        pass

    def cpu_xaa(self):
        pass

    #

    def cpu_step(self)-> int:
        """
         run a instruction
        :return: the cycles of this instruction
        """

        self.op_code = self.console.mem.memory_read_word(self.pc)
        self.pc += 1
        self.additional_cycles = 0

        self.addressing_mode_list[self.op_code]()  # addressing
        self.op_inst_list[self.op_code]()  # run instruction

        self.cycles = self.cycles + self.additional_cycles + self.op_inst_cycles_list[self.op_code]

        return self.additional_cycles + self.op_inst_cycles_list[self.op_code]

    # opcode addressing mode map size:256
    addressing_mode_list = [
        mode_implied,
        mode_indirect_x,
        mode_implied,
        mode_indirect_x,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_implied,
        mode_immediate,
        mode_accmulator,
        mode_immediate,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_relative,
        mode_indirect_y,
        mode_implied,
        mode_indirect_y,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_implied,
        mode_absolute_y,
        mode_implied,
        mode_absolute_y,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute,
        mode_indirect_x,
        mode_implied,
        mode_indirect_x,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_implied,
        mode_immediate,
        mode_accmulator,
        mode_immediate,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_relative,
        mode_indirect_y,
        mode_implied,
        mode_indirect_y,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_implied,
        mode_absolute_y,
        mode_implied,
        mode_absolute_y,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_x,
        mode_implied,
        mode_indirect_x,
        mode_implied,
        mode_indirect_x,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_implied,
        mode_immediate,
        mode_accmulator,
        mode_immediate,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_relative,
        mode_indirect_y,
        mode_implied,
        mode_indirect_y,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_implied,
        mode_absolute_y,
        mode_implied,
        mode_absolute_y,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_x,
        mode_implied,
        mode_indirect_x,
        mode_implied,
        mode_indirect_x,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_implied,
        mode_immediate,
        mode_accmulator,
        mode_immediate,
        mode_indirect,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_relative,
        mode_indirect_y,
        mode_implied,
        mode_indirect_y,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_implied,
        mode_absolute_y,
        mode_implied,
        mode_absolute_y,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_x,
        mode_immediate,
        mode_indirect_x,
        mode_immediate,
        mode_indirect_x,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_implied,
        mode_immediate,
        mode_implied,
        mode_immediate,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_relative,
        mode_indirect_y,
        mode_implied,
        mode_indirect_y,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_y,
        mode_zeropage_y,
        mode_implied,
        mode_absolute_y,
        mode_implied,
        mode_absolute_y,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_y,
        mode_absolute_y,
        mode_immediate,
        mode_indirect_x,
        mode_immediate,
        mode_indirect_x,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_implied,
        mode_immediate,
        mode_implied,
        mode_immediate,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_relative,
        mode_indirect_y,
        mode_implied,
        mode_indirect_y,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_y,
        mode_zeropage_y,
        mode_implied,
        mode_absolute_y,
        mode_implied,
        mode_absolute_y,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_y,
        mode_absolute_y,
        mode_immediate,
        mode_indirect_x,
        mode_immediate,
        mode_indirect_x,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_implied,
        mode_immediate,
        mode_implied,
        mode_immediate,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_relative,
        mode_indirect_y,
        mode_implied,
        mode_indirect_y,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_implied,
        mode_absolute_y,
        mode_implied,
        mode_absolute_y,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_x,
        mode_immediate,
        mode_indirect_x,
        mode_immediate,
        mode_indirect_x,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_zeropage,
        mode_implied,
        mode_immediate,
        mode_implied,
        mode_immediate,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_absolute,
        mode_relative,
        mode_indirect_y,
        mode_implied,
        mode_indirect_y,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_zeropage_x,
        mode_implied,
        mode_absolute_y,
        mode_implied,
        mode_absolute_y,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_x,
        mode_absolute_x
    ]

    '''
       # opcode instruction map 
       # '''
    op_inst_list = [
        cpu_brk,    #  0x00
        cpu_ora,   #  0x01
        cpu_kil,   #  0x02
        cpu_slo,   #  0x03
        cpu_nop,   #  0x04
        cpu_ora,   #  0x05
        cpu_asl,   #  0x06
        cpu_slo,   #  0x07
        cpu_php,   #  0x08
        cpu_ora,   #  0x09
        cpu_asl,   #  0x0a
        cpu_anc,   #  0x0b
        cpu_nop,   #  0x0c
        cpu_ora,   #  0x0d
        cpu_asl,   #  0x0e
        cpu_slo,   #  0x0f
        cpu_bpl,   #  0x10
        cpu_ora,   #  0x11
        cpu_kil,   #  0x12
        cpu_slo,   #  0x13
        cpu_nop,   #  0x14
        cpu_ora,   #  0x15
        cpu_asl,   #  0x16
        cpu_slo,   #  0x17
        cpu_clc,   #  0x18
        cpu_ora,   #  0x19
        cpu_nop,   #  0x1a
        cpu_slo,   #  0x1b
        cpu_nop,   #  0x1c
        cpu_ora,   #  0x1d
        cpu_asl,   #  0x1e
        cpu_slo,   #  0x1f
        cpu_jsr,   #  0x20
        cpu_and,   #  0x21
        cpu_kil,   #  0x22
        cpu_rla,   #  0x23
        cpu_bit,   #  0x24
        cpu_and,   #  0x25
        cpu_rol,   #  0x26
        cpu_rla,   #  0x27
        cpu_plp,   #  0x28
        cpu_and,   #  0x29
        cpu_rol,   #  0x2a
        cpu_anc,   #  0x2b
        cpu_bit,   #  0x2c
        cpu_and,   #  0x2d
        cpu_rol,   #  0x2e
        cpu_rla,   #  0x2f
        cpu_bmi,   #  0x30
        cpu_and,   #  0x31
        cpu_kil,   #  0x32
        cpu_rla,   #  0x33
        cpu_nop,   #  0x34
        cpu_and,   #  0x35
        cpu_rol,   #  0x36
        cpu_rla,   #  0x37
        cpu_sec,   #  0x38
        cpu_and,   #  0x39
        cpu_nop,   #  0x3a
        cpu_rla,   #  0x3b
        cpu_nop,   #  0x3c
        cpu_and,   #  0x3d
        cpu_rol,   #  0x3e
        cpu_rla,   #  0x3f
        cpu_rti,   #  0x40
        cpu_eor,   #  0x41
        cpu_kil,   #  0x42
        cpu_sre,   #  0x43
        cpu_nop,   #  0x44
        cpu_eor,   #  0x45
        cpu_lsr,   #  0x46
        cpu_sre,   #  0x47
        cpu_pha,   #  0x48
        cpu_eor,   #  0x49
        cpu_lsr,   #  0x4a
        cpu_alr,   #  0x4b
        cpu_jmp,   #  0x4c
        cpu_eor,   #  0x4d
        cpu_lsr,   #  0x4e
        cpu_sre,   #  0x4f
        cpu_bvc,   #  0x50
        cpu_eor,   #  0x51
        cpu_kil,   #  0x52
        cpu_sre,   #  0x53
        cpu_nop,   #  0x54
        cpu_eor,   #  0x55
        cpu_lsr,   #  0x56
        cpu_sre,   #  0x57
        cpu_cli,   #  0x58
        cpu_eor,   #  0x59
        cpu_nop,   #  0x5a
        cpu_sre,   #  0x5b
        cpu_nop,   #  0x5c
        cpu_eor,   #  0x5d
        cpu_lsr,   #  0x5e
        cpu_sre,   #  0x5f
        cpu_rts,   #  0x60
        cpu_adc,   #  0x61
        cpu_kil,   #  0x62
        cpu_rra,   #  0x63
        cpu_nop,   #  0x64
        cpu_adc,   #  0x65
        cpu_ror,   #  0x66
        cpu_rra,   #  0x67
        cpu_pla,   #  0x68
        cpu_adc,   #  0x69
        cpu_ror,   #  0x6a
        cpu_arr,   #  0x6b
        cpu_jmp,   #  0x6c
        cpu_adc,   #  0x6d
        cpu_ror,   #  0x6e
        cpu_rra,   #  0x6f
        cpu_bvs,   #  0x70
        cpu_adc,   #  0x71
        cpu_kil,   #  0x72
        cpu_rra,   #  0x73
        cpu_nop,   #  0x74
        cpu_adc,   #  0x75
        cpu_ror,   #  0x76
        cpu_rra,   #  0x77
        cpu_sei,   #  0x78
        cpu_adc,   #  0x79
        cpu_nop,   #  0x7a
        cpu_rra,   #  0x7b
        cpu_nop,   #  0x7c
        cpu_adc,   #  0x7d
        cpu_ror,   #  0x7e
        cpu_rra,   #  0x7f
        cpu_nop,   #  0x80
        cpu_sta,   #  0x81
        cpu_nop,   #  0x82
        cpu_sax,   #  0x83
        cpu_sty,   #  0x84
        cpu_sta,   #  0x85
        cpu_stx,   #  0x86
        cpu_sax,   #  0x87
        cpu_dey,   #  0x88
        cpu_nop,   #  0x89
        cpu_txa,   #  0x8a
        cpu_xaa,   #  0x8b
        cpu_sty,   #  0x8c
        cpu_sta,   #  0x8d
        cpu_stx,   #  0x8e
        cpu_sax,   #  0x8f
        cpu_bcc,   #  0x90
        cpu_sta,   #  0x91
        cpu_kil,   #  0x92
        cpu_ahx,   #  0x93
        cpu_sty,   #  0x94
        cpu_sta,   #  0x95
        cpu_stx,   #  0x96
        cpu_sax,   #  0x97
        cpu_tya,   #  0x98
        cpu_sta,   #  0x99
        cpu_txs,   #  0x9a
        cpu_tas,   #  0x9b
        cpu_shy,   #  0x9c
        cpu_sta,   #  0x9d
        cpu_shx,   #  0x9e
        cpu_ahx,   #  0x9f
        cpu_ldy,   #  0xa0
        cpu_lda,   #  0xa1
        cpu_ldx,   #  0xa2
        cpu_lax,   #  0xa3
        cpu_ldy,   #  0xa4
        cpu_lda,   #  0xa5
        cpu_ldx,   #  0xa6
        cpu_lax,   #  0xa7
        cpu_tay,   #  0xa8
        cpu_lda,   #  0xa9
        cpu_tax,   #  0xaa
        cpu_lax,   #  0xab
        cpu_ldy,   #  0xac
        cpu_lda,   #  0xad
        cpu_ldx,   #  0xae
        cpu_lax,   #  0xaf
        cpu_bcs,   #  0xb0
        cpu_lda,   #  0xb1
        cpu_kil,   #  0xb2
        cpu_lax,   #  0xb3
        cpu_ldy,   #  0xb4
        cpu_lda,   #  0xb5
        cpu_ldx,   #  0xb6
        cpu_lax,   #  0xb7
        cpu_clv,   #  0xb8
        cpu_lda,   #  0xb9
        cpu_tsx,   #  0xba
        cpu_las,   #  0xbb
        cpu_ldy,   #  0xbc
        cpu_lda,   #  0xbd
        cpu_ldx,   #  0xbe
        cpu_lax,   #  0xbf
        cpu_cpy,   #  0xc0
        cpu_cmp,   #  0xc1
        cpu_nop,   #  0xc2
        cpu_dcp,   #  0xc3
        cpu_cpy,   #  0xc4
        cpu_cmp,   #  0xc5
        cpu_dec,   #  0xc6
        cpu_dcp,   #  0xc7
        cpu_iny,   #  0xc8
        cpu_cmp,   #  0xc9
        cpu_dex,   #  0xca
        cpu_axs,   #  0xcb
        cpu_cpy,   #  0xcc
        cpu_cmp,   #  0xcd
        cpu_dec,   #  0xce
        cpu_dcp,   #  0xcf
        cpu_bne,   #  0xd0
        cpu_cmp,   #  0xd1
        cpu_kil,   #  0xd2
        cpu_dcp,   #  0xd3
        cpu_nop,   #  0xd4
        cpu_cmp,   #  0xd5
        cpu_dec,   #  0xd6
        cpu_dcp,   #  0xd7
        cpu_cld,   #  0xd8
        cpu_cmp,   #  0xd9
        cpu_nop,   #  0xda
        cpu_dcp,   #  0xdb
        cpu_nop,   #  0xdc
        cpu_cmp,   #  0xdd
        cpu_dec,   #  0xde
        cpu_dcp,   #  0xdf
        cpu_cpx,   #  0xe0
        cpu_sbc,   #  0xe1
        cpu_nop,   #  0xe2
        cpu_isc,   #  0xe3
        cpu_cpx,   #  0xe4
        cpu_sbc,   #  0xe5
        cpu_inc,   #  0xe6
        cpu_isc,   #  0xe7
        cpu_inx,   #  0xe8
        cpu_sbc,   #  0xe9
        cpu_nop,   #  0xea
        cpu_sbc,   #  0xeb
        cpu_cpx,   #  0xec
        cpu_sbc,   #  0xed
        cpu_inc,   #  0xee
        cpu_isc,   #  0xef
        cpu_beq,   #  0xf0
        cpu_sbc,   #  0xf1
        cpu_kil,   #  0xf2
        cpu_isc,   #  0xf3
        cpu_nop,   #  0xf4
        cpu_sbc,   #  0xf5
        cpu_inc,   #  0xf6
        cpu_isc,   #  0xf7
        cpu_sed,   #  0xf8
        cpu_sbc,   #  0xf9
        cpu_nop,   #  0xfa
        cpu_isc,   #  0xfb
        cpu_nop,   #  0xfc
        cpu_sbc,   #  0xfd
        cpu_inc,   #  0xfe
        cpu_isc    #  0xff
    ]
    
    op_inst_cycles_list = [
        7, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6,
        2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
        6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6,
        2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
        6, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6,
        2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
        6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6,
        2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
        2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
        2, 6, 2, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5,
        2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
        2, 5, 2, 5, 4, 4, 4, 4, 2, 4, 2, 4, 4, 4, 4, 4,
        2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
        2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
        2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
        2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7
    ]