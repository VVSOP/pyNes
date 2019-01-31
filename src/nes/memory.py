from .console import Console
import numpy as np

class Memory(object):
    def __init__(self, console: Console):
        self.console = console

    def memory_read_byte(self, address: np.uint16) -> np.uint8:
        if address < 0x2000:
            return self.console.ram.read_memory(address % 0x0800)
        elif address < 0x4000:
            return self.console.ppu.read_memory(0x2000 + address % 8)
        elif address < 0x4014:
            return self.console.apu.read_memory(address)
        elif address == 0x4014:
            return self.console.ppu.read_memory(address)
        elif address == 0x4015:
            return self.console.apu.read_memory(address)
        elif address == 0x4016:
            return self.console.controller_1.read_memory(address)
        elif address == 0x4017:
            return self.console.controller_2.read_memory(address)
        else:
            #TODO
            pass

    def memory_read_word(self, address: np.uint16):
        return self.memory_read_byte(address) | (self.memory_read_byte(address+1) << 8)


    def memory_write(self, address: np.uint16, value: np.uint8):
        if address < 0x2000:
            return self.console.ram.write_memory(address % 0x0800, value)
        elif address < 0x4000:
            return self.console.ppu.write_memory(0x2000 + address % 8, value)
        elif address < 0x4014:
            return self.console.apu.write_memory(address, value)
        elif address == 0x4014:
            return self.console.ppu.write_memory(address, value)
        elif address == 0x4015:
            return self.console.apu.write_memory(address, value)
        elif address == 0x4016:
            return self.console.controller_1.write_memory(value)
        elif address == 0x4017:
            return self.console.controller_2.write_memory(value)
        else:
            #TODO
            pass




