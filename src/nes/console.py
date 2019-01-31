from .apu import APU
from .cpu import CPU
from .ppu import PPU
from .ram import RAM
from .memory import Memory
from .controller import Controller

class Console(object):
    def __init__(self):
        self.apu = APU(self)
        self.cpu = CPU(self)
        self.ppu = PPU(self)
        self.ram = RAM(self)
        self.mem = Memory(self)
        self.controller_1 = Controller(self)
        self.controller_2 = Controller(self)