from .console import Console
import numpy as np

class RAM(object):
    def __init__(self, console: Console):
        self.memory = np.zeros((2 * 1024,), dtype=np.uint8)

    def read_memory(self, address: np.uint16):
        return self.memory[address % 0x0800]