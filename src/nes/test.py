import numpy as np
# def add(x,y):
#     return x + y
#
# def sub (x, y):
#     print ("sub")
#
# dict = {1: add, 2: sub}
#
# print (dict[1](1,2))

# KB = 1024
# memory = [0]*KB*2
# print(len(memory))
#
# t = np.zeros((0x0800,), dtype=np.uint8)
#
# print(len(t))

def a():
    print("a")

def b():
    print("b")

# l = [a,b]

# print("a" == l[0].__name__)

def c(value: bool):
    if value:
        print("true")
    else:
        print("false")

print(c(0))
print(c(1))

CFlag: np.uint8 = 0x01  # carry flag
ZFlag: np.uint8 = 0x02  # zero flag
IFlag: np.uint8 = 0x04  # interrupt control flag
DFlag: np.uint8 = 0x08  # decimal mode flag
BFlag: np.uint8 = 0x10  # break flag
UFlag: np.uint8 = 0x20  # unused flag
OFlag: np.uint8 = 0x40  # overflow flag
NFlag: np.uint8 = 0x80  # negative bit / sign bit


def set_flags(flags: np.uint8)->np.uint8:
    p = np.uint8(0)
    p |= flags
    return p
#
# t = 1
#
# print(10 + (1 if t else 0))
a = 1
b = 1
c = 1

result = (a^b)&0x80 == 0 & (a^c)&0x80 != 0
print(result)

result2 = ~(a ^ b) & (a ^ c) & 0x80
print(result2)

result3 = (b ^ c) & (a ^ c) & 0x80
print(result3)