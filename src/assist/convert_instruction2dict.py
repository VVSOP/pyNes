import csv

name_reader = csv.reader(open('./instructionName.csv'))
modes_reader = csv.reader(open('./instructionModes.csv'))
cycles_reader = csv.reader(open('./instructionCycles.csv'))
pageCycles_reader = csv.reader(open('./instructionPageCycles.csv'))

# generate name dict
inst_name = []
for row in name_reader:
    for i in row[:]:
        if i != "":
            inst_name.append("cpu_" + i.replace(" ", "").lower())


f = open("name_dict.txt",'w')
for i in range(256):
    f.write(hex(i)+": "+inst_name[i]+","+'\n')
f.close()
#


# generate mode dict
inst_mode = []

for row in modes_reader:
    for i in row[:]:

        if i != "":
            i = i.replace(" ", "")

            if i == "1":
                inst_mode.append("mode_absolute")
            elif i == "2":
                inst_mode.append("mode_absolute_x")
            elif i == "3":
                inst_mode.append("mode_absolute_y")
            elif i == "4":
                inst_mode.append("mode_accmulator")
            elif i == "5":
                inst_mode.append("mode_immediate")
            elif i == "6":
                inst_mode.append("mode_implied")
            elif i == "7":
                inst_mode.append("mode_indirect_x")
            elif i == "8":
                inst_mode.append("mode_indirect")
            elif i == "9":
                inst_mode.append("mode_indirect_y")
            elif i == "10":
                inst_mode.append("mode_relative")
            elif i == "11":
                inst_mode.append("mode_zeropage")
            elif i == "12":
                inst_mode.append("mode_zeropage_x")
            elif i == "13":
                inst_mode.append("mode_zeropage_y")


f = open("mode_dict.txt",'w')
for i in range(256):
    f.write(hex(i)+": "+inst_mode[i]+ "," +'\n')
f.close()
#



# generate cycles dict
inst_cycles = []
for row in cycles_reader:
    for i in row[:]:
        if i != "":
            inst_cycles.append(i.replace(" ", "").lower())


f = open("cycles_dict.txt",'w')
for i in range(256):
    f.write(hex(i)+": "+inst_cycles[i]+ "," +'\n')
f.close()
#


# generate page cycles dict
inst_pagecycles = []
for row in pageCycles_reader:
    for i in row[:]:
        if i != "":
            inst_pagecycles.append(i.replace(" ", "").lower())


f = open("pagecycles_dict.txt",'w')
for i in range(256):
    f.write(hex(i)+": "+inst_pagecycles[i]+ "," +'\n')
f.close()
#
























