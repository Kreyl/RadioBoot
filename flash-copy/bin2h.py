import sys
import os
import struct
import binascii
import string

usage = "usage: bin2h.py <file_in> <file_out>"
ITEMS_ON_LINE = 16

def write_cr(f):
    f.write("/*\n")
    f.write("*      Author: Roma Jam\n")
    f.write("*/\n\n")
 
if len(sys.argv) < 3:
    print usage
    exit()
 
f = open(sys.argv[1], "rb")
h = binascii.hexlify(f.read())
f.close()

hdr = open(sys.argv[2] + ".h", "w+")
write_cr(hdr)
hdr.write("#ifndef " + sys.argv[2].upper() + "_H\n")
hdr.write("#define " + sys.argv[2].upper() + "_H\n\n")
hdr.write("#include <stdint.h>\n\n")
hdr.write("#define FLASH_COPY_FN_SIZE                   " + str(len(h) / 2) + "\n\n")
hdr.write("extern const uint8_t __FLASH_COPY_FN[FLASH_COPY_FN_SIZE];\n\n")
hdr.write("typedef int (*FLASH_COPY_FN_TYPE)(uint32_t, uint32_t, unsigned int);\n\n")
hdr.write("#endif // " + sys.argv[2].upper() + "_H\n")
hdr.close

src = open(sys.argv[2] + ".c", "w+")
write_cr(src)

src.write("#include \"" + sys.argv[2] + ".h\"\n\n")
src.write("const uint8_t __FLASH_COPY_FN[FLASH_COPY_FN_SIZE] = {\n")

for i in range(0, len(h) / 2):
    if (i % ITEMS_ON_LINE) == 0:
        src.write("    ")
    src.write("0x" + h[i * 2] + h[i * 2 + 1])
    if i < len(h) / 2 - 1:
        if (i % ITEMS_ON_LINE) == ITEMS_ON_LINE - 1:
            src.write(",\n")
        else:
            src.write(", ")
src.write("\n};\n")
src.close