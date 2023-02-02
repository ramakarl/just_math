#!/usr/bin/python3
#

import sys
import re

verbosity = 0
state = "init"

tile_map = {}
count = 0

cell_str = ""

expect_a = 0.6
expect_b = 0.4
expect_c = 0.2
_eps = (1.0/1024.0)

valid_cell_tile = {
  "[0,0,0](0):":"r003(10)",
  "[1,0,0](1):":"T002(24)",
  "[2,0,0](2):":"r002(9)",
  "[0,1,0](3):":"T003(25)",
  "[1,1,0](4):":"T000(22)",
  "[2,1,0](5):":"T001(23)",
  "[0,2,0](6):":"r000(7)",
  "[1,2,0](7):":"|001(2)",
  "[2,2,0](8):":"r001(8)"
}

line_count = 0

for _line in sys.stdin:
  line_count+=1
  line = _line.strip()
  if line[0:5] == "got:":
    val = line.split(" ")[1]
    if (val != "0"):
      print("FAIL:", line, "expected 'got: 0'")
      sys.exit(-1)


  if len(line)==0: continue
  if line[0] == '[':
    cell_str = line
    continue

  if cell_str == "": continue

  # only consider tile information
  #
  if _line[0] != ' ': continue


  #print(state, cell_str, line[0:4], line)

  if not (cell_str in valid_cell_tile):
    print("FAIL: could not find", cell_str, "in admissible list")
    sys.exit(-1)

  v = valid_cell_tile[cell_str]
  if v[0:4] != line[0:4]:
    print("FAIL: line:", line, "... expected only", v, "(line", line_count, ")")
    sys.exit(-1)


if verbosity > 0:
  print("ok")
sys.exit(0)




  

  
