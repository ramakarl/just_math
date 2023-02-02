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

for _line in sys.stdin:
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

  # only consider tile information
  #
  if _line[0] != ' ': continue


  #print(state, cell_str, line[0:4], line)

  if ((cell_str == "[0,0,0](0):") and
      (line[0:4] != "r003")):
    print("FAIL: line:", line, "... expected only r003")
    sys.exit(-1)

  if ((cell_str == "[1,0,0](1):") and
      (line[0:4] != "r002")):
    print("FAIL: line:", line, "... expected only r002")
    sys.exit(-1)

  if ((cell_str == "[0,1,0](2):") and
      (line[0:4] != "r000")):
    print("FAIL: line:", line, "... expected only r000")
    sys.exit(-1)

  if ((cell_str == "[1,1,0](3):") and
      (line[0:4] != "r001")):
    print("FAIL: line:", line, "... expected only r001")
    sys.exit(-1)




if verbosity > 0:
  print("ok")
sys.exit(0)




  

  
