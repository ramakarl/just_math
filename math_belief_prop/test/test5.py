#!/usr/bin/python3
#
# Here are all the possible configurations:
#
#  ___  #  ___  #  ___  #  ___     ___
# /   \ # /   \ # / | \ # / | \ # / | \
# |   | # |   | # | / | # | \ | # | | |
# |   | # |---| # |-  | # |  -| # |---|
# |   | # |   | # |   | # |   | # |   |
# \   / # \   / # \   / # \   / # \   /
#  ---  #  ---  #  ---  #  ---  #  ---
#
# from this, we expect:
#
# 0,1,0: 2/5 |000, 3/5 T003
# 2,1,0: 2/5 |000, 3/5 T001
# 1,2,0: 2/5 |001, 3/5 T000
# 1,1,0: 1/5 all
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

for line in sys.stdin:
  line = line.strip()
  if line == "AFTER:":
    state = "output"
    continue


  if state == "init": continue;
  if len(line)==0: continue
  if line[0] == '[':
    cell_str = line
    continue

  #print(state, cell_str, line[0:4], line)

  if ((cell_str == "[0,1,0](3):") and
      (line[0:4] == "T003")):
    val = float(line.split(":")[4].split(" ")[0])
    if (abs(val - expect_a) > _eps):
      print("FAIL: line:", line, "... expected", expect_a, ", got", val)
      sys.exit(-1)

  if ((cell_str == "[0,1,0](3):") and
      (line[0:4] == "|000")):
    val = float(line.split(":")[4].split(" ")[0])
    if (abs(val - expect_b) > _eps):
      print("FAIL: line:", line, "... expected", expect_b, ", got", val)
      sys.exit(-1)

  if ((cell_str == "[1,1,0](4):") and
      ( (line[0:4] == ".000") or 
        (line[0:4] == "T002") or
        (line[0:4] == "|001") or
        (line[0:4] == "r003") or
        (line[0:4] == "r002") )):
    val = float(line.split(":")[4].split(" ")[0])
    if (abs(val - expect_c) > _eps):
      print("FAIL: line:", line, "... expected", expect_c, ", got", val)
      sys.exit(-1)

    val = float(line.split(":")[7].split(" ")[0])
    if (abs(val - expect_c) > _eps):
      print("FAIL: line:", line, "... expected", expect_c, ", got", val)
      sys.exit(-1)

    val = float(line.split(":")[10].split(" ")[0])
    if (abs(val - expect_c) > _eps):
      print("FAIL: line:", line, "... expected", expect_c, ", got", val)
      sys.exit(-1)

    val = float(line.split(":")[13].split(" ")[0])
    if (abs(val - expect_c) > _eps):
      print("FAIL: line:", line, "... expected", expect_c, ", got", val)
      sys.exit(-1)


  if ((cell_str == "[1,2,0](7):") and
      (line[0:4] == "T000")):
    val = float(line.split(":")[13].split(" ")[0])
    if (abs(val - expect_a) > _eps):
      print("FAIL: line:", line, "... expected", expect_a, ", got", val)

  if ((cell_str == "[1,2,0](7):") and
      (line[0:4] == "|001")):
    val = float(line.split(":")[13].split(" ")[0])
    if (abs(val - expect_b) > _eps):
      print("FAIL: line:", line, "... expected", expect_b, ", got", val)




if verbosity > 0:
  print("ok")
sys.exit(0)




  

  
