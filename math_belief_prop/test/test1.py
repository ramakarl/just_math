#!/usr/bin/python3

import sys
import re

verbosity = 0
state = "init"

tile_map = {}

for line in sys.stdin:
  line = line.strip()
  if line == "[2,1,0](6):":
    state = "count"
    continue

  if line == "":
    state = "fin"
    continue


  if state == "count":
    v = re.sub( '[^\(]*\(([^\)]*)\).*', '\\1', line)
    tile_map[v] = 1


expect_missing_tile = ['35', '36', '37', '38', '39', '40', '41', '42', '43', '44']

for u in expect_missing_tile:
  if u in tile_map:
    sys.stderr.write("ERROR: found tile " +  str(u) +  " in list when it should not have been\n")
    sys.exit(-1)


if verbosity > 0:
  print("ok")
sys.exit(0)




  

  
