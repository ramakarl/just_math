#!/usr/bin/python3
#
# To the extent possible under law, the person who associated CC0 with
# this file has waived all copyright and related or neighboring rights
# to this file.


import sys
import re

verbosity = 0
state = "init"

tile_map = {}
count = 0

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
    count += 1


expect_only_tile = [' 1', ' 2', ' 3', ' 4' , ' 5', ' 6']

if count != len(expect_only_tile):
  c = str(len(expect_only_tile))
  sys.stderr.write("ERROR: expected " + c + " number of tiles but got " + c + "\n")
  sys.exit(-1)


for u in expect_only_tile:
  if not (u in tile_map):
    sys.stderr.write("ERROR: did not found tile " +  str(u) +  " in list when it should have been\n")
    sys.exit(-1)


if verbosity > 0:
  print("ok")
sys.exit(0)




  

  
