#!/usr/bin/python3
#
# LICENSE: CC0
#
# To the extent possible under law, the person who associated CC0 with
# this file has waived all copyright and related or neighboring rights
# to this file.
#
# You should have received a copy of the CC0 legalcode along with this
# work.  If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
#


import os
import sys
from PIL import Image

import getopt

id2chr_map = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`~!@#$%^&*()_-+=[]{}\\|;:'\",.<>/?"

info = {
  "offset": [0,0],
  "stride": [16,16]
}

def usage(fp):
  fp.write("\nusage:\n")
  fp.write("\n")
  fp.write("  map2tile [-h] [-v] [-V] [-f inputfile] [-s stride] [-o offset]\n")
  fp.write("\n")
  fp.write("  -f        input file (png)\n")
  fp.write("  -s        stride (default " + str(info["stride"][0]) + ")\n")
  fp.write("  -o        offset (default 0)\n")
  fp.write("  -V        verbose\n")
  fp.write("  -h        help (this screen)\n")
  fp.write("\n")


fn = ""

try:
  opts, args = getopt.getopt(sys.argv[1:], "hvVf:s:o:", ["help", "version"])
except getopt.GetoptError as err:
  print(err)
  usage(sys.stderr)
  sys.exit(-1)

for o,a in opts:
  if o in ("-h", "--help"):
    usage(sys.stdout)
    sys.exit(0)
  elif o in ("-v", "--version"):
    usage(sys.stdout)
    sys.exit(0)
  elif o == "-f":
    fn = a
  elif o == "-s":
    info["stride"][0] = int(a)
    info["stride"][1] = int(a)
  elif o == "-o":
    info["offset"][0] = int(a)
    info["offset"][1] = int(a)

#if len(sys.argv)<2:
if len(fn) == 0:
  sys.stderr.write("provide input map (png)\n")
  usage(sys.stderr)
  #print("provide input map (png)")
  sys.exit(-1)
#fn = sys.argv[1]

#if len(sys.argv)>2:
#  _stride = int(sys.argv[2])
#  info["stride"][0] = _stride
#  info["stride"][1] = _stride


#print(fn)

img = Image.open(fn)
pxl = img.load()
sz = img.size

tile_x = int(sz[0]/info["stride"][0])
tile_y = int(sz[1]/info["stride"][1])

sx = info["stride"][0]
sy = info["stride"][1]

uniq_pixel = {}
uniq_tile = {}
uniq_tile_key_id = {}

uniq_tile_id = 0

tile_map = []

for itx in range(tile_x):
  tile_map.append([])
  for ity in range(tile_y):
    tile_map[itx].append(-1)

for itx in range(tile_x):
  for ity in range(tile_y):
    tile_a = []
    for u in range(sx):
      for v in range(sy):
        rgb = pxl[ itx*sx + u, ity*sy + v ]

        s_r = "%0.2x" % rgb[0]
        s_g = "%0.2x" % rgb[1]
        s_b = "%0.2x" % rgb[1]

        tile_a.append(s_r)
        tile_a.append(s_g)
        tile_a.append(s_b)

        key = str(rgb[0]) + ":" + str(rgb[1]) + ":" + str(rgb[2])
        if not (key in uniq_pixel):
          uniq_pixel[key] = 0
        uniq_pixel[key]+=1

    tile_key = "".join(tile_a)
    if not (tile_key in uniq_tile):
      uniq_tile[tile_key] = 0
      uniq_tile_key_id[tile_key] = uniq_tile_id
      uniq_tile_id+=1

    uniq_tile[tile_key]+=1
    tile_map[itx][ity] = uniq_tile_key_id[tile_key]


uniq_count = 0
for key in uniq_pixel:
  #print(key, uniq_pixel[key])
  uniq_count+=1
#print(uniq_count)

uniq_tile_count = 0
for tile_key in uniq_tile:
  #print(tile_key, uniq_tile[tile_key])
  uniq_tile_count+=1
#print(uniq_tile_count)


print("# size:", sz, ", tile:[", tile_x, tile_y, "], uniqe_tile_count:", uniq_tile_count)


#for ity in range(tile_y-1,-1,-1):
for ity in range(tile_y):
  row = []
  for itx in range(tile_x):
    chr_code = '.'
    if tile_map[itx][ity] < len(id2chr_map):
      chr_code = id2chr_map[tile_map[itx][ity]]
    #row.append( str(tile_map[itx][ity]) )
    row.append( chr_code )
  print( " ".join(row) )

