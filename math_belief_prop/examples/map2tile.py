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

# Create a tileset and tilemap from an input "example" image file.
#
# The intent is to use it on an example level, from an old 8-bit
# game like Pacman, Super Mario Bros or The (Original) Legend of Zelda,
# and automatically create a tileset and tilemap from it.
#
# For example, using the `demo_pacman.png`, run as follows:
#
#   ./map2tile.py -f demo_pacman.png -s 8
#

import os
import sys
import json
from PIL import Image
import png

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

if len(fn) == 0:
  sys.stderr.write("provide input map (png)\n")
  usage(sys.stderr)
  sys.exit(-1)

def debug_print_tilemap(tile_map):
  tile_x = len(tile_map)
  tile_y = len(tile_map[0])

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

def export_tiled_json(info, tm):

  stride_x = info["stride"][0]
  stride_y = info["stride"][1]

  tile_x = len(tm)
  tile_y = len(tm[0])

  if not ("tileset" in info):
    info["tileset"] = { }
  if not ("fn" in info):
    info["tileset"]["fn"] = "/dev/stdout"
  if not ("width" in info):
    info["tileset"]["width"] = -1
  if not ("height" in info):
    info["tileset"]["height"] = -1

  tiled_map_template = {
    "backgroundcolor" : "#ffffff",
    #"class": "-",
    "height": tile_y,
    "width": tile_x,
    "layers": [],
    "nextobjectid": 1,
    "orientation": "orthogonal",
    "properties": [ ],
    "renderorder" : "right-down",
    "tileheight": stride_y,
    "tilewidth": stride_x,
    "tilesets": [],
    "version": 1,
    "tiledversion":"1.0.3"
  }

  tiled_layer_template = {
    "data": [],
    "height" : tile_y,
    "width": tile_x,
    "name": "main",
    "opacity": 1,
    "properties": [],
    "type":"ilelayer",
    "visible":True,
    "x": 0,
    "y": 0
  }

  tiled_tileset_template = {
    "firstgid": 0,
    "columns": -1,
    "name":"",
    "image": info["tileset"]["fn"],
    "imageheight": info["tileset"]["height"],
    "imagewidth": info["tileset"]["width"],
    "margin": 0,
    "properties":[],
    "spacing":1,
    "tilecount":-1,
    "tileheight": -1,
    "tilewidth": -1
  }

  json_data = tiled_map_template
  json_data["layers"].append( tiled_layer_template )

  for ity in range(tile_y):
    row = []
    for itx in range(tile_x):
      json_data["layers"][0]["data"].append( tile_map[itx][ity] )

  #print(json.dumps(json_data, indent=2))

  return json_data


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

pxl_tileset = {}
#pxl_tile_cur = []

tile_map = []

for itx in range(tile_x):
  tile_map.append([])
  for ity in range(tile_y):
    tile_map[itx].append(-1)

for itx in range(tile_x):
  for ity in range(tile_y):
    tile_a = []

    pxl_tile_cur = []
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

        pxl_tile_cur.append( rgb[0] )
        pxl_tile_cur.append( rgb[1] )
        pxl_tile_cur.append( rgb[2] )

    tile_key = "".join(tile_a)
    if not (tile_key in uniq_tile):
      pxl_tileset[str(uniq_tile_id)] = pxl_tile_cur

      uniq_tile[tile_key] = 0
      uniq_tile_key_id[tile_key] = uniq_tile_id
      uniq_tile_id+=1

    uniq_tile[tile_key]+=1
    tile_map[itx][ity] = uniq_tile_key_id[tile_key]


uniq_count = 0
for key in uniq_pixel:
  uniq_count+=1

uniq_tile_count = 0
for tile_key in uniq_tile:
  uniq_tile_count+=1

td_map = export_tiled_json(info, tile_map)

## DEBUG
print(json.dumps(td_map, indent=2))
debug_print_tilemap(tile_map)


