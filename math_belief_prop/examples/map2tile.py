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
#   ./map2tile.py -i demo_pacman.png -s 8
#

import os
import sys
import json
from PIL import Image
import png
import math

import getopt

id2chr_map = "0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ`~!@#$%^&*()_-+=[]{}\\|;:'\",.<>/?"

info = {
  "tilemap": {"fn":"out_tilemap.json", "width":-1, "height":-1 },
  "tileset": {"fn": "out_tileset.png", "width":-1, "height":-1, "stride":[-1,-1], "count":0 },
  "offset": [0,0],
  "stride": [16,16]
}

def usage(fp):
  fp.write("\nusage:\n")
  fp.write("\n")
  fp.write("  map2tile [-h] [-v] [-V] [-i inputfile] [-s stride] [-o offset]\n")
  fp.write("\n")
  fp.write("  -i        input file (png)\n")
  fp.write("  -s        stride (default " + str(info["stride"][0]) + ") (width,height)\n")
  fp.write("  -o        offset (default 0)\n")
  fp.write("  -M <fn>   output tilemap filename (JSON)\n")
  fp.write("  -S <fn>   output tileset (PNG)\n")
  fp.write("  -V        verbose\n")
  fp.write("  -h        help (this screen)\n")
  fp.write("\n")


fn = ""

try:
  opts, args = getopt.getopt(sys.argv[1:], "hvVi:s:o:M:S:", ["help", "version"])
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
  elif o == "-i":
    fn = a
  elif o == "-s":
    info["stride"][0] = int(a)
    info["stride"][1] = int(a)
  elif o == "-o":
    info["offset"][0] = int(a)
    info["offset"][1] = int(a)
  elif o == "-S":
    info["tileset"]["fn"] = a
  elif o == "-M":
    info["tilemap"]["fn"] = a

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

def countVal(ele): return ele["count"]
def create_tileset_png(info, pxl_lib, tilecount_map):

  stride_c, stride_r  = info["stride"][0], info["stride"][1]

  tile_a = []
  for key in tilecount_map:
    tile_a.append( { "key": key, "count": tilecount_map[key], "start_pos": [-1,-1] } )
  #tile_a.sort(key=countVal, reverse=True)
  #tileset_dim = int(math.floor( math.sqrt(float(uniq_tile_id)) + 0.5 ))
  tileset_dim = int(math.ceil( math.sqrt(float(uniq_tile_id)) ))

  info["tileset"]["count"] = len(tile_a)

  info["tileset"]["width"] = tileset_dim
  info["tileset"]["height"] = tileset_dim

  info["tileset"]["stride"] = [ stride_c, stride_r ]

  for idx,ele in enumerate(tile_a):
    sc = (idx % tileset_dim)*stride_c
    sr = int(idx / tileset_dim)*stride_c
    tile_a[idx]["start_pos"][0] = sr
    tile_a[idx]["start_pos"][1] = sc

  png_row = tileset_dim * stride_r
  png_col = tileset_dim * stride_c

  #print("tileset info: count:", len(tile_a), "dim:", tileset_dim, "stride_r:", stride_r, "stride_c:", stride_c)
  #print("  png_row:", png_row, "png_col:", png_col)

  pxl_dat = []
  for r in range(png_row):
    pxl_row = []
    for c in range(png_col):
      pxl_row.append(0)
      pxl_row.append(0)
      pxl_row.append(0)
    pxl_dat.append(pxl_row)

  for ele in tile_a:
    key = ele["key"]
    tile_dat = pxl_lib[key]
    for idx_r in range(stride_r):
      for idx_c in range(stride_c):
        pxl_r = ele["start_pos"][0] + idx_r
        pxl_c = 3*ele["start_pos"][1] + 3*idx_c

        _r = tile_dat[idx_r*3*stride_c + 3*idx_c + 0]
        _g = tile_dat[idx_r*3*stride_c + 3*idx_c + 1]
        _b = tile_dat[idx_r*3*stride_c + 3*idx_c + 2]

        #print(len(pxl_dat), len(pxl_dat[0]), pxl_r, pxl_c)

        pxl_dat[pxl_r][pxl_c+0] = tile_dat[idx_r*3*stride_c + 3*idx_c + 0]
        pxl_dat[pxl_r][pxl_c+1] = tile_dat[idx_r*3*stride_c + 3*idx_c + 1]
        pxl_dat[pxl_r][pxl_c+2] = tile_dat[idx_r*3*stride_c + 3*idx_c + 2]

  ofn = info["tileset"]["fn"]

  w = png.Writer( png_col, png_row, greyscale=False )
  png_ofp = open(ofn, "wb")
  w.write(png_ofp, pxl_dat)
  png_ofp.close()

def export_tiled_json(info, tm):

  stride_r = info["stride"][0]
  stride_c = info["stride"][1]

  tile_r = len(tm)
  tile_c = len(tm[0])

  tiled_map_template = {
    "backgroundcolor" : "#ffffff",
    #"class": "-",
    "height": tile_r,
    "width": tile_c,
    "layers": [],
    "nextobjectid": 1,
    "orientation": "orthogonal",
    "properties": [ ],
    "renderorder" : "right-down",
    "tileheight": stride_r,
    "tilewidth": stride_c,
    "tilesets": [],
    #"tiledversion":"1.0.3",
    "version": 1
  }

  tiled_layer_template = {
    "data": [],
    "height" : tile_r,
    "width": tile_c,
    "name": "main",
    "opacity": 1,
    "properties": [],
    "type":"tilelayer",
    "visible":True,
    "x": 0,
    "y": 0
  }

  tiled_tileset_template = {
    "firstgid": 1,
    "columns": info["tileset"]["width"],
    "name":"tileset",
    #"name":"",
    "image": info["tileset"]["fn"],
    "imageheight": info["tileset"]["height"] * info["tileset"]["stride"][1],
    "imagewidth": info["tileset"]["width"] * info["tileset"]["stride"][0],
    "margin": 0,
    #"properties":[],
    "spacing": 0,
    "tilecount": info["tileset"]["count"],
    "tileheight": info["tileset"]["stride"][1],
    "tilewidth": info["tileset"]["stride"][0]
  }

  #print(json.dumps(tiled_tileset_template, indent=2))

  json_data = tiled_map_template
  json_data["layers"].append( tiled_layer_template )
  json_data["tilesets"].append( tiled_tileset_template )

  for it_r in range(tile_r):
    row = []
    for it_c in range(tile_c):
      #json_data["layers"][0]["data"].append( tile_map[it_r][it_c] )
      json_data["layers"][0]["data"].append( tm[it_r][it_c] )

  #print(json.dumps(json_data, indent=2))

  ofp = open( info["tilemap"]["fn"], "wb" )
  ofp.write( json.dumps( json_data, indent=2 ).encode() )
  ofp.close()
  return json_data


img = Image.open(fn)
pxl = img.load()
sz = img.size

tile_c = int(sz[0]/info["stride"][0])
tile_r = int(sz[1]/info["stride"][1])

sc = info["stride"][0]
sr = info["stride"][1]

uniq_pixel = {}
uniq_tile = {}
uniq_tile_key_id = {}

#uniq_tile_id = 0
uniq_tile_id = 1

pxl_tileset = {}
#pxl_tile_cur = []

tile_map = []

for it_r in range(tile_r):
  tile_map.append([])
  for it_c in range(tile_c):
    tile_map[it_r].append(-1)

for it_r in range(tile_r):
  for it_c in range(tile_c):
    tile_a = []

    pxl_tile_cur = []
    for r in range(sr):
      for c in range(sc):
        #rgb = pxl[ it_r*sr + r, it_c*sc + c ]
        rgb = pxl[ it_c*sc + c , it_r*sr + r ]

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
      #pxl_tileset[str(uniq_tile_id)] = pxl_tile_cur
      pxl_tileset[tile_key] = pxl_tile_cur

      uniq_tile[tile_key] = 0
      uniq_tile_key_id[tile_key] = uniq_tile_id
      uniq_tile_id+=1

    uniq_tile[tile_key]+=1
    tile_map[it_r][it_c] = uniq_tile_key_id[tile_key]


create_tileset_png(info, pxl_tileset, uniq_tile)

uniq_count = 0
for key in uniq_pixel:
  uniq_count+=1

uniq_tile_count = 0
for tile_key in uniq_tile:
  uniq_tile_count+=1

td_map = export_tiled_json(info, tile_map)


## DEBUG
#print(json.dumps(td_map, indent=2))
#debug_print_tilemap(tile_map)

print(">>", info["tilemap"]["fn"], info["tileset"]["fn"] )


