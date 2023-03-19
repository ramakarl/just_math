// LICENSE: cc0
//
// To the extent possible under law, the person who associated CC0 with
// this code has waived all copyright and related or neighboring rights
// to this code.
//
// You should have received a copy of the CC0 legalcode along with this
// work.  If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
//

// create simple 2d road tileset
// re-use the obj files for
//  the stair tileset

var fs = require("fs");

let tiled_compatible = false;

function write_name(tile_name, ofn) {
  let fp = fs.createWriteStream(ofn);
  for (let ii=0; ii<tile_name.length; ii++) {
    fp.write( ii.toString() + "," + tile_name[ii] + "\n");
  }
  fp.end();
}

function write_rule(tile_name, tile_rule, ofn) {

  let n_tile = tile_name.length;

  let fp = fs.createWriteStream(ofn);
  let F = tile_rule;
  for (let idir=0; idir<6; idir++) {
    for (let ii=0; ii<n_tile; ii++) {
      for (let jj=0; jj<n_tile; jj++) {
        fp.write( ii.toString() + "," + jj.toString() + "," + idir.toString() + "," + F[idir][ii][jj].toString() + "\n");
      }
    }
  }
  fp.end();
}

function write_objloc(tile_name, ofn, odir) {
  let fp = fs.createWriteStream(ofn);
  for (let ii=0; ii<tile_name.length; ii++) {
    fp.write( ii.toString() + "," + odir + "/" + tile_name[ii] + ".obj\n" );
  }
  fp.end();
}



// rotations are clockwise
//

let dir_descr = [
  "+1:0:0", "-1:0:0",
  "0:+1:0", "0:-1:0",
  "0:0:+1", "0:0:-1"
];

let oppo_dir = [
  1, 0,
  3, 2,
  5, 4
];

let name_list = [
  ".000",
//  "e000",

  "p000",
  "p001",

  "r000",
  "r001",
  "r002",
  "r003",

  "c000",

  "T000",
  "T001",
  "T002",
  "T003"
];

if (tiled_compatible) {
  name_list.splice(1,0,"e000");
}

let name_link = [

  // .
  [ 0, 0, 0, 0 ],

  // e
  //[ 0, 0, 0, 0 ],

  // p
  [ 0, 0, 1, 1 ],
  [ 1, 1, 0, 0 ],

  // r
  [ 1, 0, 0, 1 ],
  [ 0, 1, 0, 1 ],
  [ 0, 1, 1, 0 ],
  [ 1, 0, 1, 0 ],

  // c
  [ 1, 1, 1, 1 ],

  // T
  [ 1, 1, 0, 1 ],
  [ 0, 1, 1, 1 ],
  [ 1, 1, 1, 0 ],
  [ 1, 0, 1, 1 ]

];

if (tiled_compatible) {
  name_link.splice(1,0,[0,0,0,0]);
}

// initialize rule array
//
let rule_d_a = [ ];
for (let d=0; d<6; d++) {
  rule_d_a.push([]);
  for (let a=0; a<name_list.length; a++) {
    rule_d_a[d].push([]);
    for (let b=0; b<name_list.length; b++) {
      rule_d_a[d][a].push( 0 );
    }
  }
}

// populate 'null' tile (boundary tile)
//
for (let tile_id=0; tile_id<name_list.length; tile_id++) {
  rule_d_a[4][tile_id][0] = 1;
  rule_d_a[4][0][tile_id] = 1;
  rule_d_a[5][tile_id][0] = 1;
  rule_d_a[5][0][tile_id] = 1;
}

for (let src_idir=0; src_idir<4; src_idir++) {
  let dst_idir = oppo_dir[src_idir];

  for (let src_tile_id=0; src_tile_id<name_list.length; src_tile_id++) {
    for (let dst_tile_id=0; dst_tile_id<name_list.length; dst_tile_id++) {

      let src_link = name_link[src_tile_id];
      let dst_link = name_link[dst_tile_id];

      if (src_link[src_idir] == dst_link[dst_idir]) {
        rule_d_a[src_idir][src_tile_id][dst_tile_id] = 1;
      }

    }
  }

}

function debug_print_rule( name_list, rule ) {
  for (let idir=0; idir<6; idir++) {
    for (let src_tile_id=0; src_tile_id<name_list.length; src_tile_id++) {
      for (let dst_tile_id=0; dst_tile_id<name_list.length; dst_tile_id++) {
        if (rule[idir][src_tile_id][dst_tile_id]) {
          console.log( name_list[src_tile_id] + " -(" + dir_descr[idir] + ")-> " + name_list[dst_tile_id] );
        }
      }
    }
  }
}

let dstdir = "../example_tile_collection/";

let tilename_fn = dstdir + "road2d_tilename.csv";
let tilerule_fn = dstdir + "road2d_tilerule.csv";
let objloc_fn = dstdir + "road2d_objloc.csv";

//debug_print_rule( name_list, rule_d_a );

console.log("writing", tilename_fn);
write_name(name_list, tilename_fn);

console.log("writing", tilerule_fn);
write_rule(name_list, rule_d_a, tilerule_fn);

console.log("writing", objloc_fn);
write_objloc(name_list, objloc_fn, "./stair");
