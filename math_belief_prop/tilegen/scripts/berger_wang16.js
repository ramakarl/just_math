// LICENSE: cc0
//
// To the extent possible under law, the person who associated CC0 with
// this code has waived all copyright and related or neighboring rights
// to this code.
//
// You should have received a copy of the CC0 legalcode along with this
// work.  If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
//

// 32 px

var fs = require("fs");

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



let order_description = [
  "right", "left", "up", "down"
];

let oppo_dir = [ 1, 0, 3, 2, 5, 4 ];

let wt = [
  [],
  [ 2, 1, 2, 1 ],
  [ 4, 3, 4, 3 ],
  [ 5, 4, 5, 4 ],
  [ 3, 0, 3, 0 ],
  [ 4, 4, 5, 3 ],
  [ 4, 0, 3, 3 ],
  [ 5, 3, 4, 4 ],
  [ 3, 3, 4, 0 ],
  [ 3, 5, 1, 2 ],
  [ 0, 4, 1, 2 ],
  [ 4, 5, 1, 1 ],
  [ 0, 3, 2, 2 ],
  [ 1, 2, 0, 4 ],
  [ 1, 2, 3, 5 ],
  [ 2, 2, 0, 3 ],
  [ 1, 1, 4, 5 ]
];

//let tile_rule = [];
let f_rule = [ ];
for (let idir=0; idir<6; idir++) {
  f_rule.push([]);
  for (let ii=0; ii<wt.length; ii++) {
    f_rule[idir].push([]);
    for (let jj=0; jj<wt.length; jj++)  {
      f_rule[idir][ii].push(0);
    }
  }
}


let tile_name = [];



function xorder(wt) {
  for (let ii=0; ii<wt.length; ii++) {
    let tile = wt[ii];
    console.log(tile[3], tile[1], tile[2], tile[0]);
  }
}

for (let ii=0; ii<wt.length; ii++) {
  tile_name.push( (ii).toString() );
}

for (let ii=0; ii<wt.length; ii++) {
  let tile_a = wt[ii];
  for (let jj=0; jj<wt.length; jj++) {
    let tile_b = wt[jj];

    for (let idir=0; idir<4; idir++) {
      if ( (tile_a.length == 0) || 
           (tile_b.length == 0) ) {
        f_rule[idir][ii][jj] = 1;
        //tile_rule.push( [ii, jj, idir, 1] );
        continue;
      }

      let color_a = tile_a[idir];
      let color_b = tile_b[ oppo_dir[idir] ];

      if (color_a == color_b) {
        f_rule[idir][ii][jj] = 1;
        //tile_rule.push( [ ii, jj, idir, 1 ] );
      }

    }


  }
}

for (let ii=0; ii<wt.length; ii++) {
  f_rule[4][ii][0] = 1;
  f_rule[4][0][ii] = 1;
  f_rule[5][ii][0] = 1;
  f_rule[5][0][ii] = 1;
}

let dstdir = "../example_tile_collection/";

let tilename_fn = dstdir + "berger_wang16_name.csv";
let tilerule_fn = dstdir + "berger_wang16_rule.csv";


write_name(tile_name, tilename_fn);
write_rule(tile_name, f_rule, tilerule_fn);


