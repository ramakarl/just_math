// LICENSE: cc0
//
// To the extent possible under law, the person who associated CC0 with
// this project has waived all copyright and related or neighboring rights
// to this project.
//

var JEOM_VERSION = "0.2.0";
var JEOM_EPS = (1.0/(1024.0*1024.0));
//var JEOM_EPS = (1.0/(1024.0*512.0));

if (typeof module !== "undefined") {
  var numeric = require("./numeric.js");
}

var jeom_info = {
  "w": 1/4,
  "h": 1/7
};

function _3rect_xz(w,h,x,y,z,o) {
  let tri_a = [];

  if (o) {
    let tri = [];
    tri.push( [ x-w/2, y, z+h/2 ] );
    tri.push( [ x+w/2, y, z+h/2 ] );
    tri.push( [ x-w/2, y, z-h/2 ] );
    tri_a.push(tri);

    tri = [];
    tri.push( [ x+w/2, y, z+h/2 ] );
    tri.push( [ x+w/2, y, z-h/2 ] );
    tri.push( [ x-w/2, y, z-h/2 ] );
    tri_a.push(tri);
  }
  else {
    let tri = [];
    tri.push( [ x-w/2, y, z+h/2 ] );
    tri.push( [ x-w/2, y, z-h/2 ] );
    tri.push( [ x+w/2, y, z+h/2 ] );
    tri_a.push(tri);

    tri = [];
    tri.push( [ x+w/2, y, z+h/2 ] );
    tri.push( [ x-w/2, y, z-h/2 ] );
    tri.push( [ x+w/2, y, z-h/2 ] );
    tri_a.push(tri);
  }

  return tri_a;
}

function _3rect_xy(w,h,x,y,z,o) {

  let tri_a = [];

  if (o) {
    let tri = [];
    tri.push( [ x-w/2, y+h/2, z ] );
    tri.push( [ x+w/2, y+h/2, z ] );
    tri.push( [ x-w/2, y-h/2, z ] );
    tri_a.push(tri);

    tri = [];
    tri.push( [ x+w/2, y+h/2, z ] );
    tri.push( [ x+w/2, y-h/2, z ] );
    tri.push( [ x-w/2, y-h/2, z ] );
    tri_a.push(tri);
  }
  else {
    let tri = [];
    tri.push( [ x-w/2, y+h/2, z ] );
    tri.push( [ x-w/2, y-h/2, z ] );
    tri.push( [ x+w/2, y+h/2, z ] );
    tri_a.push(tri);

    tri = [];
    tri.push( [ x+w/2, y+h/2, z ] );
    tri.push( [ x-w/2, y-h/2, z ] );
    tri.push( [ x+w/2, y-h/2, z ] );
    tri_a.push(tri);
  }

  return tri_a;
}


function _3rect_zy(w,h,x,y,z,o) {
  let tri_a = [];

  if (o) {
    let tri = [];
    tri.push( [ x, y+h/2, z-w/2 ] );
    tri.push( [ x, y+h/2, z+w/2 ] );
    tri.push( [ x, y-h/2, z-w/2 ] );
    tri_a.push(tri);

    tri = [];
    tri.push( [ x, y+h/2, z+w/2 ] );
    tri.push( [ x, y-h/2, z+w/2 ] );
    tri.push( [ x, y-h/2, z-w/2 ] );
    tri_a.push(tri);
  }
  else {
    let tri = [];
    tri.push( [ x, y+h/2, z-w/2 ] );
    tri.push( [ x, y-h/2, z-w/2 ] );
    tri.push( [ x, y+h/2, z+w/2 ] );
    tri_a.push(tri);

    tri = [];
    tri.push( [ x, y+h/2, z+w/2 ] );
    tri.push( [ x, y-h/2, z-w/2 ] );
    tri.push( [ x, y-h/2, z+w/2 ] );
    tri_a.push(tri);
  }
  
  return tri_a;
}

// info.n  - number of segments
// info.v  - rotation vector (currently unimplemented)
// info.d  - direction vector (currently unimplemented)
//
// pgn is in 'clipper' format:
//
//   [ {"X": 0.0, "Y": 0.2}, {"X":0.25, "Y": 0.22}, ... , {"X":-1, "Y":0.2} ]
//
//
// pgn is assumed to by X-Z plane.
// That is, X -> x, Y -> z.
//
function jeom_extrude_angle(pgn, info) {
  info = ((typeof info === "undefined") ? {} : info);
  let seg_n = ((typeof info.n === "undefined") ? 16 : info.n);
  let rot_v = ((typeof info.v === "undefined") ? [0,0,1/seg_n] : info.v);
  let dir_v = ((typeof info.d === "undefined") ? [0,0,1/seg_n] : info.d);

  let tri = [];
  for (let seg=0; seg<seg_n; seg++) {
    let a_prv = 2.0*Math.PI*(seg/seg_n);
    let a_nxt = 2.0*Math.PI*((seg+1)/seg_n);

    for (let pgn_idx=0; pgn_idx<pgn.length; pgn_idx++) {
      let idx_prv = pgn_idx;
      let idx_nxt = ((pgn_idx+1)%pgn.length);

      let pnt0 = [ pgn[idx_prv].X, 0.0, pgn[idx_prv].Y ];
      let pnt1 = [ pgn[idx_nxt].X, 0.0, pgn[idx_nxt].Y ];


      let pnt2 = [ pnt1[0], pnt1[1], pnt1[2] ];
      let pnt3 = [ pnt0[0], pnt0[1], pnt0[2] ];

      jeom_rotz(pnt0, a_prv);
      jeom_rotz(pnt1, a_prv);

      jeom_rotz(pnt2, a_nxt);
      jeom_rotz(pnt3, a_nxt);

      tri.push( pnt2[0], pnt2[1], pnt2[2] );
      tri.push( pnt1[0], pnt1[1], pnt1[2] );
      tri.push( pnt0[0], pnt0[1], pnt0[2] );

      tri.push( pnt0[0], pnt0[1], pnt0[2] );
      tri.push( pnt3[0], pnt3[1], pnt3[2] );
      tri.push( pnt2[0], pnt2[1], pnt2[2] );

    }
  }


  return tri;
}

// info.v   - vector
// info.a   - how much to rotate
// info.c   - center [0,0,0]
// info.n   - number of stacks (default 1)
// info.h   - height
//
// pgn is in 'clipper' format:
//
//   [ {"X": 0.0, "Y": 0.2}, {"X":0.25, "Y": 0.22}, ... , {"X":-1, "Y":0.2} ]
//
//
function jeom_extrude(pgn, info) {
  info = ((typeof info === "undefined") ? {} : info);
  let n = ((typeof info.n === "undefined") ? 1 : info.n);
  let v = ((typeof info.v === "undefined") ? [0,0,1/n] : info.v);
  let a = ((typeof info.a === "undefined") ? 0 : info.a);
  let c = ((typeof info.c === "undefined") ? [0,0,0] : info.c);
  let h = ((typeof info.h === "undefined") ? 1.0 : info.h);
  let f_h = ((typeof info.f_h === "undefined") ? (function() { return 1.0; }) : info.f_h);
  let use_delaunay = ((typeof info.delaunay === "undefined") ? false : info.delaunay);

  let base_tri = [];
  let top_tri = [];

  let _z = -h/2;

  let trail_idx = 0;

  //let use_delaunay = false;

  if (use_delaunay) {

    let delaunay_tri = [];
    for (let ii=0; ii<pgn.length; ii++) {
      delaunay_tri.push([pgn[ii].X, pgn[ii].Y]);
    }
    let base_tri_idx = Delaunay.triangulate(delaunay_tri);


    for (let ii=0; ii<base_tri_idx.length; ii+=3) {
      let idx0 = base_tri_idx[ii+0];
      let idx1 = base_tri_idx[ii+1];
      let idx2 = base_tri_idx[ii+2];

      base_tri.push( delaunay_tri[idx0][0], delaunay_tri[idx0][1], _z);
      base_tri.push( delaunay_tri[idx1][0], delaunay_tri[idx1][1], _z);
      base_tri.push( delaunay_tri[idx2][0], delaunay_tri[idx2][1], _z);

      top_tri.push( delaunay_tri[idx0][0], delaunay_tri[idx0][1], -_z);
      top_tri.push( delaunay_tri[idx1][0], delaunay_tri[idx1][1], -_z);
      top_tri.push( delaunay_tri[idx2][0], delaunay_tri[idx2][1], -_z);
    }

  }
  else {

    for (let ii=0; ii<(pgn.length-trail_idx); ii++) {
      let idx0 = ii;
      let idx1 = (ii+1)%(pgn.length-trail_idx);
      base_tri.push( 0, 0, _z );
      base_tri.push( pgn[idx0].X, pgn[idx0].Y, _z);
      base_tri.push( pgn[idx1].X, pgn[idx1].Y, _z);

      top_tri.push( 0, 0, -_z );
      top_tri.push( pgn[idx0].X, pgn[idx0].Y, -_z);
      top_tri.push( pgn[idx1].X, pgn[idx1].Y, -_z);
    }
  }


  let tri = [];
  let p = [0,0, -0.5];
  for (let ii=0; ii<n; ii++) {
    let _z_nxt = ((ii+1)/n) - 0.5;
    let _z_prv = (ii/n) - 0.5;

    for (let idx=0; idx<(pgn.length-trail_idx); idx++) {
      let idx_prv = idx;
      let idx_nxt = (idx+1)%(pgn.length-trail_idx);
      tri.push(pgn[idx_prv].X,  pgn[idx_prv].Y, _z_nxt);
      tri.push(pgn[idx_nxt].X,  pgn[idx_nxt].Y, _z_prv);
      tri.push(pgn[idx_prv].X,  pgn[idx_prv].Y, _z_prv);

      tri.push(pgn[idx_prv].X,  pgn[idx_prv].Y, _z_nxt);
      tri.push(pgn[idx_nxt].X,  pgn[idx_nxt].Y, _z_nxt);
      tri.push(pgn[idx_nxt].X,  pgn[idx_nxt].Y, _z_prv);
    }

  }

  for (let idx=0; idx<base_tri.length; idx+=9) {
    tri.push(base_tri[idx+0], base_tri[idx+1], base_tri[idx+2]);
    tri.push(base_tri[idx+3], base_tri[idx+4], base_tri[idx+5]);
    tri.push(base_tri[idx+6], base_tri[idx+7], base_tri[idx+8]);
  }

  for (let idx=0; idx<base_tri.length; idx+=9) {
    tri.push(top_tri[idx+6], top_tri[idx+7], top_tri[idx+8]);
    tri.push(top_tri[idx+3], top_tri[idx+4], top_tri[idx+5]);
    tri.push(top_tri[idx+0], top_tri[idx+1], top_tri[idx+2]);
  }

  return tri;
}


function jeom_flatten() {
}

function _cross(a,b) {
  let ax = [
    [ 0, -a[2], a[1] ],
    [ a[2], 0, -a[0] ],
    [ -a[1], a[0], 0 ]
  ];

  return numeric.dot(ax,b);
}

function jeom_cube(info) {
  info = ((typeof info === "undefined") ? jeom_info : info);
  let dx = ((typeof info.dx === "undefined") ? 1 : info.dx );
  let dy = ((typeof info.dy === "undefined") ? 1 : info.dy );
  let dz = ((typeof info.dz === "undefined") ? 1 : info.dz );

  let x = dx/2,
      y = dy/2,
      z = dz/2;

  let p0=[], p1=[], p2=[], p3=[];

  let vert = [];

  p0 = [ -x, -y,  z ];
  p1 = [ -x, -y, -z ];
  p2 = [  x, -y, -z ];
  p3 = [  x, -y,  z ];

  vert.push( p0[0], p0[1], p0[2] );
  vert.push( p1[0], p1[1], p1[2] );
  vert.push( p2[0], p2[1], p2[2] );

  vert.push( p0[0], p0[1], p0[2] );
  vert.push( p2[0], p2[1], p2[2] );
  vert.push( p3[0], p3[1], p3[2] );

  p0 = [ -x,  y,  z ];
  p1 = [ -x,  y, -z ];
  p2 = [  x,  y, -z ];
  p3 = [  x,  y,  z ];

  vert.push( p0[0], p0[1], p0[2] );
  vert.push( p2[0], p2[1], p2[2] );
  vert.push( p1[0], p1[1], p1[2] );

  vert.push( p0[0], p0[1], p0[2] );
  vert.push( p3[0], p3[1], p3[2] );
  vert.push( p2[0], p2[1], p2[2] );

  p0 = [ -x, -y,  z ];
  p1 = [ -x, -y, -z ];
  p2 = [ -x,  y, -z ];
  p3 = [ -x,  y,  z ];

  vert.push( p0[0], p0[1], p0[2] );
  vert.push( p2[0], p2[1], p2[2] );
  vert.push( p1[0], p1[1], p1[2] );

  vert.push( p0[0], p0[1], p0[2] );
  vert.push( p3[0], p3[1], p3[2] );
  vert.push( p2[0], p2[1], p2[2] );

  p0 = [  x, -y,  z ];
  p1 = [  x, -y, -z ];
  p2 = [  x,  y, -z ];
  p3 = [  x,  y,  z ];

  vert.push( p0[0], p0[1], p0[2] );
  vert.push( p1[0], p1[1], p1[2] );
  vert.push( p2[0], p2[1], p2[2] );

  vert.push( p0[0], p0[1], p0[2] );
  vert.push( p2[0], p2[1], p2[2] );
  vert.push( p3[0], p3[1], p3[2] );

  p0 = [ -x,  y, -z ];
  p1 = [ -x, -y, -z ];
  p2 = [  x, -y, -z ];
  p3 = [  x,  y, -z ];

  vert.push( p0[0], p0[1], p0[2] );
  vert.push( p2[0], p2[1], p2[2] );
  vert.push( p1[0], p1[1], p1[2] );

  vert.push( p0[0], p0[1], p0[2] );
  vert.push( p3[0], p3[1], p3[2] );
  vert.push( p2[0], p2[1], p2[2] );

  p0 = [ -x,  y,  z ];
  p1 = [ -x, -y,  z ];
  p2 = [  x, -y,  z ];
  p3 = [  x,  y,  z ];

  vert.push( p0[0], p0[1], p0[2] );
  vert.push( p1[0], p1[1], p1[2] );
  vert.push( p2[0], p2[1], p2[2] );

  vert.push( p0[0], p0[1], p0[2] );
  vert.push( p2[0], p2[1], p2[2] );
  vert.push( p3[0], p3[1], p3[2] );

  return vert;
}


function jeom_sphere(info) {
  info = ((typeof info === "undefined") ? jeom_info : info);
  let slice_zdir = ((typeof info.slice_v === "undefined") ? 8 : info.slice_v );
  let slice_a = ((typeof info.slice === "undefined") ? 8 : info.slice );
  let r = ((typeof info.r === "undefined") ? 0.5 : info.r );
  let rx = ((typeof info.rx === "undefined") ? r : info.rx );
  let ry = ((typeof info.ry === "undefined") ? r : info.ry );
  let rz = ((typeof info.rz === "undefined") ? r : info.rz );

  let _eps = JEOM_EPS;

  let n = slice_zdir,
      m = slice_a;

  // theta x-y
  // phi up z
  //

  let vert = [ ];

  for (let i=0; i<n; i++) {
    let v_prv = (i/(n));
    let v_nxt = (((i+1)%(n+1))/n);
    for (let j=0; j<m; j++) {
      let u_prv = (j/(m));
      let u_nxt = (((j+1)%(m+1))/m);

      let theta = 2*Math.PI*u_prv;
      let phi = Math.acos(1 - (2*v_prv));
      let x = rx*Math.sin(phi)*Math.cos(theta);
      let y = ry*Math.sin(phi)*Math.sin(theta);
      let z = rz*Math.cos(phi);

      let p0 = [x,y,z];

      theta = 2*Math.PI*u_nxt;
      phi = Math.acos(1 - (2*v_prv));
      x = rx*Math.sin(phi)*Math.cos(theta);
      y = ry*Math.sin(phi)*Math.sin(theta);
      z = rz*Math.cos(phi);

      let p1 = [x,y,z];

      theta = 2*Math.PI*u_nxt;
      phi = Math.acos(1 - (2*v_nxt));
      x = rx*Math.sin(phi)*Math.cos(theta);
      y = ry*Math.sin(phi)*Math.sin(theta);
      z = rz*Math.cos(phi);

      let p2 = [x,y,z];

      theta = 2*Math.PI*u_prv;
      phi = Math.acos(1 - (2*v_nxt));
      x = rx*Math.sin(phi)*Math.cos(theta);
      y = ry*Math.sin(phi)*Math.sin(theta);
      z = rz*Math.cos(phi);

      let p3 = [x,y,z];

      if (i>0) {

        /*
        let zcount = 0;
        if (numeric.norm2(numeric.sub(p2, p1)) < _eps) { zcount++; }
        if (numeric.norm2(numeric.sub(p1, p0)) < _eps) { zcount++; }
        if (numeric.norm2(numeric.sub(p2, p0)) < _eps) { zcount++; }
        if (zcount > 0) {
          console.log("###!!!! a:", zcount, "(", i, "/", n, j, "/", m, ")", p0, p3, p2);
        }
        */

        vert.push(p2[0], p2[1], p2[2]);
        vert.push(p1[0], p1[1], p1[2]);
        vert.push(p0[0], p0[1], p0[2]);
      }

      if (i<(n-1)) { 

        /*
        zcount = 0;
        if (numeric.norm2(numeric.sub(p0, p3)) < _eps) { zcount++; }
        if (numeric.norm2(numeric.sub(p3, p2)) < _eps) { zcount++; }
        if (numeric.norm2(numeric.sub(p2, p0)) < _eps) { zcount++; }
        if (zcount > 0) {
          console.log("###!!!! b:", zcount, "(", i, "/", n, j, "/", m, ")", p0, p3, p2);
        }
        */

        vert.push(p0[0], p0[1], p0[2]);
        vert.push(p3[0], p3[1], p3[2]);
        vert.push(p2[0], p2[1], p2[2]);

      }

    }
  }

  return vert;
}

// info.n_i  - number of points in circle
// info.n    - number of segments
// info.r_i  - 'inner' radius of torus tire
// info.r_o  - 'outer' radius of where the circle center
//
function jeom_torus(info) {
  info = ((typeof info === "undefined") ? jeom_info : info);
  let n = ((typeof info.n_i === "undefined") ? 8 : info.n_i);
  let r = ((typeof info.r_i === "undefined") ? 0.25 : info.r_i);
  let dx = ((typeof info.r_o === "undefined") ? 0.75  : info.r_o);

  let dy = 0;

  let pgn = [];
  for (let i=0; i<n; i++) {
    let a = 2.0*Math.PI*(i/n);
    let x = r*Math.cos(a) + dx;
    let y = r*Math.sin(a) + dy;

    pgn.push( { "X": x, "Y": y } );
  }

  let tri = jeom_extrude_angle(pgn);
  return tri;

}

// flat part on z,
// lies in x-y plane
//
function jeom_road(info) {
  info = ((typeof info === "undefined") ? jeom_info : info);
  let _w = info.w;
  let _h = info.h;

  let vert = [

    // front panel
    //
    -_w/2,  1/2, -_h/2,  _w/2,  1/2, -_h/2,   -_w/2, -1/2, -_h/2,
     _w/2,  1/2, -_h/2,  _w/2, -1/2, -_h/2,   -_w/2, -1/2, -_h/2,

    // back panel
    //
    -_w/2,  1/2, +_h/2, -_w/2, -1/2, +_h/2,   _w/2,  1/2, +_h/2,
     _w/2,  1/2, +_h/2, -_w/2, -1/2, +_h/2,   _w/2, -1/2, +_h/2,

    // left side stripe
    //
    -_w/2,  1/2, -_h/2,   -_w/2, -1/2, -_h/2,  -_w/2, -1/2, +_h/2,
    -_w/2,  1/2, -_h/2,   -_w/2, -1/2, +_h/2,  -_w/2,  1/2, +_h/2,


    // right side stripe
    //
     _w/2,  1/2, -_h/2,   _w/2, -1/2, +_h/2,  _w/2, -1/2, -_h/2,
     _w/2,  1/2, -_h/2,   _w/2,  1/2, +_h/2,  _w/2, -1/2, +_h/2,

    // back cap (optional)
    //
    -_w/2,  1/2, -_h/2,   -_w/2,  1/2, +_h/2,   _w/2,  1/2, -_h/2,
     _w/2,  1/2, -_h/2,   -_w/2,  1/2, +_h/2,   _w/2,  1/2, +_h/2,

    // front cap (optional)
    //
    -_w/2, -1/2, -_h/2,   _w/2, -1/2, +_h/2,    -_w/2, -1/2, +_h/2,
     _w/2, -1/2, -_h/2,   _w/2, -1/2, +_h/2,    -_w/2, -1/2, -_h/2

  ];

  return vert;
}

// flat part on z,
// lies in x-y plane
//
function jeom_bend() {
  info = ((typeof info === "undefined") ? jeom_info : info);
  let _w = info.w;
  let _h = info.h;

  let tri = [];

  tri.push( [ -_w/2, -1/2,    -_h/2 ] );
  tri.push( [  _w/2, -_w/2, -_h/2 ] );
  tri.push( [  _w/2, -1/2,    -_h/2 ] );
  fr.push(tri);


  tri = [];
  tri.push( [ -_w/2, -1/2,    -_h/2 ] );
  tri.push( [ -_w/2, -_w/2, -_h/2 ] );
  tri.push( [  _w/2, -_w/2, -_h/2 ] );
  fr.push(tri);

  // connecting triangle
  //
  tri = [];
  tri.push( [ -_w/2, -_w/2, -_h/2 ] );
  tri.push( [  _w/2,  _w/2, -_h/2 ] );
  tri.push( [  _w/2, -_w/2, -_h/2 ] );
  fr.push(tri);

  // right square
  //
  tri = [];
  tri.push( [  1/2,    -_w/2, -_h/2 ] )
  tri.push( [  _w/2,  _w/2, -_h/2 ] )
  tri.push( [  1/2,     _w/2, -_h/2 ] )
  fr.push(tri);
  tri = [];
  tri.push( [  1/2,    -_w/2, -_h/2 ] )
  tri.push( [  _w/2, -_w/2, -_h/2 ] )
  tri.push( [  _w/2,  _w/2, -_h/2 ] )
  fr.push(tri);

  //--

  // bottom
  //

  // bottom square
  //
  tri = [];
  tri.push( [ -_w/2, -1/2,    +_h/2 ] );
  tri.push( [  _w/2, -1/2,    +_h/2 ] );
  tri.push( [  _w/2, -_w/2, +_h/2 ] );
  fr.push(tri);

  tri = [];
  tri.push( [ -_w/2, -1/2,    +_h/2 ] );
  tri.push( [  _w/2, -_w/2, +_h/2 ] );
  tri.push( [ -_w/2, -_w/2, +_h/2 ] );
  fr.push(tri);

  // connecting triangle
  //
  tri = [];
  tri.push( [ -_w/2, -_w/2, +_h/2 ] );
  tri.push( [  _w/2, -_w/2, +_h/2 ] );
  tri.push( [  _w/2,  _w/2, +_h/2 ] );
  fr.push(tri);

  // right square
  //
  tri = [];
  tri.push( [  1/2,    -_w/2, +_h/2 ] )
  tri.push( [  1/2,     _w/2, +_h/2 ] )
  tri.push( [  _w/2,  _w/2, +_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [  1/2,    -_w/2, +_h/2 ] )
  tri.push( [  _w/2,  _w/2, +_h/2 ] )
  tri.push( [  _w/2, -_w/2, +_h/2 ] )
  fr.push(tri);

  //--

  // front edge
  //
  tri = [];
  tri.push( [ -_w/2, -1/2, -_h/2 ] )
  tri.push( [  _w/2, -1/2, -_h/2 ] )
  tri.push( [  _w/2, -1/2, +_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [ -_w/2, -1/2, +_h/2 ] )
  tri.push( [ -_w/2, -1/2, -_h/2 ] )
  tri.push( [  _w/2, -1/2, +_h/2 ] )
  fr.push(tri);
  // front right edge
  //
  tri = [];
  tri.push( [  _w/2, -1/2,    -_h/2 ] )
  tri.push( [  _w/2, -_w/2, -_h/2 ] )
  tri.push( [  _w/2, -1/2,    +_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [  _w/2, -1/2,    +_h/2 ] )
  tri.push( [  _w/2, -_w/2, -_h/2 ] )
  tri.push( [  _w/2, -_w/2, +_h/2 ] )
  fr.push(tri);

  // front left edge
  //
  tri = [];
  tri.push( [ -_w/2, -1/2,    -_h/2 ] )
  tri.push( [ -_w/2, -1/2,    +_h/2 ] )
  tri.push( [ -_w/2, -_w/2, -_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [ -_w/2, -1/2,    +_h/2 ] )
  tri.push( [ -_w/2, -_w/2, +_h/2 ] )
  tri.push( [ -_w/2, -_w/2, -_h/2 ] )
  fr.push(tri);

  //---

  // right edge
  //
  tri = [];
  tri.push( [ 1/2, -_w/2, -_h/2 ] )
  tri.push( [ 1/2,  _w/2, -_h/2 ] )
  tri.push( [ 1/2,  _w/2, +_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [ 1/2, -_w/2, +_h/2 ] )
  tri.push( [ 1/2, -_w/2, -_h/2 ] )
  tri.push( [ 1/2,  _w/2, +_h/2 ] )
  fr.push(tri);

  // right up edge
  //
  tri = [];
  tri.push( [ 1/2,     _w/2, -_h/2 ] )
  tri.push( [ _w/2,  _w/2, -_h/2 ] )
  tri.push( [ 1/2,     _w/2, +_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [ _w/2,  _w/2, -_h/2 ] )
  tri.push( [ _w/2,  _w/2, +_h/2 ] )
  tri.push( [ 1/2,     _w/2, +_h/2 ] )
  fr.push(tri);

  // right down edge
  //
  tri = [];
  tri.push( [ 1/2,    -_w/2, -_h/2 ] )
  tri.push( [ 1/2,    -_w/2, +_h/2 ] )
  tri.push( [ _w/2, -_w/2, -_h/2 ] )
  fr.push(tri);


  tri = [];
  tri.push( [ _w/2, -_w/2, -_h/2 ] )
  tri.push( [ 1/2,    -_w/2, +_h/2 ] )
  tri.push( [ _w/2, -_w/2, +_h/2 ] )
  fr.push(tri);

  // diagnoal connecting edge
  //
  tri = [];
  tri.push( [ -_w/2,  -_w/2, -_h/2 ] )
  tri.push( [  _w/2,   _w/2, +_h/2 ] )
  tri.push( [  _w/2,   _w/2, -_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [  _w/2,   _w/2, +_h/2 ] )
  tri.push( [ -_w/2,  -_w/2, -_h/2 ] )
  tri.push( [ -_w/2,  -_w/2, +_h/2 ] )
  fr.push(tri);

  let flat_fr = [];
  for (let i=0; i<fr.length; i++) {
    for (let j=0; j<fr[i].length; j++) {
      for (let k=0; k<fr[i][j].length; k++) {
        flat_fr.push( fr[i][j][k] );
      }
    }
  }

  return flat_fr;
}

function jeom_stair() {
  info = ((typeof info === "undefined") ? jeom_info : info);
  let _w = info.w;
  let _h = info.h;


  let parity = 1;

  let st = [];
  let _st_n = Math.floor( (2/_h) + _h );
  let _st_m = Math.floor( (1/_h) + _h );

  let _st_ds = 1/_st_n;
  let _st_dy = 1/_st_n;
  let _st_dz = 1/_st_n;

  let _r = {};
  let dx=0, dy=0, dz=0;

  // front facing step
  //
  for (let i=0; i<(_st_m-1); i++) {
    dx = 0;
    dy = -0.5 + (i*_st_ds);
    dz = ((3/2)*_st_ds) + (i*_st_ds);
    _r = _3rect_xz( _w, _st_ds,
      dx, dy, dz, 1-parity);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

    // up facing top step
    //
    dx = 0;
    dy = -0.5 + ((1/2)*_st_ds) + (i*_st_ds);
    dz = (2*_st_ds) + (i*_st_ds);
    _r = _3rect_xy( _w, _st_ds,
      dx, dy, dz, 1-parity);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

    // right side stair
    //
    dx = _w/2;
    dy = -0.5 + _st_ds/2 + i*_st_ds;
    dz = +_st_ds/2 + i*_st_ds;
    _r = _3rect_zy(
      3*_st_ds, _st_ds,
      dx, dy, dz,
      parity);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

    // left side stair 
    //
    dx = -_w/2;
    dy = -0.5 + _st_ds/2 + i*_st_ds;
    dz = +_st_ds/2 + i*_st_ds;
    _r = _3rect_zy(
      3*_st_ds, _st_ds,
      dx, dy, dz,
      1-parity);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

  }

  for (let i=0; i<(_st_m+1); i++) {

    // back facing step
    //
    dx = 0;
    dy = -0.5 + _st_ds + (i*_st_ds);
    dz =  - ((1/2)*_st_ds) + (i*_st_ds);
    _r = _3rect_xz( _w, _st_ds,
      dx, dy, dz, parity);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

    // bottom facing bottom step
    //
    dx = 0;
    dy = -0.5 + ((1/2)*_st_ds) + (i*_st_ds);
    dz =  - (_st_ds) + (i*_st_ds);
    _r = _3rect_xy( _w, _st_ds,
      dx, dy, dz, parity);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

  }

  // right side stair
  //
  dx = _w/2;
  dy =  - (_st_ds/2);
  dz = 0.5 - _st_ds;
  _r = _3rect_zy(
    2*_st_ds, _st_ds,
    dx, dy, dz,
    parity);
  for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

  dx = _w/2;
  dy =  + (_st_ds/2);
  dz = 0.5 - (_st_ds/2);
  _r = _3rect_zy(
    1*_st_ds, _st_ds,
    dx, dy, dz,
    parity);
  for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

  // left side stair
  //
  dx = -_w/2;
  dy = - (_st_ds/2);
  dz = 0.5 - _st_ds;
  _r = _3rect_zy(
    2*_st_ds, _st_ds,
    dx, dy, dz,
    1-parity);
  for (let j=0; j<_r.length; j++) { st.push(_r[j]); }



  dx = -_w/2;
  dy = + (_st_ds/2);
  dz = 0.5 - (_st_ds/2);
  _r = _3rect_zy(
    1*_st_ds, _st_ds,
    dx, dy, dz,
    1-parity);
  for (let j=0; j<_r.length; j++) { st.push(_r[j]); }


  // optional end caps
  //
  let _st_endcap = true;
  if (_st_endcap) {
    let _r = {};
    let dx=0, dy=0, dz=0;

    // front endcap
    //
    _r = _3rect_xz( _w, _h,
      0, -0.5, 0, 0);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

    // top endcap
    //
    _r = _3rect_xy( _w, _h,
      0, 0, 0.5, 0);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }
  }


  let flat_st = [];
  for (let i=0; i<st.length; i++) {
    for (let j=0; j<st[i].length; j++) {
      for (let k=0; k<st[i][j].length; k++) {
        flat_st.push( st[i][j][k] );
      }
    }
  }

  return flat_st;
}

// flat part on z,
// lies in x-y plane
//
function jeom_tee(info) {
  info = ((typeof info === "undefined") ? jeom_info : info);
  let _w = info.w;
  let _h = info.h;

  let T = [];

  _r = _3rect_xy( 1, _w, 0, 0, -_h/2, 1);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  _r = _3rect_xy(_w, (1-_w)/2, 0, -1/2 + (1-_w)/4, -_h/2, 1);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  _r = _3rect_xy( 1, _w, 0, 0, +_h/2, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }
  _r = _3rect_xy(_w, (1-_w)/2, 0, -1/2 + (1-_w)/4, +_h/2, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  // optional...
  // bottom and top
  //
  _r = _3rect_xz( _w, _h, 0, -1/2, 0, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }
  _r = _3rect_xz( 1, _h, 0, _w/2, 0, 1);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  // optional...
  // left and right
  //
  _r = _3rect_zy( _h, _w, 1/2, 0, 0, 1);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }
  _r = _3rect_zy( _h, _w,-1/2, 0, 0, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  // inner caps
  //
  _r = _3rect_zy( _h, (1-_w)/2, -_w/2, -1/2+(1-_w)/4, 0, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }
  _r = _3rect_zy( _h, (1-_w)/2,  _w/2, -1/2+(1-_w)/4, 0, 1);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  _r = _3rect_xz( (1-_w)/2, _h, -1/2+(1-_w)/4, -_w/2, 0, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  _r = _3rect_xz( (1-_w)/2, _h,  1/2-(1-_w)/4, -_w/2, 0, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  let flat_T = [];
  for (let i=0; i<T.length; i++) {
    for (let j=0; j<T[i].length; j++) {
      for (let k=0; k<T[i][j].length; k++) {
        flat_T.push( T[i][j][k] );
      }
    }
  }

  return flat_T;
}

// flat part on z,
// lies in x-y plane
//
function jeom_cross(info) {
  info = ((typeof info === "undefined") ? jeom_info : info);
  let _w = info.w;
  let _h = info.h;
  let pl = [];

  _r = _3rect_xy( 1, _w, 0, 0, -_h/2, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  _r = _3rect_xy(_w, (1-_w)/2, 0, -1/2 + (1-_w)/4, -_h/2, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  _r = _3rect_xy(_w, (1-_w)/2, 0,  1/2 - (1-_w)/4, -_h/2, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  _r = _3rect_xy( 1, _w, 0, 0, +_h/2, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  _r = _3rect_xy(_w, (1-_w)/2, 0, -1/2 + (1-_w)/4, +_h/2, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_xy(_w, (1-_w)/2, 0,  1/2 - (1-_w)/4, +_h/2, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  // optional...
  // bottom and top
  //
  _r = _3rect_xz( _w, _h, 0, -1/2, 0, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_xz( _w, _h, 0,  1/2, 0, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  // optional...
  // left and right
  //
  _r = _3rect_zy( _h, _w,  1/2, 0, 0, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_zy( _h, _w, -1/2, 0, 0, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  // middle caps
  //
  let _mc = (1/2) - ((1-_w)/4);
  _r = _3rect_zy( _h, (1-_w)/2, -_w/2, -_mc, 0, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_zy( _h, (1-_w)/2,  _w/2, -_mc, 0, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  _r = _3rect_zy( _h, (1-_w)/2, -_w/2,  _mc, 0, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_zy( _h, (1-_w)/2,  _w/2,  _mc, 0, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  //--

  _r = _3rect_xz( (1-_w)/2, _h, -_mc, -_w/2, 0, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_xz( (1-_w)/2, _h,  _mc, -_w/2, 0, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  _r = _3rect_xz( (1-_w)/2, _h, -_mc,  _w/2, 0, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_xz( (1-_w)/2, _h,  _mc,  _w/2, 0, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  let flat_pl = [];
  for (let i=0; i<pl.length; i++) {
    for (let j=0; j<pl[i].length; j++) {
      for (let k=0; k<pl[i][j].length; k++) {
        flat_pl.push( pl[i][j][k] );
      }
    }
  }


  return flat_pl;
}

// flat part on z,
// lies in x-y plane
//
function jeom_pillar(info) {
  let default_info = { "slice": 16, "r": 0.25, "rf": (function() { return 1.0; }) };
  info = ((typeof info === "undefined") ? default_info : info);
  let _n = ((typeof info.slice === "undefined") ? 16 : info.slice);
  let _r = ((typeof info.r === "undefined") ? 0.25 : info.r);
  let _rf = ((typeof info.rf === "undefined") ? (function() { return 1.0; }) : info.rf);

  let _base = [];
  for (let ii=0; ii<_n; ii++) {
    let theta = 2.0*Math.PI*ii/_n;
    //let f = Math.random()/4 + 0.8;
    f = _rf(ii);
    _base.push({ "X": f*Math.cos(theta)*_r, "Y": -f*Math.sin(theta)*_r });
  }

  let _tri = jeom_extrude(_base);
  return _tri;
}


//----
//----
//----


function jeom_mov(tri, v) {
  for (let i=0; i<tri.length; i+=3) {
    tri[i+0] += v[0];
    tri[i+1] += v[1];
    tri[i+2] += v[2];
  }
}

function jeom_scale(tri, v) {
  for (let i=0; i<tri.length; i+=3) {
    tri[i+0] *= v[0];
    tri[i+1] *= v[1];
    tri[i+2] *= v[2];
  }
}

// https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
//
function jeom_rot(tri, v, theta) {
  let c = Math.cos(theta), s = Math.sin(theta);
  let ic = 1-c, is = 1-s;
  let uxx = v[0]*v[0],
      uyy = v[1]*v[1],
      uzz = v[2]*v[2],
      uxy = v[0]*v[1],
      uxz = v[0]*v[2],
      uyz = v[1]*v[2],
      ux = v[0],
      uy = v[1],
      uz = v[2];

  let uyx = uxy,
      uzx = uxz,
      uzy = uyz;

  let m_r = [
    [ c + uxx*ic,       uxy*ic - uz*s,  uxz*ic + uy*s ],
    [ uyx*ic + uz*s,    c + uyy*ic,     uyz*ic - ux*s ],
    [ uzx*ic - uy*s,    uzy*ic + ux*s,  c + uzz*ic ]
  ];

  let tx=0, ty=0, tz=0;
  for (let i=0; i<tri.length; i+=3) {
    tx = m_r[0][0]*tri[i+0] + m_r[0][1]*tri[i+1] + m_r[0][2]*tri[i+2];
    ty = m_r[1][0]*tri[i+0] + m_r[1][1]*tri[i+1] + m_r[1][2]*tri[i+2];
    tz = m_r[2][0]*tri[i+0] + m_r[2][1]*tri[i+1] + m_r[2][2]*tri[i+2];

    tri[i+0] = tx;
    tri[i+1] = ty;
    tri[i+2] = tz;
  }

}

function jeom_rotx(tri, theta) { jeom_rot(tri, [1,0,0], theta); }
function jeom_roty(tri, theta) { jeom_rot(tri, [0,1,0], theta); }
function jeom_rotz(tri, theta) { jeom_rot(tri, [0,0,1], theta); }

function jeom_dup(tri) {
  let v = Array(tri.length);
  for (let i=0; i<tri.length; i++) {
    v[i] = tri[i];
  }
  return v;
}

function jeom_csg2tri(_csg) {
  let _debug = false;
  let tri = [];

  let pgn = _csg.polygons;

  for (let ii=0; ii<pgn.length; ii++) {

    let vertices = pgn[ii].vertices;
    let pos0 = vertices[0].pos;
    tri.push(pos0.x, pos0.y, pos0.z);

    let pos1 = vertices[1].pos;
    tri.push(pos1.x, pos1.y, pos1.z);

    let pos2 = vertices[2].pos;
    tri.push(pos2.x, pos2.y, pos2.z);

    if (_debug) {
      console.log(ii, "pos0:", pos0, vertices[0].normal);
      console.log(ii, "pos1:", pos1, vertices[1].normal);
      console.log(ii, "pos2:", pos2, vertices[2].normal);
    }

    if (vertices.length > 3) {
      let pos3 = vertices[3].pos;

      if (_debug) {
        console.log(ii, "pos3:", pos3, vertices[3].normal);
      }

      tri.push(pos2.x, pos2.y, pos2.z);
      tri.push(pos3.x, pos3.y, pos3.z);
      tri.push(pos0.x, pos0.y, pos0.z);
    }

    if (_debug) {
      console.log("");
    }

  }
  return tri;
}

function jeom_stl_print(fp, tri) {

  let x = 0, y = 0, z = 0;
  let d = 1;
  let _eps = JEOM_EPS;

  //console.log("solid");
  fp.write("solid\n");
  for (let i=0; i<tri.length; i+=9) {
    let ax = x + tri[i + 0]*d ;
    let ay = y + tri[i + 1]*d ;
    let az = z + tri[i + 2]*d ;

    let bx = x + tri[i + 3]*d ;
    let by = y + tri[i + 4]*d ;
    let bz = z + tri[i + 5]*d ;

    let cx = x + tri[i + 6]*d ;
    let cy = y + tri[i + 7]*d ;
    let cz = z + tri[i + 8]*d ;

    let pA = [ ax, ay, az ];
    let pB = [ bx, by, bz ];
    let pC = [ cx, cy, cz ];

    let cb = numeric.sub( pC, pB );
    let ab = numeric.sub( pA, pB );
    let _n = _cross(cb, ab);

    let _nn = numeric.norm2(_n);
    if (Math.abs(_nn) > _eps) {
      _n = numeric.mul(_n, 1.0/_nn);
    }

    //console.log("facet normal", _n[0], _n[1], _n[2]);
    //console.log("  outer loop");
    //console.log("    vertex", ax, ay, az);
    //console.log("    vertex", bx, by, bz);
    //console.log("    vertex", cx, cy, cz);
    //console.log("  endloop");
    //console.log("endfacet");

    fp.write("facet normal " +  _n[0].toString() + " " +  _n[1].toString() + " " + _n[2].toString() + "\n");
    fp.write("  outer loop\n");
    fp.write("    vertex " + ax.toString() + " " +  ay.toString() + " " +  az.toString() + "\n" );
    fp.write("    vertex " + bx.toString() + " " +  by.toString() + " " +  bz.toString() + "\n" );
    fp.write("    vertex " + cx.toString() + " " +  cy.toString() + " " +  cz.toString() + "\n" );
    fp.write("  endloop\n");
    fp.write("endfacet\n");

  }
  //console.log("endsolid");
  fp.write("endsolid\n");

}

function jeom_gnuplot_print(fp, tri) {

  for (let i=0; i<tri.length; i+=9) {
    fp.write(tri[i+0].toString() + " " + tri[i+1].toString() + " " + tri[i+2].toString() + "\n");
    fp.write(tri[i+3].toString() + " " + tri[i+4].toString() + " " + tri[i+5].toString() + "\n");
    fp.write(tri[i+6].toString() + " " + tri[i+7].toString() + " " + tri[i+8].toString() + "\n");
    fp.write("\n\n");
  }

}

function jeom_off_print(fp, tri, face) {
  let implicit_face = ((typeof face === "undefined") ? true : false );

  //console.log("OFF");
  fp.write("OFF\n");

  //console.log(tri.length/3, tri.length/9, 0);
  if (implicit_face) {
    fp.write( (tri.length/3).toString() + " " + (tri.length/9).toString() + " 0\n");
    //fp.write( (tri.length/3).toString() + " " + (tri.length/3).toString() + " 0\n");
  }
  else {
    fp.write( (tri.length/3).toString() + " " + (face.length/3).toString() + " 0\n");
  }

  for (let i=0; i<tri.length; i+=3) {
    //console.log(tri[i], tri[i+1], tri[i+2]);
    fp.write(tri[i].toString() + " " + tri[i+1].toString() + " " + tri[i+2].toString() + "\n");
  }

  if (implicit_face) {
    for (let i=0; i<tri.length; i+=9) {
      //console.log(3, i+0, i+1, i+2);
      //console.log(3, i+3, i+4, i+5);
      //console.log(3, i+6, i+7, i+8);

      let p0 = i/3;
      let p1 = (i+3)/3;
      let p2 = (i+6)/3;
      fp.write("3 " + (p0).toString() + " " + (p1).toString() + " " + (p2).toString() + "\n");

      //fp.write("3 " + (i+0).toString() + " " + (i+1).toString() + " " + (i+2).toString() + "\n");
      //fp.write("3 " + (i+3).toString() + " " + (i+4).toString() + " " + (i+5).toString() + "\n");
      //fp.write("3 " + (i+6).toString() + " " + (i+7).toString() + " " + (i+8).toString() + "\n");
    }
  }
  else {
    for (let i=0; i<face.length; i+=3) {
      fp.write("3 " + face[i+0].toString() + " " + face[i+1].toString() + " " + face[i+2].toString() + "\n");
    }
  }
}

function jeom_obj_print(fp, tri) {
  for (let i=0; i<tri.length; i+=3) {

    //console.log("v", tri[i+0], tri[i+1], tri[i+2]);
    fp.write("v " + tri[i+0].toString() + " " + tri[i+1].toString() + " " + tri[i+2].toString() + "\n");
  }

  let v0=0, v1=0, v2=0;
  for (let i=0; i<tri.length; i+=9) {
    v0 = Math.floor((i+0)/3);
    v1 = Math.floor((i+3)/3);
    v2 = Math.floor((i+6)/3);

    //console.log("f", v0+1, v1+1, v2+1);
    fp.write("f " + (v0+1).toString() + " "  + (v1+1).toString() + " " + (v2+1).toString() + "\n" );
  }
}

function _vkey(v, _eps) {
  //_eps = ((typeof _eps === "undefined") ? (1/(1024*1024)) : _eps);
  _eps = ((typeof _eps === "undefined") ? JEOM_EPS : _eps);
  let N = Math.floor(1/_eps);
  let ix = Math.floor(N*v[0] + 0.5);
  let iy = Math.floor(N*v[1] + 0.5);
  let iz = Math.floor(N*v[2] + 0.5);
  let key = ix.toString() + ":" + iy.toString() + ":" + iz.toString();
  return key;
}

function jeom_stitch(tri, _eps) {
  //_eps = ((typeof _eps === "undefined") ? (1/(1024*1024)) : _eps);
  _eps = ((typeof _eps === "undefined") ? JEOM_EPS : _eps);

  let vtx_uniq = [];
  let vtx_rep_idx = {};

  let vtx = [];

  let N = Math.floor(1/_eps);
  for (let idx=0; idx<tri.length; idx+=3) {
    let key = _vkey([ tri[idx+0], tri[idx+1], tri[idx+2] ], _eps);

    if (!(key in vtx_rep_idx)) {
      let uniq_idx = vtx_uniq.length;

      vtx.push( tri[idx+0], tri[idx+1], tri[idx+2] );

      vtx_uniq.push( { "name": key, "p": [ tri[idx+0], tri[idx+1], tri[idx+2] ] } );
      vtx_rep_idx[key] = uniq_idx;
    }

  }

  let face = [];

  for (let idx=0; idx<tri.length; idx+=9) {
    let v0 = [ tri[idx+0], tri[idx+1], tri[idx+2] ];
    let v1 = [ tri[idx+3], tri[idx+4], tri[idx+5] ];
    let v2 = [ tri[idx+6], tri[idx+7], tri[idx+8] ];

    let k0 = _vkey(v0);
    let k1 = _vkey(v1);
    let k2 = _vkey(v2);

    let i0 = vtx_rep_idx[k0];
    let i1 = vtx_rep_idx[k1];
    let i2 = vtx_rep_idx[k2];

    face.push( i0, i1, i2 );

  }

  return { "v": vtx, "f": face };
}

if (typeof module !== "undefined") {
  module.exports["extrude_angle"] = jeom_extrude_angle;
  module.exports["extrude"] = jeom_extrude;
  module.exports["flatten"] = jeom_flatten;

  module.exports["cube"] = jeom_cube;
  module.exports["sphere"] = jeom_sphere;
  module.exports["torus"] = jeom_torus;
  module.exports["road"] = jeom_road;
  module.exports["bend"] = jeom_bend;
  module.exports["stair"] = jeom_stair;
  module.exports["tee"] = jeom_tee;
  module.exports["cross"] = jeom_cross;
  module.exports["pillar"] = jeom_pillar;
  module.exports["mov"] = jeom_mov;
  module.exports["scale"] = jeom_scale;
  module.exports["rot"] = jeom_rot;
  module.exports["rotx"] = jeom_rotx;
  module.exports["roty"] = jeom_roty;
  module.exports["rotz"] = jeom_rotz;
  module.exports["dup"] = jeom_dup;
  module.exports["csg2tri"] = jeom_csg2tri;
  module.exports["stl_print"] = jeom_stl_print;
  module.exports["gnuplot_print"] = jeom_gnuplot_print;
  module.exports["off_print"] = jeom_off_print;
  module.exports["obj_print"] = jeom_obj_print;
  module.exports["stitch"] = jeom_stitch;

  module.exports["precision"] = function(_e) {
    if (typeof _e === "undefined") { return JEOM_EPS; }
    JEOM_EPS = _e;
  };
}
