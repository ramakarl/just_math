// LICENSE: cc0
//

var fs = require("fs");

//
// To the extent possible under law, the person who associated CC0 with
// this code has waived all copyright and related or neighboring rights
// to this code.
//
// You should have received a copy of the CC0 legalcode along with this
// work.  If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
//

const TILE_WIDTH = 1/3;
const TILE_HEIGHT = 1/4;

//----
// Parts of the following were taken from https://webglfundamentals.org/
// https://github.com/gfxfundamentals/webgl-fundamentals
// which are used with permission via a BSD-3 clause license.
//

var m4 = {

  projection: function(width, height, depth) {

    // Note: This matrix flips the Y axis so 0 is at the top.
    //
    return [
       2 / width, 0, 0, 0,
       0, -2 / height, 0, 0,
       0, 0, 2 / depth, 0,
      -1, 1, 0, 1,
    ];
  },

  multiply: function(a, b) {
    var a00 = a[0 * 4 + 0];
    var a01 = a[0 * 4 + 1];
    var a02 = a[0 * 4 + 2];
    var a03 = a[0 * 4 + 3];
    var a10 = a[1 * 4 + 0];
    var a11 = a[1 * 4 + 1];
    var a12 = a[1 * 4 + 2];
    var a13 = a[1 * 4 + 3];
    var a20 = a[2 * 4 + 0];
    var a21 = a[2 * 4 + 1];
    var a22 = a[2 * 4 + 2];
    var a23 = a[2 * 4 + 3];
    var a30 = a[3 * 4 + 0];
    var a31 = a[3 * 4 + 1];
    var a32 = a[3 * 4 + 2];
    var a33 = a[3 * 4 + 3];
    var b00 = b[0 * 4 + 0];
    var b01 = b[0 * 4 + 1];
    var b02 = b[0 * 4 + 2];
    var b03 = b[0 * 4 + 3];
    var b10 = b[1 * 4 + 0];
    var b11 = b[1 * 4 + 1];
    var b12 = b[1 * 4 + 2];
    var b13 = b[1 * 4 + 3];
    var b20 = b[2 * 4 + 0];
    var b21 = b[2 * 4 + 1];
    var b22 = b[2 * 4 + 2];
    var b23 = b[2 * 4 + 3];
    var b30 = b[3 * 4 + 0];
    var b31 = b[3 * 4 + 1];
    var b32 = b[3 * 4 + 2];
    var b33 = b[3 * 4 + 3];
    return [
      b00 * a00 + b01 * a10 + b02 * a20 + b03 * a30,
      b00 * a01 + b01 * a11 + b02 * a21 + b03 * a31,
      b00 * a02 + b01 * a12 + b02 * a22 + b03 * a32,
      b00 * a03 + b01 * a13 + b02 * a23 + b03 * a33,
      b10 * a00 + b11 * a10 + b12 * a20 + b13 * a30,
      b10 * a01 + b11 * a11 + b12 * a21 + b13 * a31,
      b10 * a02 + b11 * a12 + b12 * a22 + b13 * a32,
      b10 * a03 + b11 * a13 + b12 * a23 + b13 * a33,
      b20 * a00 + b21 * a10 + b22 * a20 + b23 * a30,
      b20 * a01 + b21 * a11 + b22 * a21 + b23 * a31,
      b20 * a02 + b21 * a12 + b22 * a22 + b23 * a32,
      b20 * a03 + b21 * a13 + b22 * a23 + b23 * a33,
      b30 * a00 + b31 * a10 + b32 * a20 + b33 * a30,
      b30 * a01 + b31 * a11 + b32 * a21 + b33 * a31,
      b30 * a02 + b31 * a12 + b32 * a22 + b33 * a32,
      b30 * a03 + b31 * a13 + b32 * a23 + b33 * a33,
    ];
  },

  t2: function(tx, ty, tz) {
    return [
       1,  0,  0, tx,
       0,  1,  0, ty,
       0,  0,  1, tz,
       0,  0,  0,  1,
    ];
  },

  translation: function(tx, ty, tz) {
    return [
       1,  0,  0,  0,
       0,  1,  0,  0,
       0,  0,  1,  0,
       tx, ty, tz, 1,
    ];
  },

  xRotation: function(angleInRadians) {
    var c = Math.cos(angleInRadians);
    var s = Math.sin(angleInRadians);

    return [
      1, 0, 0, 0,
      0, c, s, 0,
      0, -s, c, 0,
      0, 0, 0, 1,
    ];
  },
  yRotation: function(angleInRadians) {
    var c = Math.cos(angleInRadians);
    var s = Math.sin(angleInRadians);

    return [
      c, 0, -s, 0,
      0, 1, 0, 0,
      s, 0, c, 0,
      0, 0, 0, 1,
    ];
  },

  zRotation: function(angleInRadians) {
    var c = Math.cos(angleInRadians);
    var s = Math.sin(angleInRadians);

    return [
       c, s, 0, 0,
      -s, c, 0, 0,
       0, 0, 1, 0,
       0, 0, 0, 1,
    ];
  },

  scaling: function(sx, sy, sz) {
    return [
      sx, 0,  0,  0,
      0, sy,  0,  0,
      0,  0, sz,  0,
      0,  0,  0,  1,
    ];
  },

  translate: function(m, tx, ty, tz) {
    return m4.multiply(m, m4.translation(tx, ty, tz));
  },

  xRotate: function(m, angleInRadians) {
    return m4.multiply(m, m4.xRotation(angleInRadians));
  },

  yRotate: function(m, angleInRadians) {
    return m4.multiply(m, m4.yRotation(angleInRadians));
  },

  zRotate: function(m, angleInRadians) {
    return m4.multiply(m, m4.zRotation(angleInRadians));
  },

  scale: function(m, sx, sy, sz) {
    return m4.multiply(m, m4.scaling(sx, sy, sz));
  },

};

//
//----

function _template_rot_mov(tplate, rx, ry, rz, tx, ty, tz) {

  let tri = [];

  let mx = m4.xRotation(rx);
  let my = m4.yRotation(ry);
  let mz = m4.zRotation(rz);

  let mr = m4.multiply( m4.multiply(mx, my), mz );

  for (let p=0; p<tplate.length; p+=3) {
    tri.push(tx); tri.push(ty); tri.push(tz);
    for (let _i=0; _i<3; _i++) {
      for (let _j=0; _j<3; _j++) {
        tri[p+_i] += tplate[p+_j]*mr[4*_i + _j];
      }
    }
  }

  return tri;
}

function _p_mul_mov(v, s, tx, ty ,tz) {
  for (let i=0; i<v.length; i++) {
    v[i] *= s;

    if ((i%3)==0) { v[i] += tx; }
    if ((i%3)==1) { v[i] += ty; }
    if ((i%3)==2) { v[i] += tz; }
  }

  return v;
}

function _m_v_mul(m, v) {
  let tx = 0;
  let ty = 0;
  let tz = 0;

  let tri = [];

  tri.push(tx);
  tri.push(ty);
  tri.push(tz);
  for (let _i=0; _i<3; _i++) {
    for (let _j=0; _j<3; _j++) {
      tri[_i] += m[4*_i + _j] * v[_j];
    }
  }

  return tri;
}

function _dist3(u,v) {
  let dsq = (u[0] - v[0])*(u[0] - v[0]) +
            (u[1] - v[1])*(u[1] - v[1]) +
            (u[2] - v[2])*(u[2] - v[2]);

  return Math.sqrt(dsq);
}


function _v_in(v, va, _eps) {
  _eps = ((typeof _eps === "undefined") ? (1/128.0) : _eps);

  for (let i=0; i<va.length; i++) {
    //if (_dist3(v, va[i]) <= _eps) { return true; }
    if (_dist3(v, va[i]) <= _eps) { return i; }
  }
  //return false;
  return -1;
}



// template tiles.
// These will be rotated to build the whole tile library.
// The endpoints are there so that we can weed out duplicates
// (from our brute force rotate) and so we can see how each
// tile can join with the others.
//
let _template = {

  "admissible_pos" : [
    { "dv_key" : "1:0:0"  , "dv": [ 1,  0,  0] },
    { "dv_key" : "-1:0:0" , "dv": [-1,  0,  0] },

    { "dv_key" : "0:1:0"  , "dv": [ 0,  1,  0] },
    { "dv_key" : "0:-1:0" , "dv": [ 0, -1,  0] },

    { "dv_key" : "0:0:1"  , "dv": [ 0,  0,  1] },
    { "dv_key" : "0:0:-1" , "dv": [ 0,  0, -1] }
  ],

  "oppo" :  {
    "-1:0:0" :  "1:0:0",
    "1:0:0"  : "-1:0:0",

    "0:-1:0" :  "0:1:0",
    "0:1:0"  : "0:-1:0",

    "0:0:-1" : "0:0:1",
    "0:0:1"  : "0:0:-1"
  },

  "weight": {
    //"d": 0,
    ".": 1,

    "s": 1,
    "|": 1,
    "r": 1,
    "^": 1,

    "t": 1,
    "-": 1,
    "/": 1,
    "~": 1,

    "u": 1,
    "L": 1,
    "!": 1,
    "`": 1

    //"|": 1,
    //"+": 1,
    //"T": 1,
    //"r": 1,
    //"p": 1,
    //"^": 1
  },

  "pdf":  {
    ".": -1,

    "s": 1,
    "|": 1,
    "r": 1,
    "^": 1,

    "t": 1,
    "-": 1,
    "/": 1,
    "~": 1,

    "u": 1,
    "!": 1,
    "L": 1,
    "`": 1

    //"|": -1,
    //"+": -1,
    //"T": -1,
    //"r": -1,
    //"p": -1,
    //"^": -1
  },

  "cdf": [],

  // enpoints tell how we can connect to the other tiles
  //
  // these are flush with the interface plane but in a rectangular pattern, so
  // four points to an interface.
  //
  // The null ('.') and debug ('d') tiles don't have any interfaces
  //
  "endpoint": {}

};


function weight2pd(w) {
  let S = 0.0;
  let pdf = {};
  let cdf = [];

  for (key in w) { S += w[key]; }

  let cdf_s = 0.0;
  for (key in w) {
    pdf[key] = w[key] / S;

    cdf_s += pdf[key];
    cdf.push( {"key": key, "s": cdf_s} );
  }

  return { "pdf": pdf, "cdf": cdf, "w": w };
}


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

// xy plane, direction 0 (+1:0:0)
//
function widget_xy_d0(_w, _h) {
  return [
    [  1/2,  _w/2, -_h/2 ], [  1/2, -_w/2, -_h/2 ],
    [  1/2,  _w/2, +_h/2 ], [  1/2, -_w/2, +_h/2 ]
  ];
}

// xy plane, direction 1 (-1:0:0)
//
function widget_xy_d1(_w,_h) {
  return [
    [ -1/2,  _w/2, -_h/2 ], [ -1/2, -_w/2, -_h/2 ],
    [ -1/2,  _w/2, +_h/2 ], [ -1/2, -_w/2, +_h/2 ]
  ];
}

// xy plane, direction 2 (0:+1:0)
//
function widget_xy_d2(_w,_h) {
  return [
    [ -_w/2,  1/2, -_h/2 ], [ _w/2,  1/2, -_h/2 ],
    [ -_w/2,  1/2, +_h/2 ], [ _w/2,  1/2, +_h/2 ],
  ];
}

// xy plane, direction 3 (0:-1:0)
//
function widget_xy_d3(_w,_h) {
  return [
    [ -_w/2, -1/2, -_h/2 ], [ _w/2, -1/2, -_h/2 ],
    [ -_w/2, -1/2, +_h/2 ], [ _w/2, -1/2, +_h/2 ],
  ];
}

// xz plane, direction 4 (0:0:+1)
//
function widget_xz_d4(_w,_h) {
  return [
    [  _w/2,  +_h/2,  1/2 ], [ -_w/2, +_h/2,  1/2 ],
    [  _w/2,  -_h/2,  1/2 ], [ -_w/2, -_h/2,  1/2 ]
  ];
}

// xz plane, direction 4 (0:0:+1)
//
function widget_xz_d5(_w,_h) {
  return [
    [  _w/2,  +_h/2, -1/2 ], [ -_w/2, +_h/2, -1/2 ],
    [  _w/2,  -_h/2, -1/2 ], [ -_w/2, -_h/2, -1/2 ]
  ];
}

function init_template(template) {

  let _g_w = TILE_WIDTH;
  let _g_h = TILE_HEIGHT;

  // init pdf, cdf
  //
  let S = 0;
  for (let _tsn in template.weight) {
    S += template.weight[_tsn];
  }

  let cdf_s = 0;
  for (let _tsn in template.weight) {
    template.pdf[_tsn] = template.weight[_tsn]/S;
    cdf_s += template.pdf[_tsn];
    template.cdf.push( { "tile_code": _tsn, "s": cdf_s } );
  }
  template.cdf[ template.cdf.length-1 ].s = 1.01;


  let template_list = [];
  for (let key in template.weight) {
    template_list.push(key);
  }


  // init endpoint
  //

  template["endpoint"] = {};
  template["endpoint"]["."] = [];

  //---

  _g_w = 1/3; _g_h = 1/5;
  let w_s = widget_xy_d0(_g_w,_g_h);
  template["endpoint"]["s"] = [];
  for (let i=0; i<w_s.length; i++) {
    template["endpoint"]["s"].push(w_s[i]);
  }

  let w_sr0 = widget_xy_d2(_g_w,_g_h);
  let w_sr1 = widget_xy_d3(_g_w,_g_h);
  template["endpoint"]["|"] = [];
  for (let i=0; i<w_sr0.length; i++) {
    template["endpoint"]["|"].push(w_sr0[i]);
  }
  for (let i=0; i<w_sr1.length; i++) {
    template["endpoint"]["|"].push(w_sr1[i]);
  }

  let w_sb0 = widget_xy_d0(_g_w,_g_h);
  let w_sb1 = widget_xy_d3(_g_w,_g_h);
  template["endpoint"]["r"] = [];
  for (let i=0; i<w_sb0.length; i++) {
    template["endpoint"]["r"].push(w_sb0[i]);
  }
  for (let i=0; i<w_sb1.length; i++) {
    template["endpoint"]["r"].push(w_sb1[i]);
  }

  let w_sS0 = widget_xy_d3(_g_w,_g_h);
  let w_sS1 = widget_xz_d4(_g_w,_g_h);
  template["endpoint"]["^"] = [];
  for (let i=0; i<w_sS0.length; i++) {
    template["endpoint"]["^"].push(w_sS0[i]);
  }
  for (let i=0; i<w_sS1.length; i++) {
    template["endpoint"]["^"].push(w_sS1[i]);
  }


  //---

  _g_w = 1/7; _g_h = 1/11;
  let w_t = widget_xy_d0(_g_w,_g_h);
  template["endpoint"]["t"] = [];
  for (let i=0; i<w_t.length; i++) {
    template["endpoint"]["t"].push(w_t[i]);
  }

  // try and match symbol with representation
  // +-x
  //
  let w_tr0 = widget_xy_d0(_g_w,_g_h);
  let w_tr1 = widget_xy_d1(_g_w,_g_h);
  template["endpoint"]["-"] = [];
  for (let i=0; i<w_tr0.length; i++) {
    template["endpoint"]["-"].push(w_tr0[i]);
  }
  for (let i=0; i<w_tr1.length; i++) {
    template["endpoint"]["-"].push(w_tr1[i]);
  }

  // +x -y
  //
  let w_tb0 = widget_xy_d0(_g_w,_g_h);
  let w_tb1 = widget_xy_d3(_g_w,_g_h);
  template["endpoint"]["/"] = [];
  for (let i=0; i<w_tb0.length; i++) {
    template["endpoint"]["/"].push(w_tb0[i]);
  }
  for (let i=0; i<w_tb1.length; i++) {
    template["endpoint"]["/"].push(w_tb1[i]);
  }

  let w_tS0 = widget_xy_d3(_g_w,_g_h);
  let w_tS1 = widget_xz_d4(_g_w,_g_h);
  template["endpoint"]["~"] = [];
  for (let i=0; i<w_tS0.length; i++) {
    template["endpoint"]["~"].push(w_tS0[i]);
  }
  for (let i=0; i<w_tS1.length; i++) {
    template["endpoint"]["~"].push(w_tS1[i]);
  }


  //---

  _g_w = 1/13; _g_h = 1/17;
  let w_u = widget_xy_d0(_g_w,_g_h);
  template["endpoint"]["u"] = [];
  for (let i=0; i<w_u.length; i++) {
    template["endpoint"]["u"].push(w_u[i]);
  }

  // +-y
  //
  let w_ur0 = widget_xy_d2(_g_w,_g_h);
  let w_ur1 = widget_xy_d3(_g_w,_g_h);
  template["endpoint"]["!"] = [];
  for (let i=0; i<w_ur0.length; i++) {
    template["endpoint"]["!"].push(w_ur0[i]);
  }
  for (let i=0; i<w_ur1.length; i++) {
    template["endpoint"]["!"].push(w_ur1[i]);
  }



  // try and match symbol better with actual endpoint
  // +x +y
  //
  let w_ub0 = widget_xy_d0(_g_w,_g_h);
  let w_ub1 = widget_xy_d2(_g_w,_g_h);
  template["endpoint"]["L"] = [];
  for (let i=0; i<w_ub0.length; i++) {
    template["endpoint"]["L"].push(w_ub0[i]);
  }
  for (let i=0; i<w_ub1.length; i++) {
    template["endpoint"]["L"].push(w_ub1[i]);
  }

  let w_uS0 = widget_xy_d3(_g_w,_g_h);
  let w_uS1 = widget_xz_d4(_g_w,_g_h);
  template["endpoint"]["`"] = [];
  for (let i=0; i<w_uS0.length; i++) {
    template["endpoint"]["`"].push(w_uS0[i]);
  }
  for (let i=0; i<w_uS1.length; i++) {
    template["endpoint"]["`"].push(w_uS1[i]);
  }


  return template;
}


// the geometry for some of the more copmlex shapes is a little
// too involved to list out statically so do it here.
//
function _init_template(template) {

  let _g_w = TILE_WIDTH;
  let _g_h = TILE_HEIGHT;

  let _g_epd = 0;
  let _plat_del = 0;
  let _p_w = _g_w + _plat_del;


  let S = 0;
  for (let _tsn in template.weight) {
    S += template.weight[_tsn];
  }

  let cdf_s = 0;
  for (let _tsn in template.weight) {
    template.pdf[_tsn] = template.weight[_tsn]/S;
    cdf_s += template.pdf[_tsn];
    template.cdf.push( { "tile_code": _tsn, "s": cdf_s } );
  }
  template.cdf[ template.cdf.length-1 ].s = 1.01;

  // enpoints tell how we can connect to the other tiles
  //
  // these are flush with the interface plane but in a rectangular pattern, so
  // four points to an interface.
  //
  // The null ('.') and debug ('d') tiles don't have any interfaces
  //
  template["endpoint"] = {

    ".": [],

    //"d": [],

    "|": [
      [ -_g_w/2, -1/2, -_g_h/2 ], [ _g_w/2, -1/2, -_g_h/2 ],
      [ -_g_w/2, -1/2, +_g_h/2 ], [ _g_w/2, -1/2, +_g_h/2 ],

      [ -_g_w/2,  1/2, -_g_h/2 ], [  _g_w/2,  1/2, -_g_h/2 ],
      [ -_g_w/2,  1/2, +_g_h/2 ], [  _g_w/2,  1/2, +_g_h/2 ]
    ],

    /*
    "p": [
      [ -_g_w/2, -1/2, -_g_h/2 ], [  _g_w/2, -1/2, -_g_h/2 ],
      [ -_g_w/2, -1/2, +_g_h/2 ], [  _g_w/2, -1/2, +_g_h/2 ]
    ],
    */

    "r": [
      [ -_g_w/2, -1/2, -_g_h/2 ], [  _g_w/2, -1/2, -_g_h/2 ],
      [ -_g_w/2, -1/2, +_g_h/2 ], [  _g_w/2, -1/2, +_g_h/2 ],

      [  1/2,  _g_w/2, -_g_h/2 ], [  1/2, -_g_w/2, -_g_h/2 ],
      [  1/2,  _g_w/2, +_g_h/2 ], [  1/2, -_g_w/2, +_g_h/2 ]
    ],


    "+": [
      [ -_g_w/2,  1/2, -_g_h/2 ], [  _g_w/2,  1/2, -_g_h/2 ],
      [ -_g_w/2,  1/2, +_g_h/2 ], [  _g_w/2,  1/2, +_g_h/2 ],

      [ -_g_w/2, -1/2, -_g_h/2 ], [  _g_w/2, -1/2, -_g_h/2 ],
      [ -_g_w/2, -1/2, +_g_h/2 ], [  _g_w/2, -1/2, +_g_h/2 ],

      [  1/2,  _g_w/2, -_g_h/2 ], [  1/2, -_g_w/2, -_g_h/2 ],
      [  1/2,  _g_w/2, +_g_h/2 ], [  1/2, -_g_w/2, +_g_h/2 ],

      [ -1/2,  _g_w/2, -_g_h/2 ], [ -1/2, -_g_w/2, -_g_h/2 ],
      [ -1/2,  _g_w/2, +_g_h/2 ], [ -1/2, -_g_w/2, +_g_h/2 ]
    ],
    
    "T": [
      [  _g_w/2, -1/2, -_g_h/2 ], [ -_g_w/2, -1/2, -_g_h/2 ],
      [  _g_w/2, -1/2, +_g_h/2 ], [ -_g_w/2, -1/2, +_g_h/2 ],
  
      [ -1/2,  _g_w/2, -_g_h/2 ], [ -1/2, -_g_w/2, -_g_h/2 ],
      [ -1/2,  _g_w/2, +_g_h/2 ], [ -1/2, -_g_w/2, +_g_h/2 ],
  
      [  1/2,  _g_w/2, -_g_h/2 ], [  1/2, -_g_w/2, -_g_h/2 ],
      [  1/2,  _g_w/2, +_g_h/2 ], [  1/2, -_g_w/2, +_g_h/2 ]
    ],

    "^": [
      [  _g_w/2, -1/2, -_g_h/2 ], [ -_g_w/2, -1/2, -_g_h/2 ],
      [  _g_w/2, -1/2, +_g_h/2 ], [ -_g_w/2, -1/2, +_g_h/2 ],

      [  _g_w/2,  +_g_h/2,  1/2 ], [ -_g_w/2, +_g_h/2,  1/2 ],
      [  _g_w/2,  -_g_h/2,  1/2 ], [ -_g_w/2, -_g_h/2,  1/2 ]
    
    ]

  };

  // simple plane for debuging
  //
  let debug_plane = false;
  if (debug_plane) {
    template["d"] = [
      -1/2,  1/2, 0,  -1/2, -1/2, 0,    1/2,  1/2, 0,
      -1/2, -1/2, 0,   1/2, -1/2, 0,    1/2,  1/2, 0,
      -1/2,  1/2, 0,   1/2,  1/2, 0,   -1/2, -1/2, 0,
      -1/2, -1/2, 0,   1/2,  1/2, 0,    1/2, -1/2, 0
    ];
  }   
  
  template["|"] = [

    // front panel
    //
    -_g_w/2,  1/2, -_g_h/2,  _g_w/2,  1/2, -_g_h/2,   -_g_w/2, -1/2, -_g_h/2,
     _g_w/2,  1/2, -_g_h/2,  _g_w/2, -1/2, -_g_h/2,   -_g_w/2, -1/2, -_g_h/2,

    // back panel
    //
    -_g_w/2,  1/2, +_g_h/2, -_g_w/2, -1/2, +_g_h/2,   _g_w/2,  1/2, +_g_h/2,
     _g_w/2,  1/2, +_g_h/2, -_g_w/2, -1/2, +_g_h/2,   _g_w/2, -1/2, +_g_h/2,

    // left side stripe
    //
    -_g_w/2,  1/2, -_g_h/2,   -_g_w/2, -1/2, -_g_h/2,  -_g_w/2, -1/2, +_g_h/2,
    -_g_w/2,  1/2, -_g_h/2,   -_g_w/2, -1/2, +_g_h/2,  -_g_w/2,  1/2, +_g_h/2,


    // right side stripe
    //
     _g_w/2,  1/2, -_g_h/2,   _g_w/2, -1/2, +_g_h/2,  _g_w/2, -1/2, -_g_h/2,
     _g_w/2,  1/2, -_g_h/2,   _g_w/2,  1/2, +_g_h/2,  _g_w/2, -1/2, +_g_h/2,

    // back cap (optional)
    //
    -_g_w/2,  1/2, -_g_h/2,   -_g_w/2,  1/2, +_g_h/2,   _g_w/2,  1/2, -_g_h/2,
     _g_w/2,  1/2, -_g_h/2,   -_g_w/2,  1/2, +_g_h/2,   _g_w/2,  1/2, +_g_h/2,

    // front cap (optional)
    //
    -_g_w/2, -1/2, -_g_h/2,   _g_w/2, -1/2, +_g_h/2,    -_g_w/2, -1/2, +_g_h/2,
     _g_w/2, -1/2, -_g_h/2,   _g_w/2, -1/2, +_g_h/2,    -_g_w/2, -1/2, -_g_h/2

  ];


  /*
  template["p"] =  [

    // front panel
    //
    -_p_w/2,    0, -_g_h/2,  _p_w/2,    0, -_g_h/2,   -_g_w/2, -1/2, -_g_h/2,
     _p_w/2,    0, -_g_h/2,  _g_w/2, -1/2, -_g_h/2,   -_g_w/2, -1/2, -_g_h/2,

    // back panel
    //
    -_p_w/2,    0, +_g_h/2, -_g_w/2, -1/2, +_g_h/2,   _p_w/2,    0, +_g_h/2,
     _p_w/2,    0, +_g_h/2, -_g_w/2, -1/2, +_g_h/2,   _g_w/2, -1/2, +_g_h/2,

    // left side stripe
    //
    -_p_w/2,    0, -_g_h/2,   -_g_w/2, -1/2, -_g_h/2,  -_g_w/2, -1/2, +_g_h/2,
    -_p_w/2,    0, -_g_h/2,   -_g_w/2, -1/2, +_g_h/2,  -_p_w/2,    0, +_g_h/2,

    // right side stripe
    //
     _p_w/2,    0, -_g_h/2,   _g_w/2, -1/2, +_g_h/2,  _g_w/2, -1/2, -_g_h/2,
     _p_w/2,    0, -_g_h/2,   _p_w/2,    0, +_g_h/2,  _g_w/2, -1/2, +_g_h/2,

    // back cap (not optional anymore);
    //
    -_p_w/2,    0, -_g_h/2,   -_p_w/2,    0, +_g_h/2,   _p_w/2,    0, -_g_h/2,
     _p_w/2,    0, -_g_h/2,   -_p_w/2,    0, +_g_h/2,   _p_w/2,    0, +_g_h/2,

    // front cap (optional)
    //
    -_g_w/2, -1/2, -_g_h/2,   _g_w/2, -1/2, +_g_h/2,    -_g_w/2, -1/2, +_g_h/2,
     _g_w/2, -1/2, -_g_h/2,   _g_w/2, -1/2, +_g_h/2,    -_g_w/2, -1/2, -_g_h/2

  ];
  */


  // 'r'
  // top
  //

  // bottom square
  //
  let fr = [];
  let tri = [];
  tri.push( [ -_g_w/2, -1/2,    -_g_h/2 ] );
  tri.push( [  _g_w/2, -_g_w/2, -_g_h/2 ] );
  tri.push( [  _g_w/2, -1/2,    -_g_h/2 ] );
  fr.push(tri);


  tri = [];
  tri.push( [ -_g_w/2, -1/2,    -_g_h/2 ] );
  tri.push( [ -_g_w/2, -_g_w/2, -_g_h/2 ] );
  tri.push( [  _g_w/2, -_g_w/2, -_g_h/2 ] );
  fr.push(tri);

  // connecting triangle
  //
  tri = [];
  tri.push( [ -_g_w/2, -_g_w/2, -_g_h/2 ] );
  tri.push( [  _g_w/2,  _g_w/2, -_g_h/2 ] );
  tri.push( [  _g_w/2, -_g_w/2, -_g_h/2 ] );
  fr.push(tri);

  // right square
  //
  tri = [];
  tri.push( [  1/2,    -_g_w/2, -_g_h/2 ] )
  tri.push( [  _g_w/2,  _g_w/2, -_g_h/2 ] )
  tri.push( [  1/2,     _g_w/2, -_g_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [  1/2,    -_g_w/2, -_g_h/2 ] )
  tri.push( [  _g_w/2, -_g_w/2, -_g_h/2 ] )
  tri.push( [  _g_w/2,  _g_w/2, -_g_h/2 ] )
  fr.push(tri);

  //--

  // bottom
  //

  // bottom square
  //
  tri = [];
  tri.push( [ -_g_w/2, -1/2,    +_g_h/2 ] );
  tri.push( [  _g_w/2, -1/2,    +_g_h/2 ] );
  tri.push( [  _g_w/2, -_g_w/2, +_g_h/2 ] );
  fr.push(tri);

  tri = [];
  tri.push( [ -_g_w/2, -1/2,    +_g_h/2 ] );
  tri.push( [  _g_w/2, -_g_w/2, +_g_h/2 ] );
  tri.push( [ -_g_w/2, -_g_w/2, +_g_h/2 ] );
  fr.push(tri);

  // connecting triangle
  //
  tri = [];
  tri.push( [ -_g_w/2, -_g_w/2, +_g_h/2 ] );
  tri.push( [  _g_w/2, -_g_w/2, +_g_h/2 ] );
  tri.push( [  _g_w/2,  _g_w/2, +_g_h/2 ] );
  fr.push(tri);

  // right square
  //
  tri = [];
  tri.push( [  1/2,    -_g_w/2, +_g_h/2 ] )
  tri.push( [  1/2,     _g_w/2, +_g_h/2 ] )
  tri.push( [  _g_w/2,  _g_w/2, +_g_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [  1/2,    -_g_w/2, +_g_h/2 ] )
  tri.push( [  _g_w/2,  _g_w/2, +_g_h/2 ] )
  tri.push( [  _g_w/2, -_g_w/2, +_g_h/2 ] )
  fr.push(tri);

  //--

  // front edge
  //
  tri = [];
  tri.push( [ -_g_w/2, -1/2, -_g_h/2 ] )
  tri.push( [  _g_w/2, -1/2, -_g_h/2 ] )
  tri.push( [  _g_w/2, -1/2, +_g_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [ -_g_w/2, -1/2, +_g_h/2 ] )
  tri.push( [ -_g_w/2, -1/2, -_g_h/2 ] )
  tri.push( [  _g_w/2, -1/2, +_g_h/2 ] )
  fr.push(tri);

  // front right edge
  //
  tri = [];
  tri.push( [  _g_w/2, -1/2,    -_g_h/2 ] )
  tri.push( [  _g_w/2, -_g_w/2, -_g_h/2 ] )
  tri.push( [  _g_w/2, -1/2,    +_g_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [  _g_w/2, -1/2,    +_g_h/2 ] )
  tri.push( [  _g_w/2, -_g_w/2, -_g_h/2 ] )
  tri.push( [  _g_w/2, -_g_w/2, +_g_h/2 ] )
  fr.push(tri);

  // front left edge
  //
  tri = [];
  tri.push( [ -_g_w/2, -1/2,    -_g_h/2 ] )
  tri.push( [ -_g_w/2, -1/2,    +_g_h/2 ] )
  tri.push( [ -_g_w/2, -_g_w/2, -_g_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [ -_g_w/2, -1/2,    +_g_h/2 ] )
  tri.push( [ -_g_w/2, -_g_w/2, +_g_h/2 ] )
  tri.push( [ -_g_w/2, -_g_w/2, -_g_h/2 ] )
  fr.push(tri);

  //---

  // right edge
  //
  tri = [];
  tri.push( [ 1/2, -_g_w/2, -_g_h/2 ] )
  tri.push( [ 1/2,  _g_w/2, -_g_h/2 ] )
  tri.push( [ 1/2,  _g_w/2, +_g_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [ 1/2, -_g_w/2, +_g_h/2 ] )
  tri.push( [ 1/2, -_g_w/2, -_g_h/2 ] )
  tri.push( [ 1/2,  _g_w/2, +_g_h/2 ] )
  fr.push(tri);

  // right up edge
  //
  tri = [];
  tri.push( [ 1/2,     _g_w/2, -_g_h/2 ] )
  tri.push( [ _g_w/2,  _g_w/2, -_g_h/2 ] )
  tri.push( [ 1/2,     _g_w/2, +_g_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [ _g_w/2,  _g_w/2, -_g_h/2 ] )
  tri.push( [ _g_w/2,  _g_w/2, +_g_h/2 ] )
  tri.push( [ 1/2,     _g_w/2, +_g_h/2 ] )
  fr.push(tri);

  // right down edge
  //
  tri = [];
  tri.push( [ 1/2,    -_g_w/2, -_g_h/2 ] )
  tri.push( [ 1/2,    -_g_w/2, +_g_h/2 ] )
  tri.push( [ _g_w/2, -_g_w/2, -_g_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [ _g_w/2, -_g_w/2, -_g_h/2 ] )
  tri.push( [ 1/2,    -_g_w/2, +_g_h/2 ] )
  tri.push( [ _g_w/2, -_g_w/2, +_g_h/2 ] )
  fr.push(tri);

  // diagnoal connecting edge
  //
  tri = [];
  tri.push( [ -_g_w/2,  -_g_w/2, -_g_h/2 ] )
  tri.push( [  _g_w/2,   _g_w/2, +_g_h/2 ] )
  tri.push( [  _g_w/2,   _g_w/2, -_g_h/2 ] )
  fr.push(tri);

  tri = [];
  tri.push( [  _g_w/2,   _g_w/2, +_g_h/2 ] )
  tri.push( [ -_g_w/2,  -_g_w/2, -_g_h/2 ] )
  tri.push( [ -_g_w/2,  -_g_w/2, +_g_h/2 ] )
  fr.push(tri);

  let flat_fr = [];
  for (let i=0; i<fr.length; i++) {
    for (let j=0; j<fr[i].length; j++) {
      for (let k=0; k<fr[i][j].length; k++) {
        flat_fr.push( fr[i][j][k] );
      }
    }
  }

  template["r"] = flat_fr;



  //TODO
  //---
  //
  // stairs (^)
  //
  //     ^
  //     |           x-
  //     z         .xx-
  //     |         xxx
  //     ._y_>    -xx.
  //    /         -x
  //   x
  //  /
  // L

  let parity = 1;

  let st = [];
  let _st_n = Math.floor( (2/_g_h) + _g_h );
  let _st_m = Math.floor( (1/_g_h) + _g_h );

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
    _r = _3rect_xz( _g_w, _st_ds,
      dx, dy, dz, 1-parity);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

    // up facing top step
    //
    dx = 0;
    dy = -0.5 + ((1/2)*_st_ds) + (i*_st_ds);
    dz = (2*_st_ds) + (i*_st_ds);
    _r = _3rect_xy( _g_w, _st_ds,
      dx, dy, dz, 1-parity);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

    // right side stair
    //
    dx = _g_w/2;
    dy = -0.5 + _st_ds/2 + i*_st_ds;
    dz = +_st_ds/2 + i*_st_ds;
    _r = _3rect_zy(
      3*_st_ds, _st_ds,
      dx, dy, dz,
      parity);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }


    // left side stair 
    //
    dx = -_g_w/2;
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
    _r = _3rect_xz( _g_w, _st_ds,
      dx, dy, dz, parity);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

    // bottom facing bottom step
    //
    dx = 0;
    dy = -0.5 + ((1/2)*_st_ds) + (i*_st_ds);
    dz =  - (_st_ds) + (i*_st_ds);
    _r = _3rect_xy( _g_w, _st_ds,
      dx, dy, dz, parity);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

  }

  // right side stair
  //
  dx = _g_w/2;
  dy =  - (_st_ds/2);
  dz = 0.5 - _st_ds;
  _r = _3rect_zy(
    2*_st_ds, _st_ds,
    dx, dy, dz,
    parity);
  for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

  dx = _g_w/2;
  dy =  + (_st_ds/2);
  dz = 0.5 - (_st_ds/2);
  _r = _3rect_zy(
    1*_st_ds, _st_ds,
    dx, dy, dz,
    parity);
  for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

  // left side stair
  //
  dx = -_g_w/2;
  dy = - (_st_ds/2);
  dz = 0.5 - _st_ds;
  _r = _3rect_zy(
    2*_st_ds, _st_ds,
    dx, dy, dz,
    1-parity);
  for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

 
  dx = -_g_w/2;
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
    _r = _3rect_xz( _g_w, _g_h,
      0, -0.5, 0, 0);
    for (let j=0; j<_r.length; j++) { st.push(_r[j]); }

    // top endcap
    //
    _r = _3rect_xy( _g_w, _g_h,
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

  template["^"] = flat_st;


  //---
  // T
  //

  // t faces (top and bottom)
  //
  let T = [];

  _r = _3rect_xy( 1, _g_w, 0, 0, -_g_h/2, 1);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  _r = _3rect_xy(_g_w, (1-_g_w)/2, 0, -1/2 + (1-_g_w)/4, -_g_h/2, 1);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  _r = _3rect_xy( 1, _g_w, 0, 0, +_g_h/2, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }
  _r = _3rect_xy(_g_w, (1-_g_w)/2, 0, -1/2 + (1-_g_w)/4, +_g_h/2, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  // optional...
  // bottom and top
  //
  _r = _3rect_xz( _g_w, _g_h, 0, -1/2, 0, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }
  _r = _3rect_xz( 1, _g_h, 0, _g_w/2, 0, 1);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  // optional...
  // left and right
  //
  _r = _3rect_zy( _g_h, _g_w, 1/2, 0, 0, 1);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }
  _r = _3rect_zy( _g_h, _g_w,-1/2, 0, 0, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  // inner caps
  //
  _r = _3rect_zy( _g_h, (1-_g_w)/2, -_g_w/2, -1/2+(1-_g_w)/4, 0, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }
  _r = _3rect_zy( _g_h, (1-_g_w)/2,  _g_w/2, -1/2+(1-_g_w)/4, 0, 1);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  _r = _3rect_xz( (1-_g_w)/2, _g_h, -1/2+(1-_g_w)/4, -_g_w/2, 0, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  _r = _3rect_xz( (1-_g_w)/2, _g_h,  1/2-(1-_g_w)/4, -_g_w/2, 0, 0);
  for (let j=0; j<_r.length; j++) { T.push(_r[j]); }

  let flat_T = [];
  for (let i=0; i<T.length; i++) {
    for (let j=0; j<T[i].length; j++) {
      for (let k=0; k<T[i][j].length; k++) {
        flat_T.push( T[i][j][k] );
      }
    }
  }

  template["T"] = flat_T;

  //---
  //+
  //

  // + faces (top and bottom)
  //
  let pl = [];

  _r = _3rect_xy( 1, _g_w, 0, 0, -_g_h/2, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  _r = _3rect_xy(_g_w, (1-_g_w)/2, 0, -1/2 + (1-_g_w)/4, -_g_h/2, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  _r = _3rect_xy(_g_w, (1-_g_w)/2, 0,  1/2 - (1-_g_w)/4, -_g_h/2, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  _r = _3rect_xy( 1, _g_w, 0, 0, +_g_h/2, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  _r = _3rect_xy(_g_w, (1-_g_w)/2, 0, -1/2 + (1-_g_w)/4, +_g_h/2, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_xy(_g_w, (1-_g_w)/2, 0,  1/2 - (1-_g_w)/4, +_g_h/2, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  // optional...
  // bottom and top
  //
  _r = _3rect_xz( _g_w, _g_h, 0, -1/2, 0, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_xz( _g_w, _g_h, 0,  1/2, 0, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  // optional...
  // left and right
  //
  _r = _3rect_zy( _g_h, _g_w,  1/2, 0, 0, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_zy( _g_h, _g_w, -1/2, 0, 0, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  // middle caps
  //
  let _mc = (1/2) - ((1-_g_w)/4);
  _r = _3rect_zy( _g_h, (1-_g_w)/2, -_g_w/2, -_mc, 0, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_zy( _g_h, (1-_g_w)/2,  _g_w/2, -_mc, 0, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  _r = _3rect_zy( _g_h, (1-_g_w)/2, -_g_w/2,  _mc, 0, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_zy( _g_h, (1-_g_w)/2,  _g_w/2,  _mc, 0, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  //--

  _r = _3rect_xz( (1-_g_w)/2, _g_h, -_mc, -_g_w/2, 0, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_xz( (1-_g_w)/2, _g_h,  _mc, -_g_w/2, 0, 0);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  _r = _3rect_xz( (1-_g_w)/2, _g_h, -_mc,  _g_w/2, 0, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }
  _r = _3rect_xz( (1-_g_w)/2, _g_h,  _mc,  _g_w/2, 0, 1);
  for (let j=0; j<_r.length; j++) { pl.push(_r[j]); }

  let flat_pl = [];
  for (let i=0; i<pl.length; i++) {
    for (let j=0; j<pl[i].length; j++) {
      for (let k=0; k<pl[i][j].length; k++) {
        flat_pl.push( pl[i][j][k] );
      }
    }
  }

  template["+"] = flat_pl;


  //----

}


// by using the endpoint library,
// build a list of all rotations of the endpoints (raw_lib),
// a map of equivalent representations (equiv_lib),
// a map that has the representative of each rotational class (repr_map),
// a 'tile_attach' list of tiles that can attach to each other (based off
// of the endpoint)
//
function build_tile_library( template ) {
  let raw_lib = {};
  let rot_lib = {};

  // equiv_map holds array of equivalent rotations
  //
  let _equiv_map = {};

  // repr map takes a single representation for all
  // equivalent rotations
  //
  let _repr_map = {};

  // *all* admissible nieghbors
  //
  let admissible_tile_pos = {};
  let admissible_pos_tile = {};
  let admissible_nei = {};

  let _endp_lib = template.endpoint;

  for (let pkey in _endp_lib) {

    let _endp = _endp_lib[pkey];

    let _type_a = [];
    let _type_a_key = [];

    for (let xidx=0; xidx<4; xidx++) {
      for (let yidx=0; yidx<4; yidx++) {
        for (let zidx=0; zidx<4; zidx++) {

          mx = m4.xRotation(xidx*Math.PI/2);
          my = m4.yRotation(yidx*Math.PI/2);
          mz = m4.zRotation(zidx*Math.PI/2);
          let mr = m4.multiply( m4.multiply(mx, my), mz );

          let ukey = pkey + xidx.toString() + yidx.toString() + zidx.toString();

          raw_lib[ukey] = [];

          // rotate endpoints around the various pi/2 rotations (x,y,z)
          // and put them in the 'raw' library
          //
          for (let ep_idx=0; ep_idx<_endp.length; ep_idx++) {
            let v = _m_v_mul(mr, _endp[ep_idx]);
            raw_lib[ukey].push(v);
          }

          _type_a.push( raw_lib[ukey] );
          _type_a_key.push(ukey);

          rot_lib[ukey] = { "m": [mx, my, mz], "r": [xidx, yidx, zidx ] };

        }
      }

    }

    // once in the library, go through and compare each
    // to see if they're equivalent by seeing if the endpoints
    // line up
    //  
    for (let i=0; i<_type_a.length; i++) {
      for (let j=i+1; j<_type_a.length; j++) {

        let k=0;
        for (k=0; k<_type_a[i].length; k++) {
          if (_v_in(_type_a[i][k], _type_a[j])<0) { break; }
        }

        if (k==_type_a[i].length) {
          if (!(_type_a_key[i] in _equiv_map)) { _equiv_map[_type_a_key[i]] = []; }
          if (!(_type_a_key[j] in _equiv_map)) { _equiv_map[_type_a_key[j]] = []; }
          _equiv_map[_type_a_key[i]].push( _type_a_key[j] );
          _equiv_map[_type_a_key[j]].push( _type_a_key[i] );
        }

      }
    }

    // once we've constructed the equiv_map, go through and pick out the
    // representative
    //
    for (let key in _equiv_map) {
      if (key in _repr_map) { continue; }
      _repr_map[key] = key;
      for (let ii=0; ii<_equiv_map[key].length; ii++) {
        _repr_map[_equiv_map[key][ii]] = key;
      }
    }

  }

  let uniq_repr = {};
  for (let key in _repr_map) {
    let repr_id = _repr_map[key];
    if (!(repr_id in uniq_repr)) {
      uniq_repr[repr_id] = {"count":0, "attach_dv": {}, "pos_tile_attach":{} };
    }
    uniq_repr[repr_id].count++;
  }

  let tile_attach = {};

  // now for each representative, test every other tile to see if can
  // be joined
  //
  for (let anchor_key in uniq_repr) {
    if ((anchor_key.charAt(0) == '.') ||
        (anchor_key.charAt(0) == 'd')) { continue; }
    for (let test_key in uniq_repr) {
      if ((test_key.charAt(0) == '.') ||
          (test_key.charAt(0) == 'd')) { continue; }

      // anchor_key is at the 'center' where test_key
      // is shifted around the 3x3x3 cube, excluding the
      // center point.
      //
      for (let dx=-1; dx<2; dx++) {
        for (let dy=-1; dy<2; dy++) {
          for (let dz=-1; dz<2; dz++) {

            // test_key can't be at center because that's where the
            // anchor_key is
            //
            if ((dx==0) && (dy==0) && (dz==0)) { continue; }

            //---

            let anch_endp = raw_lib[anchor_key];
            let test_endp = raw_lib[test_key];

            let endp_group = -1;
            let endp_count = 0;
            for (let idx=0; idx<test_endp.length; idx++) {
              let tv = [ test_endp[idx][0] + dx, test_endp[idx][1] + dy, test_endp[idx][2] + dz ];
              let _anch_endp_pos = _v_in(tv, anch_endp);
              if (_anch_endp_pos >= 0) {
                endp_count++;
                endp_group = Math.floor(_anch_endp_pos/4);
              }

            }

            if (endp_count==4) {
              if (!(anchor_key in tile_attach)) { tile_attach[anchor_key] = {}; }
              if (!(test_key in tile_attach[anchor_key])) {
                tile_attach[anchor_key][test_key] = { "anchor": anchor_key, "attach": test_key, "dv": [], "endpoint_group": [] };
              }
              tile_attach[anchor_key][test_key].dv.push( [dx, dy, dz ] );
              tile_attach[anchor_key][test_key].endpoint_group.push( endp_group );

              dv_key = dx.toString() + ":" + dy.toString() + ":" + dz.toString();
              uniq_repr[anchor_key].attach_dv[ dv_key ] = [ dx, dy, dz ];

              if (!(dv_key in uniq_repr[anchor_key].pos_tile_attach)) {
                uniq_repr[anchor_key].pos_tile_attach[dv_key] = {};
              }
              uniq_repr[anchor_key].pos_tile_attach[dv_key][test_key] = true;
            }

          }
        }
      }
    }

  }

  // easy access to endpoint group count
  //
  for (let key in uniq_repr) {
    let n_endpoint_group = 0;
    for (let _dvkey in uniq_repr[key].attach_dv) {
      n_endpoint_group++;
    }
    uniq_repr[key].n_endpoint_group = n_endpoint_group;
  }

  let admissible_pos = template.admissible_pos;
  let oppo  = template.oppo;

  // tile_attach only has actual connections
  // use it to fill out the admissible_attach
  // hash with *all* valid neighboring tile pairs
  //
  for (let key_anchor in uniq_repr) {

    if (!(key_anchor in admissible_nei)) {
      admissible_nei[key_anchor] = {};
    }

    for (let key_nei in uniq_repr) {

      if (!(key_nei in admissible_nei)) {
        admissible_nei[key_nei] = {};
      }

      for (let ii=0; ii<admissible_pos.length; ii++) {
        let dv_anch = admissible_pos[ii].dv_key;
        let dv_nei = oppo[dv_anch];

        if ((dv_anch in uniq_repr[key_anchor].pos_tile_attach) &&
            (dv_nei  in uniq_repr[key_nei].pos_tile_attach)) {
          if (key_nei in uniq_repr[key_anchor].pos_tile_attach[dv_anch]) {

            if (!(key_nei in admissible_nei[key_anchor])) {
              admissible_nei[key_anchor][key_nei] = {};
            }
            if (!(dv_anch in admissible_nei[key_anchor][key_nei])) {
              admissible_nei[key_anchor][key_nei][dv_anch] = {};
            }
            if (!(dv_anch in admissible_nei[key_anchor])) {
              admissible_nei[key_anchor][dv_anch] = {};
            }


            if (!(key_anchor in admissible_nei[key_nei])) {
              admissible_nei[key_nei][key_anchor] = {};
            }
            if (!(key_anchor in admissible_nei[key_nei][key_anchor])) {
              admissible_nei[key_nei][key_anchor][dv_nei] = {};
            }
            if (!(dv_nei in admissible_nei[key_nei])) {
              admissible_nei[key_nei][dv_nei] = {};
            }


            admissible_nei[key_anchor][key_nei][dv_anch] = {"conn": true};
            admissible_nei[key_nei][key_anchor][dv_nei] = {"conn": true};

            admissible_nei[key_anchor][dv_anch][key_nei] = {"conn": true};
            admissible_nei[key_nei][dv_nei][key_anchor] = {"conn": true};
          }
        }
        if ((!(dv_anch in uniq_repr[key_anchor].pos_tile_attach)) &&
            (!(dv_nei  in uniq_repr[key_nei].pos_tile_attach))) {

          if (!(key_nei in admissible_nei[key_anchor])) {
            admissible_nei[key_anchor][key_nei] = {};
          }
          if (!(dv_anch in admissible_nei[key_anchor][key_nei])) {
            admissible_nei[key_anchor][key_nei][dv_anch] = {};
          }
          if (!(dv_anch in admissible_nei[key_anchor])) {
            admissible_nei[key_anchor][dv_anch] = {};
          }

          if (!(key_anchor in admissible_nei[key_nei])) {
            admissible_nei[key_nei][key_anchor] = {};
          }
          if (!(key_anchor in admissible_nei[key_nei][key_anchor])) {
            admissible_nei[key_nei][key_anchor][dv_nei] = {};
          }
          if (!(dv_nei in admissible_nei[key_nei])) {
            admissible_nei[key_nei][dv_nei] = {};
          }

          admissible_nei[key_anchor][key_nei][dv_anch] = {"conn": false};
          admissible_nei[key_nei][key_anchor][dv_nei] = {"conn": false};

          admissible_nei[key_anchor][dv_anch][key_nei] = {"conn": false};
          admissible_nei[key_nei][dv_nei][key_anchor] = {"conn": false};
        }

      }

    }
  }

  template["raw_lib"] = raw_lib;
  template["equiv_map"] = _equiv_map;
  template["repr_map"] = _repr_map
  template["uniq_repr"] = uniq_repr;
  template["tile_attach"] = tile_attach;
  template["rot_lib"] = rot_lib;
  template["admissible_nei"] = admissible_nei;


  // restrict undesirable combinations
  //
  //filter_steeple(template);


  //---
  //
  // build matrix
  //
  let _eps = (1/(1024*1024*1024));

  let tile_name_a = [];
  let tile_name_a_map_idx = {};
  for (let tn in uniq_repr) {
    tile_name_a_map_idx[tn] = tile_name_a.length;
    tile_name_a.push(tn);
  }

  let F_matrix = {};
  let _adj_info = {};

  template["svd"] = [];

  for (let pos_idx=0; pos_idx<admissible_pos.length; pos_idx++) {

    let dv_key = admissible_pos[pos_idx].dv_key;

    let _F = [];
    let _F_map = {};
    for (let ii=0; ii<tile_name_a.length; ii++) {
      let anchor_key = tile_name_a[ii];

      _F.push([]);
      _F_map[ tile_name_a[ii] ] = {};
      for (let jj=0; jj<tile_name_a.length; jj++) {
        let nei_key = tile_name_a[jj];

        let _val = ((nei_key in admissible_nei[ anchor_key ][ dv_key ]) ? 1 : 0);

        _F[ii].push( _val );
        _F_map[ anchor_key ][ nei_key ] = _val;
      }
    }

    F_matrix[ dv_key ] = _F;

    let _SVt = [];
    let _U = [];
    let S = [];


    if (typeof numeric !== "undefined") {

      let svd = numeric.svd( _F );

      let Sm = [];
      let Vt = numeric.transpose(svd.V);
      let nz_count = 0;

      template.svd.push(svd);

      for (let ii=0; ii<svd.S.length; ii++) {
        S.push([]);
        for (let jj=0; jj<svd.S.length; jj++) {
          S[ii].push( (ii==jj) ? svd.S[ii] : 0 );
        }
        if (Math.abs(svd.S[ii]) > _eps) { nz_count++; }
      }

      let SVt_all = numeric.dot(S, numeric.transpose(svd.V));

      //console.log("SVt_all", SVt_all);
      //console.log(">>>nz", nz_count);

      for (let ii=0; ii<nz_count; ii++) { _SVt.push(SVt_all[ii]); }

      //console.log( svd.U.length, svd.U[0].length, "::", svd.S.length, "::", svd.V.length, svd.V[0].length);
      //let SVt_all = numeric.dot(Sm, Vt);

      for (let ii=0; ii<svd.U.length; ii++) {
        _U.push([]);
        for (let jj=0; jj<nz_count; jj++) {
          _U[ii].push( svd.U[ii][jj] );
        }
      }

    }

    _adj_info[ dv_key ] = {
      "F": _F,
      "Fmap": _F_map,
      "S": S,
      "SV": _SVt,
      "U": _U
    };

  }

  //
  //---

  template["tile_name"] = tile_name_a;
  template["tile_name_idx"] = tile_name_a_map_idx;

  template["F"] = F_matrix;
  template["adj"] = _adj_info;

}

function filter_steeple(template) {
  let _eps = 1/(1024*1024);

  let admissible_nei  = template.admissible_nei;
  let admissible_pos  = template.admissible_pos;
  let oppo            = template.oppo;
  let raw_lib         = template.raw_lib;

  let delete_list = [];

  for (let key_anchor in admissible_nei) {
    if (key_anchor.charAt(0) != '^') { continue; }

    let anc_p_repr = [0,0,0];
    let endp = raw_lib[key_anchor];
    for (let ii=0; ii<endp.length; ii++) {
      anc_p_repr[0] += endp[ii][0];
      anc_p_repr[1] += endp[ii][1];
      anc_p_repr[2] += endp[ii][2];
    }

    for (let posidx=0; posidx<admissible_pos.length; posidx++) {
      let dv_anc_key = admissible_pos[posidx].dv_key;
      let dv_nei_key = oppo[dv_anc_key];

      let dv_anc = admissible_pos[posidx].dv;
      let dv_nei = [ -dv_anc[0], -dv_anc[1], -dv_anc[2] ];

      for (let key_nei in admissible_nei[key_anchor][dv_anc_key]) {
        if (key_nei.charAt(0) != '^') { continue; }
        if (!admissible_nei[key_anchor][dv_anc_key][key_nei].conn) { continue; }

        let nei_p_repr = [0,0,0];
        let endp = raw_lib[key_nei];
        for (let ii=0; ii<endp.length; ii++) {
          nei_p_repr[0] += endp[ii][0];
          nei_p_repr[1] += endp[ii][1];
          nei_p_repr[2] += endp[ii][2];
        }

        let anc_v = [
          anc_p_repr[0] - 2*dv_anc[0],
          anc_p_repr[1] - 2*dv_anc[1],
          anc_p_repr[2] - 2*dv_anc[2]
        ];

        let nei_v = [
          nei_p_repr[0] - 2*dv_nei[0],
          nei_p_repr[1] - 2*dv_nei[1],
          nei_p_repr[2] - 2*dv_nei[2]
        ];

        let de = Math.abs(nei_v[0] - anc_v[0] + nei_v[1] - anc_v[1] + nei_v[2] - anc_v[2]);

        if (de < _eps) {
          delete_list.push( [ key_anchor, key_nei, dv_anc_key ] );
          delete_list.push( [ key_anchor, dv_anc_key, key_nei ] );
        }
      }

    }
  }
  for (let ii=0; ii<delete_list.length; ii++) {
    let a = delete_list[ii][0];
    let b = delete_list[ii][1];
    let c = delete_list[ii][2];

    delete admissible_nei[a][b][c];
  }

}


function _has_nei(template, name, dir) {
  let v = template.admissible_nei[name][dir];

  for (let key in v) {
    if (v[key].conn)  { return true; }
  }
  return false;
}

function csv_name(template, fn) {

  let _an = template.admissible_nei;

  let dir = [
    "1:0:0",
    "-1:0:0",
    "0:1:0",
    "0:-1:0",
    "0:0:1",
    "0:0:-1"
  ];

  let colors = [ "#000000", "#ff0000", "#00ff00", "#0000ff" ];

  //let fp = fs.openSync(fn, "w");

  //console.log("#id,name");
  let s = "#id,name\n" ;
  //fp.writeFileSync(s);
  for (let i=0; i<template.tile_name.length; i++) {

    let conn_vec = [0,0,0,0,0,0];

    let c = colors[0];
    if (i>0) {
      c = colors[Math.floor((i)*3/template.tile_name.length) + 1];
    }

    for (let j=0; j<6; j++) {
      if (_has_nei(template, template.tile_name[i], dir[j])) {
        conn_vec[j] = 1;
      }
    }



    //console.log(i + "," + template.tile_name[i]);
    s += i + "," + template.tile_name[i] + "," + conn_vec.join(",") + "," + c + "\n";
    //fp.writeFileSync(s);
  }

  fs.writeFileSync(fn, s);

  //fp.close();
}

function csv_rule(template, fn) {
  let _eps = (1.0/(1024.0*1024.0));
  let _verbose = true;

  //let fp = fs.openSync(fn, "w");

  let s = "#tileida,tileidb,dir,val\n";
  //console.log("#tileida,tileidb,dir,val");
  //fp.writeFileSync(s);

  let dv_key = [];

  for (let i=0; i<template.admissible_pos.length; i++) {
    s += "# " + i + " , " + template.admissible_pos[i].dv_key + "\n";
    //console.log("# " + i + " , " + template.admissible_pos[i].dv_key);
    //fp.writefileSync(s);
    dv_key.push( template.admissible_pos[i].dv_key );
  }

  let F = template.F;

  for (let i=0; i<dv_key.length; i++) {
    for (let j=0; j<F[dv_key[i]].length; j++) {
      for (let k=0; k<F[dv_key[i]][j].length; k++) {
        if (F[dv_key[i]][j][k] < _eps) { continue; }

        if (_verbose) {
          s += "#\n# " + template.tile_name[j] + " =(" + dv_key[i] + ")=> " + template.tile_name[k] + " : " + F[dv_key[i]][j][k] + "\n";
        }

        s += j + "," + k + "," + i + "," + F[dv_key[i]][j][k] + "\n";
        //console.log(j + "," + k + "," + i + "," + F[dv_key[i]][j][k]);
        //fp.writeFileSync(s);
      }
    }
  }

  fs.writeFileSync(fn, s);

  //fp.close();
}

function csv_constraint(template, fn, dist) {
  let n=11, m=11, z=3;

  let remove_list = [];

  let tile_name_idx_map = template.tile_name_idx;
  let tile_name_list = template.tile_name;

  let admissible_tile_id = [];

  let stu = [];

  for (let tn in tile_name_idx_map) {
    if ((tn.charAt(0) == 'u') ||
        (tn.charAt(0) == 's') ||
        (tn.charAt(0) == 't')) {
      continue;
    }
    admissible_tile_id.push( tile_name_idx_map[tn] );
  }

  stu.push( tile_name_idx_map["s000"] );
  stu.push( tile_name_idx_map["s002"] );

  stu.push( tile_name_idx_map["t000"] );
  stu.push( tile_name_idx_map["t002"] );

  stu.push( tile_name_idx_map["u000"] );
  stu.push( tile_name_idx_map["u002"] );

  let gr = [];
  for (let i=0; i<n; i++) {
    gr.push([]);
    for (let j=0; j<m; j++) {
      gr[i].push([]);
      for (let k=0; k<z; k++) {
        gr[i][j].push(admissible_tile_id);
      }
    }
  }

  gr[  1][  1][0] = [ stu[0] ];
  //gr[n-2][  1][0] = [ stu[1] ];
  gr[n-2][m-2][0] = [ stu[1] ];

  //gr[3][  1][0] = [ stu[1] ];

  gr[  1][  5][0] = [ stu[2] ];
  gr[n-2][  5][0] = [ stu[3] ];

  //gr[3][  5][0] = [ stu[3] ];

  gr[  1][m-2][0] = [ stu[4] ];
  //gr[n-2][m-2][0] = [ stu[5] ];
  gr[n-2][  1][0] = [ stu[5] ];

  //gr[4][m-2][0] = [ stu[5] ];

  //---

  let s = "";

  for (let i=0; i<gr.length; i++) {
    for (let j=0; j<gr[i].length; j++) {
      for (let k=0; k<gr[i][j].length; k++) {
        for (let l=0; l<gr[i][j][k].length; l++) {
          s += "" + i.toString() + "," + j.toString() + "," + k.toString() + "," + gr[i][j][k][l].toString() + "\n";
        }
      }
    }
  }

  fs.writeFileSync(fn, s);
}

function json_write(template, fn) {
  fs.writeFileSync(fn,JSON.stringify(template, null, 2));
}


init_template(_template);
build_tile_library(_template);

//_print(_template);
//process.exit();

csv_name(_template, "rgb_name.csv");
csv_rule(_template, "rgb_rule.csv");

// wfc ok
//
//csv_constraint(_template, "rg_constraint.csv", 3);

// wfc succeeds 34%
//
//csv_constraint(_template, "rg_constraint.csv", 6);

// wfc succeeds 
//
csv_constraint(_template, "rgb_constraint.csv", 8);

json_write(_template, "rgb.json");

//console.log(JSON.stringify(_template, null, 2));

