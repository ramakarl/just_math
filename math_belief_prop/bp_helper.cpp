//--------------------------------------------------------------------------------
// JUST MATH:
// Belief Propagation - on a 3D grid domain
//
// Demonstration of Sum-Product Belief Propagation on a 3D spatial domain.
// Computes:
//     mu_{i,j}[b] = SUM f_{i,j}[a,b] g_i[a] PROD mu_{k,i}[b]
// The message function 'mu' is stored sparsely for neighboring cells in 3D, with size 6*R^3*B,
// where R is the grid resolution, B is the number of discrete values, and 6 is number of neighbors.
//
// To render the result, the belief is estimated at each vertex (voxel), and
// raytraced as a density volume where value probabilities are mapped to color.
//

//--------------------------------------------------------------------------------
// Copyright 2019-2022 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
//
// * Derivative works may append the above copyright notice but should not remove or modify earlier notices.
//
// MIT License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction, including without
// limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
// and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
// BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
// OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#include "bp_helper.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

int bp_restart ( BeliefPropagation& bpc )
{
    int ret;

    // dynamic restart
    // (not needed if this is first init)
    //
    ret = bpc.start ();

    bp_opt_t* op = bpc.get_opt();

    // apply dsl constraints
    if (op->constraint_cmd.size() > 0) {

        std::vector< int > dim;
        dim.push_back( op->X );
        dim.push_back( op->Y );
        dim.push_back( op->Z );
        std::vector< constraint_op_t > constraint_op_list;

        ret = parse_constraint_dsl ( constraint_op_list, op->constraint_cmd, dim, bpc.m_tile_name);
        if (ret < 0) {
            fprintf(stderr, "incorrect syntax when parsing constraint DSL\n");
            exit(-1);
        }

        ret = constrain_bp ( bpc, constraint_op_list);
        if (ret < 0) {
            fprintf(stderr, "constrain_bp failure\n");
            exit(-1);
        }
    }

    ret = bpc.RealizePre ();

    return ret;
}

// *NOTE*: Multirun can be used in place to bp_init.
// Assumes the caller has already populated cmd args.
//
int bp_multirun ( BeliefPropagation& bpc, int num_runs, std::string outfile )
{
    int ret, runret;
    std::string csv;

    // start report file:
    // overwrite first
    FILE* fp = fopen ( outfile.c_str(), "w" );
    if ( fp==0 ) {
        printf ( "ERROR: Cannot open %s for output.\n", outfile.c_str() );
        exit(-7);
    }
    fprintf ( fp, "reset\n" );
    fclose (fp);

    // append
    fp = fopen ( outfile.c_str(), "w" );

    // write header
    fprintf ( fp, "run, iter, time(ms), constr, iter_resolv, total_resolv, verts, resolv%%, cur_step, max_step, maxdmu, eps, avemu, avedmu\n" );


    // platform-specific, find name & rule files
    std::string name_path, rule_path;
    #ifdef _WIN32
      getFileLocation ( bpc.op.name_fn, name_path );
      getFileLocation ( bpc.op.rule_fn, rule_path );
    #else
      name_path = bpc.op.name_fn;
      rule_path = bpc.op.rule_fn;
    #endif

    // initialize BP
    ret = bp_init_CSV ( bpc, bpc.op.X, bpc.op.Y, bpc.op.Z, name_path, rule_path );
    if (ret<0) {
        fprintf(stderr, "bpc error loading CSV\n");
        exit(-1);
    }

    // start multiple runs
    bpc.op.max_run = num_runs;

    for (int run=0; run < num_runs; run++) {

        bpc.op.cur_run = run;

        // restart
        bp_restart ( bpc );

        // run iterations & steps
        runret = 1;
        while ( runret == 1 ) {

            ret = bpc.RealizeStep ();

            if (ret == 0 || ret == -2) {
                // finish this iteration
                ret = bpc.RealizePost();

                if ( ret > 0) {
                    // iteration complete (all steps), start new iteration
                    bpc.RealizePre();
                    runret = 1;

                    // append csv iteration
                    fprintf ( fp, "%s\n", bpc.getStatCSV().c_str() );  fflush ( fp );

                } else if (ret == 0) {
                    // this run DONE!
                    runret = 0;

                    // append csv run done
                    fprintf ( fp, "%s\n", bpc.getStatCSV( 1 ).c_str() );  fflush ( fp );
                    fprintf ( fp, "%s\n", bpc.getStatCSV( 2 ).c_str() );  fflush ( fp );

                    // write json output
                    write_tiled_json( bpc );
                }

            } else {
                // error condition
                switch (ret) {
                case -1: printf ( "bpc chooseMaxBelief error.\n" ); break;
                case -2: printf ( "bpc tileIdxCollapse error.\n" ); break;
                case -3: printf ( "bpc cellConstraintPropagate error.\n" ); break;
                };
                // ignore error. can set runret=ret if you want to stop on error
                runret = 1;
            }
        }

    }

    fclose ( fp );

    return 0;
}

// Experiments
// - experiments consist of multiple runs over a change in state variables
// - the state variables are stored in bpc.expr struct
// - set the desired min/max state variables prior to calling this func
//
int bp_experiments ( BeliefPropagation& bpc, std::string outfile )
{
    int ret, runret;
    std::string csv;

    // start report file:
    // overwrite first
    FILE* fp = fopen ( outfile.c_str(), "w" );   
    if ( fp==0 ) {
        printf ( "ERROR: Cannot open %s for output.\n", outfile.c_str() );
        exit(-7);
    }
    fprintf ( fp, "reset\n" );
    fclose (fp);

    // append
    fp = fopen ( outfile.c_str(), "w" ); 

    // write header
    fprintf ( fp, "gridx, gridy, gridz, tiles, max_step, eps, step_rate, # runs, success, %%, fail_constr, total time, ave time(ms), start seed\n" );

    // platform-specific, find name & rule files
    #ifdef _WIN32
        std::string name_path, rule_path;
        getFileLocation ( bpc.op.name_fn, name_path );
        getFileLocation ( bpc.op.rule_fn, rule_path );
    #endif

    int num_experiments = bpc.expr.num_expr;
    int num_runs = bpc.expr.num_run;
    
    Vector3DF dgrid = Vector3DF(bpc.expr.grid_max - bpc.expr.grid_min) / num_experiments;
    float dmstep = float(bpc.expr.maxstep_max - bpc.expr.maxstep_min) / num_experiments;
    float dsteprate = float(bpc.expr.steprate_max - bpc.expr.steprate_min) / num_experiments;
    float deps = float(bpc.expr.eps_max - bpc.expr.eps_min) / num_experiments;    

    Vector3DF grid = bpc.expr.grid_min;
    float maxstep = bpc.expr.maxstep_min;
    float steprate = bpc.expr.steprate_min;
    float eps = bpc.expr.eps_min;
    
    for (int e=0; e < num_experiments; e++) {

        // reset bp
        bpc.reset ();
        
        // setup BP options
        bpc.op.X = grid.x;
        bpc.op.Y = grid.y;
        bpc.op.Z = grid.z;
        bpc.op.max_step = int(maxstep);
        bpc.op.eps_converge = eps;
        bpc.op.step_rate = steprate;

        // initialize BP       
        ret = bp_init_CSV ( bpc, bpc.op.X, bpc.op.Y, bpc.op.Z, name_path, rule_path );
        if (ret<0) {
            fprintf(stderr, "bpc error loading CSV\n");
            exit(-1);
        }        

        // start multiple runs        
        bpc.op.max_run = num_runs;

        int success=0;
        float fail_constr=0;
        float total_time=0;

        for (int run=0; run < bpc.op.max_run; run++) {

            bpc.op.cur_run = run;

            // restart 
            bp_restart ( bpc );

            // run iterations & steps
            runret = 1;
            while ( runret >= 1 ) {            
                ret = bpc.RealizeStep ();
                if (ret == 0 || ret == -2) {             
                    // finish this iteration
                    ret = bpc.RealizePost();                                
                    if ( ret > 0) {
                        // iteration complete (all steps), start new iteration
                        bpc.RealizePre();                    
                        runret = 1;
                    } else if (ret == 0) {
                        // done!
                        runret = 0;                                            
                    } 
                } else {                    
                    // error, ignore and continue (1). can set runret=ret if you want to stop on error. -1=maxbelief err, -2=tileidx err, -3=cellConstrProp err
                    runret = 1;
                }
            }       
            // run completion            
            if (bpc.st.constraints==0) {
                success++;
            } else {
                fail_constr += bpc.st.constraints;
            }
            total_time += bpc.st.elapsed_time;

        } 
        // experiment completion

        float ave_time = total_time / bpc.op.max_run;     
        fail_constr /= bpc.op.max_run;

        printf ( "GRID: %d,%d,%d, tiles:%d, maxstep: %d, eps:%f, srate:%f, runs:%d, success: %d (%4.1f%%), failconstr: %f, time: %f, avetime: %f, sseed: %d\n", 
                        bpc.op.X, bpc.op.Y, bpc.op.Z, bpc.m_num_values, bpc.op.max_step, bpc.op.eps_converge, bpc.op.step_rate,
                        bpc.op.max_run, success, 100*float(success)/bpc.op.max_run, fail_constr, total_time, ave_time, bpc.op.seed );
        
        fprintf ( fp, "%d,%d,%d, %d, %d, %f, %f, %d, %d, %1.5f, %f, %f, %f, %d\n", 
                        bpc.op.X, bpc.op.Y, bpc.op.Z, bpc.m_num_values, bpc.op.max_step, bpc.op.eps_converge, bpc.op.step_rate,
                        bpc.op.max_run, success, float(success)/bpc.op.max_run, fail_constr, total_time, ave_time, bpc.op.seed );

        // proper flush
        fclose ( fp );
        fp = fopen ( outfile.c_str(), "a" ); 

        // next experiment
        grid += dgrid;
        maxstep += dmstep;
        steprate += dsteprate;
        eps += deps;
    }
    fclose ( fp );

    return 0;
}

int bp_init_CSV( BeliefPropagation &bp, int rx, int ry, int rz, std::string name_path, std::string rule_path ) {

  int ret;

  std::vector< std::string >            name_list;
  std::vector< float >                  weight_list;
  std::vector< std::vector< float > >   rule_list;

  bp.op.X = rx;
  bp.op.Y = ry;
  bp.op.Z = rz;

  // we dont use name_fn/rule_fn from opt because name_path/rule_path
  // passed in are platform-specific resolved absolute paths

  ret = _read_name_csv( name_path, name_list, weight_list );
  if (ret<0) {
    fprintf(stderr, "error loading name CSV\n");
    return -1;
  }

  ret =_read_rule_csv( rule_path, rule_list );
  if (ret<0) {
    fprintf(stderr, "error loading rule CSV\n");
    return -1;
  }

  ret = bp.init( bp.op.X, bp.op.Y, bp.op.Z, name_list, weight_list, rule_list );

  return ret;
}

//------------------------------- secondary helpers

int _read_line(FILE *fp, std::string &line) {
  int ch=0, count=0;

  while (!feof(fp)) {
    ch = fgetc(fp);
    if (ch == '\n') { break; }
    if (ch == EOF) { break; }
    line += (char)ch;
    count++;
  }
  return count;
}

int _read_name_csv(std::string &fn, std::vector<std::string> &name, std::vector< float > &weight) {
  int i, idx;
  FILE *fp;
  float w = 1.0;

  std::string line, tok, _s;
  std::vector<std::string> toks;

  int dir_idx;

  std::string _color;

  fp = fopen(fn.c_str(), "r");
  if (!fp) { return -1; }

  while (!feof(fp)) {
    line.clear();
    _read_line(fp, line);

    if (line.size()==0) { continue; }
    if (line[0] == '#') { continue; }
    if (line[0] == ' ') { continue; }

    toks.clear();
    tok.clear();
    for (i=0; i<line.size(); i++) {
      if (line[i]==',') {
        toks.push_back(tok);
        tok.clear();
        continue;
      }
      tok += line[i];
    }
    toks.push_back(tok);

    //if (toks.size() != 2) { continue; }
    if (toks.size() < 2) { continue; }

    idx = atoi(toks[0].c_str());
    if (idx <= name.size()) {
      for (i=name.size(); i<=idx; i++) {
        _s.clear();
        name.push_back(_s);
        weight.push_back(0.0);
      }
    }
    name[idx] = toks[1];

    w = 1.0;
    if (toks.size() > 2) { w = atof(toks[2].c_str()); }
    weight[idx] = w;

    if (toks.size() > 3) { dir_idx  = atoi(toks[3].c_str()); }
    if (toks.size() > 4) { _color = toks[4]; }
  }

  fclose(fp);

  w = 0.0;
  for (i=0; i<weight.size(); i++) { w += weight[i]; }
  for (i=0; i<weight.size(); i++) { weight[i] /= w; }

  return 0;
}


int _read_rule_csv(std::string &fn, std::vector< std::vector<float> > &rule) {
  int i;
  float _weight;
  FILE *fp;
  std::string line, tok;
  std::vector<std::string> toks;
  std::vector<float> v;

  float _tile_src, _tile_dst;

  fp = fopen(fn.c_str(), "r");
  if (!fp) { return -1; }

  while (!feof(fp)) {
    line.clear();
    _read_line(fp, line);

    if (line.size()==0) { continue; }
    if (line[0] == '#') { continue; }
    if (line[0] == ' ') { continue; }

    toks.clear();
    tok.clear();
    for (i=0; i<line.size(); i++) {
      if (line[i]==',') {
        toks.push_back(tok);
        tok.clear();
        continue;
      }
      tok += line[i];
    }
    toks.push_back(tok);

    if ((toks.size() < 3) || (toks.size() > 4)) { continue; }

    _tile_src = atof(toks[0].c_str());
    _tile_dst = atof(toks[1].c_str());
    _weight = 1;

    if ((toks.size() >= 4) &&
        (toks[3].size() != 0) &&
        (toks[3][0] != 'u')) {
      _weight = atof(toks[3].c_str());
    }

    // direction wild card
    //
    if ((toks[2].size()==0) ||
        (toks[2][0] == '*')) {
      v.clear();
      v.push_back(0.0);
      v.push_back(0.0);
      v.push_back(0.0);
      v.push_back(0.0);
      for (i=0; i<6; i++) {
        v[0] = _tile_src;
        v[1] = _tile_dst;
        v[2] = (float)i;
        v[3] = _weight ;
        rule.push_back(v);
      }
    }

    // explicit entry
    //
    else {
      v.clear();
      v.push_back(_tile_src);
      v.push_back(_tile_dst);
      v.push_back(atof(toks[2].c_str()));
      v.push_back(_weight);
      rule.push_back(v);
    }

  }

  fclose(fp);

  return 0;
}



//---


// Split string on separator
//   e.g. "object:car, other".. left='object:car', right='other'
bool strSplit_ ( std::string str, std::string sep, std::string& left, std::string& right ) {
  std::string result;
  size_t f1, f2;

  //f1 = str.find_first_not_of ( sep );
  //if ( f1 == std::string::npos ) f1 = 0;
  f1 = 0;
  f2 = str.find_first_of ( sep, f1 );
  if ( f2 != std::string::npos) {
    left = str.substr ( f1, f2-f1 );
    right = str.substr ( f2+1 );
    return true;
  }
  left = "";
  right = str;
  return false;
}


int parse_frange(std::vector<float> &range, std::string &s) {
  int err_code = 0, iret;
  float val;

  std::string cur_s = s;
  std::string innard;
  std::string csep = ":";
  std::string l_innard, r_innard;
  size_t pos=0;

  bool r;

  range.clear();
  range.push_back(0);
  range.push_back(0);

  if (s.size()==0) { return -1; }

  r = strSplit_( s, csep, l_innard, r_innard );
  if (!r) {
    iret = strToF(r_innard, val);
    if (iret<0) { return -1; }
    range[0] = val;
    range[1] = val;
    return 0;
  }

  if (l_innard.size() > 0) {
    iret = strToF(l_innard, val);
    if (iret < 0) { return -1; }
    range[0] = val;
  }

  if (r_innard.size() > 0) {
    iret = strToF(r_innard, val);
    if (iret < 0) { return -1; }
    range[1] = val;
  }

  return err_code;
}

int parse_range(std::vector<int> &range, std::string &s, std::vector<int> &dim) {
  int err_code = 0;
  int val, iret;

  std::string cur_s = s;
  std::string innard;
  std::string csep = ":";
  std::string l_innard, r_innard;
  size_t pos=0;

  bool r;

  range.clear();
  range.push_back(0);
  range.push_back(0);
  if (dim.size() > 0) { range[1] = dim[0]; }

  if (s.size()==0) { return -1; }

  r = strSplit_( s, csep, l_innard, r_innard );
  if (!r) {

    iret = strToI(r_innard, val);
    if (iret<0) { return -1; }
    if (val<0) {
      if (dim.size() > 0) {
        val = dim[0] + val;
      }
    }
    range[0] = val;
    range[1] = range[0]+1;
    return 0;
  }

  if (l_innard.size() > 0) {
    iret = strToI(l_innard, val);

    if (iret < 0) { return -1; }
    if (val < 0) {
      if (dim.size() > 0) {
        val = dim[0] + val;
      }
    }
    range[0] = val;
  }

  if (r_innard.size() > 0) {
    iret = strToI(r_innard, val);

    if (iret < 0) { return -1; }
    if (val < 0) {
      if (dim.size() > 0) {
        val = dim[0] + val;
      }
    }
    range[1] = val+1;
  }

  return err_code;
}

int parse_bracket_range(std::vector<int> &range, std::string &s, std::vector<int> &dim) {
  int dim_idx=0;
  int err_code = 0;
  int val, iret;

  std::string cur_s = s;
  std::string innard;
  std::string lsep = "[", rsep = "]", csep = ":";
  std::string l_innard, r_innard;
  size_t pos=0;

  bool r;

  range.clear();

  for (dim_idx=0; dim_idx<dim.size(); dim_idx++) {
    range.push_back(0);
    range.push_back(dim[dim_idx]);
  }

  if (s.size()==0) { return 0; }

  for (dim_idx=0; dim_idx<dim.size(); dim_idx++) {

    r = strGet( cur_s, lsep, rsep, innard, pos );
    if (!r) {
      err_code = -1-dim_idx;
      break;
    }

    // bad parse or not enclosed in brackets
    //
    if (innard.size()==0) { err_code = -4; break; }
    if ((innard[0] != lsep[0]) ||
        (innard[ innard.size()-1 ] != rsep[0])) {
      err_code = -5;
      break;
    }

    // lop off first range
    //
    cur_s  = cur_s.substr(pos + innard.size());

    if (innard.size()==2) { continue; }

    // lop off left and right separator
    //
    innard = innard.substr(1, innard.size()-2);

    r = strSplit_( innard, csep, l_innard, r_innard );
    if (!r) {
      iret = strToI(r_innard, val);
      if (iret<0) { return -1; }
      if (val<0) { val = dim[dim_idx] + val; }
      range[2*dim_idx] = val;
      range[2*dim_idx+1] = range[2*dim_idx]+1;
      continue;
    }

    if (l_innard.size() > 0) {
      iret = strToI(l_innard, val);
      if (iret < 0) { return -1; }
      if (val < 0) { val = dim[dim_idx] + val; }
      range[2*dim_idx] = val;
    }

    if (r_innard.size() > 0) {
      iret = strToI(r_innard, val);
      if (iret < 0) { return -1; }
      if (val < 0) { val = dim[dim_idx] + val; }
      range[2*dim_idx+1] = val;
    }


  }

  return err_code;
}

int parse_constraint_dsl(std::vector< constraint_op_t > &op_list, std::string &s, std::vector< int > dim, std::vector< std::string > name) {
  int i, n, r;
  std::vector< std::string > raw_tok;
  std::string ws_sep = " \n\t";
  std::vector< std::string > tok;
  std::vector< int > tiledim;

  constraint_op_t op;

  std::string srange;

  //tiledim.push_back(0);
  tiledim.push_back(name.size());

  op_list.clear();

  n = strSplitMultiple( s, ws_sep, raw_tok );

  for (i=0; i<n; i++) {
    if (raw_tok[i].size() > 0) {
      tok.push_back( raw_tok[i] );
    }
  }

  if ((tok.size()%2) != 0) { return -1; }

  for (i=0; i<tok.size(); i+=2) {

    op.dim_range.clear();
    op.tile_range.clear();

    srange = tok[i].substr(1);

    op.op = tok[i][0];
    r = parse_bracket_range(op.dim_range, srange, dim);
    if (r<0) { return -1; }

    r = parse_range(op.tile_range, tok[i+1], tiledim);
    if (r<0) { return -2; }

    op_list.push_back(op);
  }

  return 0;
}

void debug_constraint_op_list(std::vector< constraint_op_t > &op_list) {
  int i, j;
  for (i=0; i<op_list.size(); i++) {
    printf("op_list[%i] op:%c dim_range", i, op_list[i].op);
    for (j=0; j<op_list[i].dim_range.size(); j+=2) {
      printf("[%i:%i]",
          (int)op_list[i].dim_range[j],
          (int)op_list[i].dim_range[j+1]);
    }
    printf(" tile_range");
    for (j=0; j<op_list[i].tile_range.size(); j+=2) {
      printf("(%i:%i)",
          (int)op_list[i].tile_range[j],
          (int)op_list[i].tile_range[j+1]);
    }
    printf("\n");
  }
}

int constrain_bp(BeliefPropagation &bp, std::vector< constraint_op_t > &op_list) {
  int ret=0;
  int op_idx, i, j, k, n;
  int x,y,z,t;
  int64_t pos;

  std::vector<int32_t> v;

  // debug_constraint_op_list(op_list);

  bp.m_note_n[0] = 0;
  bp.m_note_n[1] = 0;

  for (op_idx=0; op_idx<op_list.size(); op_idx++) {

    // discard
    //
    if (op_list[op_idx].op == 'd') {

      v.clear();
      for (t=op_list[op_idx].tile_range[0]; t<op_list[op_idx].tile_range[1]; t++) {
        v.push_back(t);
      }

      for (x=op_list[op_idx].dim_range[0]; x<op_list[op_idx].dim_range[1]; x++) {
        for (y=op_list[op_idx].dim_range[2]; y<op_list[op_idx].dim_range[3]; y++) {
          for (z=op_list[op_idx].dim_range[4]; z<op_list[op_idx].dim_range[5]; z++) {
            pos = bp.getVertex(x,y,z);
            ret = bp.filterDiscard(pos, v);
            if (ret < 0) { return ret; }

            bp.cellFillVisited (pos, bp.m_note_plane);
          }
        }
      }

    }

    // force (only)
    //
    else if (op_list[op_idx].op == 'f') {

      v.clear();
      for (t=op_list[op_idx].tile_range[0]; t<op_list[op_idx].tile_range[1]; t++) {
        v.push_back(t);
      }

      for (x=op_list[op_idx].dim_range[0]; x<op_list[op_idx].dim_range[1]; x++) {
        for (y=op_list[op_idx].dim_range[2]; y<op_list[op_idx].dim_range[3]; y++) {
          for (z=op_list[op_idx].dim_range[4]; z<op_list[op_idx].dim_range[5]; z++) {
            pos = bp.getVertex(x,y,z);
            ret = bp.filterKeep(pos, v);
            if (ret < 0) { return ret; }

            bp.cellFillVisited (pos, bp.m_note_plane);
          }
        }
      }

    }

    else if (op_list[op_idx].op == 'a') {
      // sorry
    }

    else {
      return -1;
    }

  }

  bp.unfillVisited (bp.m_note_plane);
  ret = bp.cellConstraintPropagate();
  if (ret == 0) { bp.NormalizeMU(); }

  return ret;
}

//------------//
//       _ _  //
//   ___| (_) //
//  / __| | | //
// | (__| | | //
//  \___|_|_| //
//            //
//------------//

//void stl_print(FILE *fp, std::vector< float > &tri, float dx=0.0, float dy=0.0, float dz=0.0);
void stl_print(FILE *, std::vector< float > &, float, float, float);

int write_bp_stl (BeliefPropagation& bp, std::vector< std::vector< float > > tri_lib) {
  FILE *fp=stdout;

  int i, j, k, n;
  int ix, iy, iz;

  float stride_x = 1.0,
        stride_y = 1.0,
        stride_z = 1.0;

  float cx=0.0, cy=0.0, cz=0.0,
        dx, dy, dz,
        nx, ny, nz;
  int64_t pos;
  int32_t tile_id;


  fp = fopen( bp.op.outstl_fn.c_str(), "w");
  if (!fp) { return -1; }

  for (ix=0; ix<bp.m_res.x; ix++) {
    for (iy=0; iy<bp.m_res.y; iy++) {
      for (iz=0; iz<bp.m_res.z; iz++) {
        pos = bp.getVertex(ix, iy, iz);

        tile_id = bp.getValI ( BUF_TILE_IDX, 0, pos );

        dx = (float)ix*stride_x + cx;
        dy = (float)iy*stride_y + cy;
        dz = (float)iz*stride_z + cz;

        if (tile_id >= tri_lib.size()) {
          fprintf(stderr, "ERROR: tile_id %i, exceeds tri (%i)\n",
              (int)tile_id, (int)tri_lib.size());
          continue;
        }

        stl_print(fp, tri_lib[tile_id], dx, dy, dz);

      }
    }
  }

  fclose(fp);

  return 0;
}

int write_tiled_json ( BeliefPropagation & bpc) {
  FILE *fp;
  int i, j, n, tileset_size;
  int64_t vtx;

  int sy, ey_inc;

  // set filename
  char fname[1024];
  sprintf (fname, "%s%04d.json", bpc.op.tilemap_fn.c_str(), bpc.op.cur_run );

  // get BP options
  bp_opt_t* op = bpc.get_opt();

  int tilecount = (int)bpc.m_tile_name.size();
  tilecount--;

  //opt.tileset_width = ceil( sqrt( ((double)bpc.m_tile_name.size()) - 1.0 ) );
  op->tileset_width = ceil( sqrt( (double) tilecount ) );
  op->tileset_height = op->tileset_width;

  op->tileset_width *= op->tileset_stride_x;
  op->tileset_height *= op->tileset_stride_y;


  // open file for write
  fp = fopen( fname, "w");

  if (!fp) {
      printf("ERROR: Failed to write (%s)\n", fname );
      return -1;
  } else {
      if (op->verbose >= 2) printf("Writing tilemap (%s)\n", fname );
  }

  fprintf(fp, "{\n");
  fprintf(fp, "  \"backgroundcolor\":\"#ffffff\",\n");
  fprintf(fp, "  \"height\": %i,\n", (int) bpc.m_res.y);
  fprintf(fp, "  \"width\": %i,\n", (int) bpc.m_res.x);
  fprintf(fp, "  \"layers\": [{\n");

  fprintf(fp, "    \"data\": [");

  // tiled expects y to increment in the negative direction
  // so we need to reverse the y direction when exporting
  //

  int tile;

  if (op->tiled_reverse_y) {

    for (i=(int) (bpc.m_res.y-1); i>=0; i--) {
      for (j=0; j<(int) bpc.m_res.x; j++) {

        vtx = bpc.getVertex(j, i, 0);

        //tile = bpc.getMaxBeliefTile ( vtx );
         tile = bpc.getValI( BUF_TILE_IDX, 0, vtx );

        fprintf(fp, " %i", tile );
        if ((i==0) && (j==(bpc.m_res.x-1))) { fprintf(fp, "%s",  ""); }
        else                                { fprintf(fp, "%s", ","); }
      }
      fprintf(fp, "\n  ");
    }

  }
  else {
    for (i=0; i<(int) (bpc.m_res.y); i++) {
      for (j=0; j<(int) bpc.m_res.x; j++) {
         vtx = bpc.getVertex(j, i, 0);

         //tile = bpc.getMaxBeliefTile ( vtx );
          tile = bpc.getValI( BUF_TILE_IDX, 0, vtx );

         fprintf(fp, " %i", tile );

         if ((i==(bpc.m_res.y-1)) && (j==(bpc.m_res.x-1))) { fprintf(fp, "%s",  ""); }
         else                                { fprintf(fp, "%s", ","); }
      }
      fprintf(fp, "\n  ");
    }

  }

  /*
  n = bpc.m_num_verts;
  for (i=0; i<n; i++) {
    if ((i%(int)bpc.m_res.x)==0) {
      fprintf(fp, "\n   ");
    }
    fprintf(fp, " %i%s", bpc.getVali( BUF_TILE_IDX, i, 0 ), (i<(n-1)) ? "," : "" );
  }
  */

  fprintf(fp, "\n    ],\n");
  fprintf(fp, "    \"name\":\"main\",\n");
  fprintf(fp, "    \"opacity\":1,\n");
  fprintf(fp, "    \"type\":\"tilelayer\",\n");
  fprintf(fp, "    \"visible\":true,\n");
  fprintf(fp, "    \"width\": %i,\n", (int)bpc.m_res.x);
  fprintf(fp, "    \"height\": %i,\n", (int)bpc.m_res.y);
  fprintf(fp, "    \"x\":0,\n");
  fprintf(fp, "    \"y\":0\n");

  fprintf(fp, "  }\n");

  fprintf(fp, "  ],\n");
  fprintf(fp, "  \"nextobjectid\": %i,\n", 1);
  fprintf(fp, "  \"orientation\": \"%s\",\n", "orthogonal");
  fprintf(fp, "  \"properties\": [ ],\n");
  fprintf(fp, "  \"renderorder\": \"%s\",\n", "right-down");
  fprintf(fp, "  \"tileheight\": %i,\n", (int) op->tileset_stride_y);
  fprintf(fp, "  \"tilewidth\": %i,\n", (int) op->tileset_stride_x);
  fprintf(fp, "  \"tilesets\": [{\n");

  fprintf(fp, "    \"firstgid\": %i,\n", 1);
  fprintf(fp, "    \"columns\": %i,\n", (int) bpc.m_res.x);
  fprintf(fp, "    \"name\": \"%s\",\n", "tileset");
  fprintf(fp, "    \"image\": \"%s\",\n", op->tileset_fn.c_str());
  fprintf(fp, "    \"imageheight\": %i,\n", (int) op->tileset_height);
  fprintf(fp, "    \"imagewidth\": %i,\n", (int) op->tileset_width);
  fprintf(fp, "    \"margin\": %i,\n", (int) op->tileset_margin);
  fprintf(fp, "    \"spacing\": %i,\n", (int) op->tileset_spacing);
  //fprintf(fp, "    \"tilecount\": %i,\n", (int)(bpc.m_tile_name.size()-1));
  fprintf(fp, "    \"tilecount\": %i,\n", tilecount);
  fprintf(fp, "    \"tileheight\": %i,\n", (int) op->tileset_stride_y);
  fprintf(fp, "    \"tilewidth\": %i\n", (int) op->tileset_stride_x);

  fprintf(fp, "  }],\n");
  fprintf(fp, "  \"version\": %i\n", 1);
  fprintf(fp, "}\n");

  fclose(fp);

  return 0;
}
int grid_obj2stl_out(std::string ofn, BeliefPropagation &bp, std::vector< std::vector< float > > tri) {
  int i, j, k, n;

  float stride_x = 1.0,
        stride_y = 1.0,
        stride_z = 1.0;

  float cx = 0.0,
        cy = 0.0,
        cz = 0.0;

  float dx = 1.0,
        dy = 1.0,
        dz = 1.0;

  float nx, ny, nz;

  int ix, iy, iz;
  int64_t pos;
  int32_t tile_id;

  FILE *fp;

  fp = fopen(ofn.c_str(), "w");
  if (!fp) { return -1; }

  for (ix=0; ix<bp.m_res.x; ix++) {
    for (iy=0; iy<bp.m_res.y; iy++) {
      for (iz=0; iz<bp.m_res.z; iz++) {
        pos = bp.getVertex(ix, iy, iz);

        tile_id = bp.getValI( BUF_TILE_IDX, 0, pos );

        dx = (float)ix*stride_x + cx;
        dy = (float)iy*stride_y + cy;
        dz = (float)iz*stride_z + cz;

        if (tile_id >= tri.size()) {
          fprintf(stderr, "ERROR: tile_id %i, exceeds tri (%i)\n",
              (int)tile_id, (int)tri.size());
          continue;
        }

        fprintf(fp, "solid\n");
        for (i=0; i<tri[tile_id].size(); i+=9) {
          fprintf(fp, "  facet normal %f %f %f\n",
              (float)nx, (float)ny, (float)nz);

          for (j=0; j<3; j++) {
            fprintf(fp, "    vertex %f %f %f\n",
              (float)tri[tile_id][i + 3*j + 0],
              (float)tri[tile_id][i + 3*j + 1],
              (float)tri[tile_id][i + 3*j + 2]);
          }

          fprintf(fp, "  endfacet\n");
        }
        fprintf(fp, "endsolid\n");

      }
    }
  }

  fclose(fp);

  return 0;
}

int load_obj2tri(std::string inputfile, std::vector< float > &tri) {

  tri.clear();

  //std::string inputfile = "./examples/.data/s000.obj";
  tinyobj::ObjReaderConfig reader_config;
  reader_config.mtl_search_path = "./"; // Path to material files

  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(inputfile, reader_config)) {
    if (!reader.Error().empty()) {
        std::cerr << "TinyObjReader[E]: " << reader.Error();
    }
    return -1;
  }

  if (!reader.Warning().empty()) {
    std::cout << "TinyObjReader[W]: " << reader.Warning();
  }

  auto& attrib = reader.GetAttrib();
  auto& shapes = reader.GetShapes();
  auto& materials = reader.GetMaterials();
  // Loop over shapes
  for (size_t s = 0; s < shapes.size(); s++) {

    //printf("#shape %i\n", (int)s);

    // Loop over faces(polygon)
    size_t index_offset = 0;
    for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
      size_t fv = size_t(shapes[s].mesh.num_face_vertices[f]);

      //printf("# face %i\n", (int)f);

      // Loop over vertices in the face.
      for (size_t v = 0; v < fv; v++) {

        //printf("#  vertex %i\n", (int)v);

        // access to vertex
        tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
        tinyobj::real_t vx = attrib.vertices[3*size_t(idx.vertex_index)+0];
        tinyobj::real_t vy = attrib.vertices[3*size_t(idx.vertex_index)+1];
        tinyobj::real_t vz = attrib.vertices[3*size_t(idx.vertex_index)+2];

        tri.push_back(vx);
        tri.push_back(vy);
        tri.push_back(vz);

        // Check if `normal_index` is zero or positive. negative = no normal data
        if (idx.normal_index >= 0) {

          tinyobj::real_t nx = attrib.normals[3*size_t(idx.normal_index)+0];
          tinyobj::real_t ny = attrib.normals[3*size_t(idx.normal_index)+1];
          tinyobj::real_t nz = attrib.normals[3*size_t(idx.normal_index)+2];

          printf("!! normal: %f %f %f\n", nx, ny, nz);
        }

        // Check if `texcoord_index` is zero or positive. negative = no texcoord data
        if (idx.texcoord_index >= 0) {
          tinyobj::real_t tx = attrib.texcoords[2*size_t(idx.texcoord_index)+0];
          tinyobj::real_t ty = attrib.texcoords[2*size_t(idx.texcoord_index)+1];
        }

        // Optional: vertex colors
        // tinyobj::real_t red   = attrib.colors[3*size_t(idx.vertex_index)+0];
        // tinyobj::real_t green = attrib.colors[3*size_t(idx.vertex_index)+1];
        // tinyobj::real_t blue  = attrib.colors[3*size_t(idx.vertex_index)+2];
      }
      index_offset += fv;

      // per-face material
      shapes[s].mesh.material_ids[f];
    }
  }

  return 0;

  /*
  tinyobj::attrib_t ax;
  std::vector< tinyobj::shape_t > sx;
  std::vector< tinyobj::material_t > mx;

  ax = attrib;
  sx = shapes;
  mx = materials;

  bool coordTransform = false;
  bool ignoreMaterial = true;

  //tinyobj::WriteObj( "./akok.0.obj", attrib, shapes, materials, coordTransform, ignoreMaterial);
  tinyobj::WriteObj( "./akok.0.obj", ax, sx, mx, coordTransform, ignoreMaterial);

  for (size_t i=0; i<ax.vertices.size(); i+=3) {
    ax.vertices[i+0] += 2.0;
    ax.vertices[i+1] += 0.0;
    ax.vertices[i+2] += 0.0;
  }

  //tinyobj::WriteObj( "./akok.1.obj", attrib, shapes, materials, coordTransform, ignoreMaterial);
  tinyobj::WriteObj( "./akok.1.obj", ax, sx, mx, coordTransform, ignoreMaterial);

  exit(-1);
  return;
  */

  /*
  bool br;
  MeshX mm;

  br = mm.LoadObj("./examples/.data/s000.obj", 1.0);
  if (br) { printf("load successful\n"); }
  else { printf("load failed\n"); }

  exit(-1);
  */

}

void stl_print(FILE *fp, std::vector< float > &tri, float dx=0.0, float dy=0.0, float dz=0.0) {
  int i, j;
  float nx=0.0, ny=0.0, nz=0.0;

  fprintf(fp, "solid\n");
  for (i=0; i<tri.size(); i+=9) {
    fprintf(fp, "  facet normal %f %f %f\n",
        (float)nx, (float)ny, (float)nz);

    fprintf(fp, "    outer loop\n");
    for (j=0; j<3; j++) {
      fprintf(fp, "      vertex %f %f %f\n",
        (float)tri[i + 3*j + 0] + dx,
        (float)tri[i + 3*j + 1] + dy,
        (float)tri[i + 3*j + 2] + dz);
    }

    fprintf(fp, "    endloop\n");
    fprintf(fp, "  endfacet\n");
  }
  fprintf(fp, "endsolid\n");
}

int load_obj_stl_lib(std::string fn, std::vector< std::vector< float > > &tris) {
  int i, j, k, ret;
  std::vector< std::string > obj_fns;
  std::vector< float > w;
  std::vector< float > tri;

  ret = _read_name_csv(fn, obj_fns, w);
  if (ret<0) { return ret; }

  for (i=0; i<obj_fns.size(); i++) {
    ret = load_obj2tri(obj_fns[i], tri);
    if (ret<0) { return ret; }
    tris.push_back(tri);
  }

  return 0;
}



