#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

#include <vector>
#include <string>

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

int read_name_csv(std::string &fn, std::vector<std::string> &name) {
  int i, idx;
  FILE *fp;
  std::string line, tok, _s;
  std::vector<std::string> toks;

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

    if (toks.size() != 2) { continue; }

    idx = atoi(toks[0].c_str());
    if (idx <= name.size()) {
      for (i=name.size(); i<=idx; i++) {
        _s.clear();
        name.push_back(_s);
      }
    }
    name[idx] = toks[1];
  }

  fclose(fp);

  return 0;
}

int read_rule_csv(std::string &fn, std::vector< std::vector<float> > &rule) {
  int i;
  float val, _weight;
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

void pretty_print_rule_name(std::vector<std::string> &name, std::vector< std::vector<float> > &rule) {
  int i, tile_a, tile_b, dir_code;

  std::vector< std::string > dir_str;

  dir_str.push_back("+1:0:0");
  dir_str.push_back("-1:0:0");

  dir_str.push_back("0:+1:0");
  dir_str.push_back("0:-1:0");

  dir_str.push_back("0:0:+1");
  dir_str.push_back("0:0:-1");

  for (i=0; i<rule.size(); i++) {

    tile_a = (int)rule[i][0];
    tile_b = (int)rule[i][1];

    if ((tile_a<0) || (tile_a>=name.size())) { continue; }
    if ((tile_b<0) || (tile_b>=name.size())) { continue; }

    dir_code = (int)rule[i][2];
    if ((dir_code < 0) || (dir_code >= dir_str.size())) { continue; }

    printf("%s(%i) -(%s)-> %s(%i) : %f\n", name[tile_a].c_str(), tile_a, dir_str[dir_code].c_str(), name[tile_b].c_str(), tile_b, rule[i][3]);

  }
}

int main(int argc, char **argv) {
  int i;
  std::string fn_name, fn_rule, line;
  FILE *fp;
  std::vector<std::string> name_lookup;
  std::vector< std::vector<float> > rule_vec;

  //fn_name = "../examples/example0_name.csv";
  //fn_rule = "../examples/example0_rule.csv";

  fn_name = "../examples/stair_name.csv";
  fn_rule = "../examples/stair_rule.csv";

  read_name_csv(fn_name, name_lookup);
  read_rule_csv(fn_rule, rule_vec);

  for (i=0; i<name_lookup.size(); i++) {
    printf("%i %s\n", i, name_lookup[i].c_str());
  }
  printf("---\n");

  for (i=0; i<rule_vec.size(); i++) {
    printf("%i %f %f %f %f\n", i,
      rule_vec[i][0], rule_vec[i][1],
      rule_vec[i][2], rule_vec[i][3]);
  }
  printf("---\n");


  pretty_print_rule_name(name_lookup, rule_vec);

}
