#!/bin/bash

# non-trivial run with constrained tileset

#valgrind ./bpc.debug -e 0.00001 \
#  -I 100000  \
#  -R ./rgb_rule.csv \
#  -N ./rgb_name.csv \
#  -C ./rgb_constraint.csv \
#  -X 11 -Y 11 -Z 3 \
#  -r -S 0 | tee rgb.bp.11x11x3S0.log

./bpc.debug -e 0.00001 \
  -I 100000  \
  -R ./rgb_rule.csv \
  -N ./rgb_name.csv \
  -C ./rgb_constraint.csv \
  -X 11 -Y 11 -Z 3 \
  -r 512 -S 0 | tee rgb.bp.11x11x3S0.log
