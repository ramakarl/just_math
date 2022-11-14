#!/bin/bash

# non-trivial run with constrained tileset

./bpc -e 0.00001 \
  -I 100000  \
  -R ./rgb_rule.csv \
  -N ./rgb_name.csv \
  -C ./rgb_constraint.csv \
  -X 11 -Y 11 -Z 3 \
  -S 0 | tee rgb.bp.11x11x3S0.log
