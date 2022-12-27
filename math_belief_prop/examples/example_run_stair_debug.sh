#!/bin/bash

# non-trivial run with constrained tileset

./bpc.debug -e 0.001 \
  -I 100000  \
  -R ./stair_rule.csv \
  -N ./stair_name.csv \
  -X 6 -Y 6 -Z 6 \
  -S 0 | tee stair.bp.6x6x6S0.log

exit

./bpc -e 0.00001 \
  -I 100000  \
  -R ./rgb_rule.csv \
  -N ./rgb_name.csv \
  -C ./rgb_constraint.csv \
  -X 11 -Y 11 -Z 3 \
  -r 512 -S 0 | tee rgb.bp.11x11x3S0.log
