#!/bin/bash

#  lib/libsvd.a \

g++ -g -I../libmin/include/ \
  -I /usr/include/eigen3 \
  ../libmin/src/camera3d.cpp \
  ../libmin/src/file_png.cpp \
  ../libmin/src/vec.cpp \
  ../libmin/src/quaternion.cpp \
  ../libmin/src/dataptr.cpp \
  ../libmin/src/mersenne.cpp \
  ../libmin/src/common_defs.cpp \
  belief_propagation.cpp \
  bpr.cpp \
  tests_belief_propagation.cpp \
  main_belief_propagation.cpp \
  -lm \
  -o bpc.debug

exit

g++ -O3 -I../libmin/include/ \
  -I /usr/include/eigen3 \
  ../libmin/src/camera3d.cpp \
  ../libmin/src/file_png.cpp \
  ../libmin/src/vec.cpp \
  ../libmin/src/quaternion.cpp \
  ../libmin/src/dataptr.cpp \
  ../libmin/src/mersenne.cpp \
  ../libmin/src/common_defs.cpp \
  belief_propagation.cpp \
  tests_belief_propagation.cpp \
  main_belief_propagation.cpp \
  -lm \
  -o bpc


