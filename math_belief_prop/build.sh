#!/bin/bash

#  lib/libsvd.a \

g++ -g -I../libmin/include/ \
  -D__linux__ \
  -DLIBHELP_STATIC \
  -I /usr/include/eigen3 \
  ../libmin/src/camera3d.cpp \
  ../libmin/src/file_png.cpp \
  ../libmin/src/vec.cpp \
  ../libmin/src/quaternion.cpp \
  ../libmin/src/dataptr.cpp \
  ../libmin/src/mersenne.cpp \
  ../libmin/src/common_defs.cpp \
  ../libmin/src/string_helper.cpp \
  ../libmin/src/datax.cpp \
  -I ../libmin/mains \
  bp_helper.cpp \
  belief_propagation.cpp \
  belief_propagation_residue.cpp \
  tests_belief_propagation.cpp \
  main_belief_propagation.cpp \
  -lm \
  -o bpc.debug

exit
#  -I ../math_displace_mesh \
#  ../math_displace_mesh/mesh.cpp \

g++ -O3 -I../libmin/include/ \
  -D__linux__ \
  -DLIBHELP_STATIC \
  -I /usr/include/eigen3 \
  ../libmin/src/camera3d.cpp \
  ../libmin/src/file_png.cpp \
  ../libmin/src/vec.cpp \
  ../libmin/src/quaternion.cpp \
  ../libmin/src/dataptr.cpp \
  ../libmin/src/mersenne.cpp \
  ../libmin/src/common_defs.cpp \
  ../libmin/src/string_helper.cpp \
  ../libmin/src/datax.cpp \
  -I ../libmin/mains \
  bp_helper.cpp \
  belief_propagation.cpp \
  belief_propagation_residue.cpp \
  tests_belief_propagation.cpp \
  main_belief_propagation.cpp \
  -lm \
  -o bpc

#  -I ../math_displace_mesh \
#  ../math_displace_mesh/mesh.cpp \


