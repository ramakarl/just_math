#!/bin/bash

g++ -g -I../libmin/include/ \
  ../libmin/src/camera3d.cpp \
  ../libmin/src/file_png.cpp \
  ../libmin/src/vec.cpp \
  ../libmin/src/quaternion.cpp \
  ../libmin/src/dataptr.cpp \
  ../libmin/src/mersenne.cpp \
  ../libmin/src/common_defs.cpp \
  belief_propagation.cpp \
  main_belief_propagation.cpp \
  -o bpc.debug


g++ -O3 -I../libmin/include/ \
  ../libmin/src/camera3d.cpp \
  ../libmin/src/file_png.cpp \
  ../libmin/src/vec.cpp \
  ../libmin/src/quaternion.cpp \
  ../libmin/src/dataptr.cpp \
  ../libmin/src/mersenne.cpp \
  ../libmin/src/common_defs.cpp \
  belief_propagation.cpp \
  main_belief_propagation.cpp \
  -o bpc


