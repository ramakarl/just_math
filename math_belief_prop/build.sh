#!/bin/bash

#g++ -O3 -I../libmin/include/ ../libmin/src/vec.cpp ../libmin/src/quaternion.cpp ../libmin/src/dataptr.cpp ../libmin/src/mersenne.cpp ../libmin/src/common_defs.cpp belief_propagation.cpp

#g++ -g -I../libmin/include/ ../libmin/src/vec.cpp ../libmin/src/quaternion.cpp ../libmin/src/dataptr.cpp ../libmin/src/mersenne.cpp ../libmin/src/common_defs.cpp belief_propagation.cpp
g++ -O3 -I../libmin/include/ ../libmin/src/vec.cpp ../libmin/src/quaternion.cpp ../libmin/src/dataptr.cpp ../libmin/src/mersenne.cpp ../libmin/src/common_defs.cpp belief_propagation.cpp
#g++ -O2 -I../libmin/include/ ../libmin/src/vec.cpp ../libmin/src/quaternion.cpp ../libmin/src/dataptr.cpp ../libmin/src/mersenne.cpp ../libmin/src/common_defs.cpp belief_propagation.cpp

