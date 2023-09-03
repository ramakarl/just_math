
lib: lib/camera3d.o lib/common_defs.o lib/dataptr.o lib/datax.o lib/file_png.o lib/mersenne.o lib/quaternion.o lib/string_helper.o lib/vec.o lib/belief_propagation.o lib/bp_helper.o lib/belief_propagation_residue.o lib/tests_belief_propagation.o

lib.debug: lib.debug/camera3d.o lib.debug/common_defs.o lib.debug/dataptr.o lib.debug/datax.o lib.debug/file_png.o lib.debug/mersenne.o lib.debug/quaternion.o lib.debug/string_helper.o lib.debug/vec.o lib.debug/belief_propagation.o lib.debug/bp_helper.o lib.debug/belief_propagation_residue.o lib.debug/tests_belief_propagation.o

lib.debug/camera3d.o: ../libmin/src/camera3d.cpp
	g++ -c -g -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/camera3d.cpp -o lib.debug/camera3d.o

lib.debug/common_defs.o: ../libmin/src/common_defs.cpp
	g++ -c -g -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/common_defs.cpp -o lib.debug/common_defs.o

lib.debug/dataptr.o: ../libmin/src/dataptr.cpp
	g++ -c -g -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/dataptr.cpp -o lib.debug/dataptr.o

lib.debug/datax.o: ../libmin/src/datax.cpp
	g++ -c -g -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/datax.cpp -o lib.debug/datax.o

lib.debug/file_png.o: ../libmin/src/file_png.cpp
	g++ -c -g -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/file_png.cpp -o lib.debug/file_png.o

lib.debug/mersenne.o: ../libmin/src/mersenne.cpp
	g++ -c -g -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/mersenne.cpp -o lib.debug/mersenne.o

lib.debug/quaternion.o: ../libmin/src/quaternion.cpp
	g++ -c -g -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/quaternion.cpp -o lib.debug/quaternion.o

lib.debug/string_helper.o: ../libmin/src/string_helper.cpp
	g++ -c -g -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/string_helper.cpp -o lib.debug/string_helper.o

lib.debug/vec.o: ../libmin/src/vec.cpp
	g++ -c -g -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/vec.cpp -o lib.debug/vec.o


lib/camera3d.o: ../libmin/src/camera3d.cpp
	g++ -c -O3 -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/camera3d.cpp -o lib/camera3d.o

lib/common_defs.o: ../libmin/src/common_defs.cpp
	g++ -c -O3 -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/common_defs.cpp -o lib/common_defs.o

lib/dataptr.o: ../libmin/src/dataptr.cpp
	g++ -c -O3 -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/dataptr.cpp -o lib/dataptr.o

lib/datax.o: ../libmin/src/datax.cpp
	g++ -c -O3 -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/datax.cpp -o lib/datax.o

lib/file_png.o: ../libmin/src/file_png.cpp
	g++ -c -O3 -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/file_png.cpp -o lib/file_png.o

lib/mersenne.o: ../libmin/src/mersenne.cpp
	g++ -c -O3 -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/mersenne.cpp -o lib/mersenne.o

lib/quaternion.o: ../libmin/src/quaternion.cpp
	g++ -c -O3 -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/quaternion.cpp -o lib/quaternion.o

lib/string_helper.o: ../libmin/src/string_helper.cpp
	g++ -c -O3 -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/string_helper.cpp -o lib/string_helper.o

lib/vec.o: ../libmin/src/vec.cpp
	g++ -c -O3 -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC ../libmin/src/vec.cpp -o lib/vec.o


lib/tests_belief_propagation.o: tests_belief_propagation.cpp
	g++ -c -O3 -I../libmin/include/ -I /usr/include/eigen3 -D__linux__ -DLIBHELP_STATIC tests_belief_propagation.cpp -o lib/tests_belief_propagation.o

lib/belief_propagation.o: belief_propagation.cpp belief_propagation.h
	g++ -c -O3 -I../libmin/include/ -I /usr/include/eigen3 -D__linux__ -DLIBHELP_STATIC belief_propagation.cpp -o lib/belief_propagation.o

lib/bp_helper.o: bp_helper.cpp
	g++ -c -O3 -I../libmin/include/ -I /usr/include/eigen3 -D__linux__ -DLIBHELP_STATIC bp_helper.cpp -o lib/bp_helper.o

lib/belief_propagation_residue.o: belief_propagation_residue.cpp
	g++ -c -O3 -I../libmin/include/ -I /usr/include/eigen3 -D__linux__ -DLIBHELP_STATIC belief_propagation_residue.cpp -o lib/belief_propagation_residue.o


lib.debug/tests_belief_propagation.o: tests_belief_propagation.cpp
	g++ -c -g -I../libmin/include/ -I /usr/include/eigen3 -D__linux__ -DLIBHELP_STATIC tests_belief_propagation.cpp -o lib.debug/tests_belief_propagation.o

lib.debug/belief_propagation.o: belief_propagation.cpp belief_propagation.h
	g++ -c -g -I../libmin/include/ -I /usr/include/eigen3 -D__linux__ -DLIBHELP_STATIC belief_propagation.cpp -o lib.debug/belief_propagation.o

lib.debug/bp_helper.o: bp_helper.cpp
	g++ -c -g -I../libmin/include/ -I /usr/include/eigen3 -D__linux__ -DLIBHELP_STATIC bp_helper.cpp -o lib.debug/bp_helper.o

lib.debug/belief_propagation_residue.o: belief_propagation_residue.cpp
	g++ -c -g -I../libmin/include/ -I /usr/include/eigen3 -D__linux__ -DLIBHELP_STATIC belief_propagation_residue.cpp -o lib.debug/belief_propagation_residue.o


bpc: lib/*.o main_belief_propagation.cpp
	g++ -O3 -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC -I/usr/include/eigen3 -I../libmin/mains $^ -lm -o bpc

bpc.debug: lib.debug/*.o main_belief_propagation.cpp
	g++ -g -I../libmin/include/ -D__linux__ -DLIBHELP_STATIC -I/usr/include/eigen3 -I../libmin/mains $^  -lm -o bpc.debug
