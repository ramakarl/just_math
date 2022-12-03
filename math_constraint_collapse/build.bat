
REM # MSVC compiler
set clexe="C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC\14.27.29110\bin\Hostx64\x64\cl.exe"
set stdinc="C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC\14.27.29110\include"
set crtinc="C:\Program Files (x86)\Windows Kits\10\Include\10.0.18362.0\ucrt"

REM # STD and CRT libs
set stdlib="C:\Program Files (x86)\Microsoft Visual Studio\2019\Community\VC\Tools\MSVC\14.27.29110\lib\x64"
set kernlib="C:\Program Files (x86)\Windows Kits\10\Lib\10.0.18362.0\um\x64"
set crtlib="C:\Program Files (x86)\Windows Kits\10\Lib\10.0.18362.0\ucrt\x64"

REM # Libmin path
set libmininc="D:\Codes\just_math\libmin\include"
set libmin="D:\Codes\just_math\libmin\src"

%clexe% main_belief_propagation.cpp belief_propagation.cpp %libmin%\file_png.cpp %libmin%\camera3d.cpp %libmin%\vec.cpp %libmin%\dataptr.cpp %libmin%\mersenne.cpp %libmin%\quaternion.cpp %libmin%\common_defs.cpp -w -O2 -DBUILD_CMDLINE -I %stdinc% -I %crtinc% -I %libmininc% /link /LIBPATH:%stdlib% /LIBPATH:%kernlib% /LIBPATH:%crtlib% 

copy main_belief_propagation.exe bpc.exe


REM # Example run
REM # bpc -N examples/rg_name.csv -R examples/rg_rule.csv -C examples/rg_constraint.csv -X 10 -Y 5 -Z 1