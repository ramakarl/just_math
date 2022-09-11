//--------------------------------------------------------------------------------
// Copyright 2019-2022 (c) Quanta Sciences, Rama Hoetzlein, ramakarl.com
//
// 
// * Derivative works may append the above copyright notice but should not remove or modify earlier notices.
//
// MIT License:
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and 
// associated documentation files (the "Software"), to deal in the Software without restriction, including without 
// limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
// and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
// OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS 
// BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF 
// OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#ifdef USE_CUDA

	#include "common_defs.h"
	#include "common_cuda.h"
	#include <assert.h>

	bool cuCheck(CUresult launch_stat, char* method, char* apicall, char* arg, bool bDebug)
	{
		CUresult kern_stat = CUDA_SUCCESS;

		if (bDebug) {
			kern_stat = cuCtxSynchronize();
		}
		if (kern_stat != CUDA_SUCCESS || launch_stat != CUDA_SUCCESS) {
			const char* launch_statmsg = "";
			const char* kern_statmsg = "";
			cuGetErrorString(launch_stat, &launch_statmsg);
			cuGetErrorString(kern_stat, &kern_statmsg);
			dbgprintf("------- CUDA ERROR:\n");
			dbgprintf("  Launch status: %s\n", launch_statmsg);
			dbgprintf("  Kernel status: %s\n", kern_statmsg);
			dbgprintf("  Caller: Particles::%s\n", method);
			dbgprintf("  Call:   %s\n", apicall);
			dbgprintf("  Args:   %s\n", arg);

			if (bDebug) {
				dbgprintf("  Generating assert to examine call stack.\n");
				assert(0);		// debug - trigger break (see call stack)
			}
			else {
				exit(-1);
			}
			return false;
		}
		return true;
	}

	void cuStart(int devsel, CUcontext ctxsel, CUdevice& dev, CUcontext& ctx, CUstream* strm, bool verbose)
	{
		int version = 0;
		char name[128];

		int cnt = 0;
		CUdevice dev_id;
		cuInit(0);

		//--- List devices
		cuDeviceGetCount(&cnt);

		if (cnt == 0) {
			dbgprintf("ERROR: No CUDA devices found.\n");
			dev = NULL; ctx = NULL;
			exit(-1);
			return;
		}
		if (verbose) dbgprintf("  Device List:\n");
		for (int n = 0; n < cnt; n++) {
			cuDeviceGet(&dev_id, n);
			cuDeviceGetName(name, 128, dev_id);

			int w1, h1, d1;
			cuDeviceGetAttribute(&w1, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_WIDTH, dev_id);
			cuDeviceGetAttribute(&h1, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_HEIGHT, dev_id);
			cuDeviceGetAttribute(&d1, CU_DEVICE_ATTRIBUTE_MAXIMUM_TEXTURE3D_DEPTH, dev_id);

			if (verbose) dbgprintf("   %d. %s  maxtex (%d,%d,%d)\n", n, name, w1, h1, d1);
		}

		if (devsel == DEV_CURRENT) {
			//--- Get currently active context 
			cuCheck(cuCtxGetCurrent(&ctx), "", "cuCtxGetCurrent", "", false);
			cuCheck(cuCtxGetDevice(&dev), "", "cuCtxGetDevice", "", false);
		}
		if (devsel == DEV_EXISTING) {
			//--- Use existing context passed in
			ctx = ctxsel;
			cuCheck(cuCtxSetCurrent(ctx), "", "cuCtxSetCurrent", "", false);
			cuCheck(cuCtxGetDevice(&dev), "", "cuCtxGetDevice", "", false);
		}
		if (devsel == DEV_FIRST || devsel >= cnt) devsel = 0;
		if (devsel >= cnt) devsel = 0;				// Fallback to dev 0 if addition GPUs not found

		if (devsel >= 0) {
			//--- Create new context with Driver API 
			cuCheck(cuDeviceGet(&dev, devsel), "", "cuDeviceGet", "", false);
			cuCheck(cuCtxCreate(&ctx, CU_CTX_SCHED_AUTO, dev), "", "cuCtxCreate", "", false);
		}
		cuDeviceGetName(name, 128, dev);
		if (verbose) dbgprintf("   Using Device: %d, %s, Context: %p\n", (int)dev, name, (void*)ctx);

		cuCtxSetCurrent(NULL);
		cuCtxSetCurrent(ctx);
	}

	void cuGetMemUsage(int& total, int& used, int& free)
	{
		size_t f, t;
		cuMemGetInfo(&f, &t);
		free = f / size_t(1024) * size_t(1024);	// MB
		total = t / size_t(1024) * size_t(1024);
		used = total - free;                	// used
	}

#endif



