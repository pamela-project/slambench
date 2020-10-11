/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "sb_malloc.h"

#include <dlfcn.h> 

#include <map>
#include <mutex>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <new>

#ifdef ANDROID
#define __throw() 
#else
#define __throw() throw()
#endif

static bool track_allocs_frame = false;
static bool track_allocs = false;

namespace slambench {
namespace memory {

MemoryProfile MemoryProfile::singleton;



void MemoryProfile::StartFrame(int i) { _frame = i; data_[_frame]; gpu_data_[_frame]; track_allocs_frame = true;}
void MemoryProfile::EndFrame() { track_allocs_frame = false; }

void MemoryProfile::StartAlgorithm() {
	std::cerr << "*** Start memory tracking" << std::endl;
	track_allocs = true;
}
void MemoryProfile::EndAlgorithm() { track_allocs = false; }


class MemoryProfiler {
	public:
		static MemoryProfiler &GetSingleton() { return *singleton_ptr_; }
		static bool SingletonAvailable() { return singleton_ptr_ != nullptr && singleton_ptr_->IsEnabled(); }
		
		MemoryProfiler() : _enabled(true) {}
		~MemoryProfiler() { singleton_ptr_ = nullptr; }
		
		MemoryData &GetData() { return MemoryProfile::singleton.DataForCurrentFrame(); }
		MemoryData &GetGPUData() { return MemoryProfile::singleton.GPUDataForCurrentFrame(); }
		MemoryData &GetOverallData() { return MemoryProfile::singleton.OverallData(); }
		MemoryData &GetOverallGPUData() { return MemoryProfile::singleton.OverallGPUData(); }
	
		bool IsEnabled() const { return _enabled; }
		void Enable() { _enabled = true; }
		
		void AddAllocation(void *ptr, size_t size) {

			if(!track_allocs) return;
			
			track_allocs = false;
			
			_allocation_sizes[ptr] = size;
			
			if(track_allocs_frame) {
				GetData().RecordAllocation(size);
			}
			GetOverallData().RecordAllocation(size);
			
			track_allocs = true;
		}
		void FreeAllocation(void *ptr) {

			if(!track_allocs) return;
			
			track_allocs = false;
			
			auto size = _allocation_sizes[ptr];
			
			if(track_allocs_frame) {
				GetData().RecordFree(size);
			}
			GetOverallData().RecordFree(size);
			
			_allocation_sizes.erase(ptr);
			
			track_allocs = true;
		}
		void AddAllocationGPU(void *ptr, size_t size) {
				if(!track_allocs) return;

				track_allocs = false;

				_gpu_allocation_sizes[ptr] = size;

				if(track_allocs_frame) {
					GetGPUData().RecordAllocation(size);
				}
				GetOverallGPUData().RecordAllocation(size);

				track_allocs = true;
			}
			void FreeAllocationGPU(void *ptr) {
				if(!track_allocs) return;

				track_allocs = false;

				auto size = _gpu_allocation_sizes[ptr];

				if(track_allocs_frame) {
					GetGPUData().RecordFree(size);
				}
				GetOverallGPUData().RecordFree(size);

				_gpu_allocation_sizes.erase(ptr);

				track_allocs = true;
			}


	private:
			std::map<void*, size_t> _allocation_sizes;
			std::map<void*, size_t> _gpu_allocation_sizes;
			
			static MemoryProfiler singleton_;
			static MemoryProfiler *singleton_ptr_;
		bool _enabled;

};

MemoryProfiler MemoryProfiler::singleton_;
MemoryProfiler *MemoryProfiler::singleton_ptr_ = &singleton_;
	
}
}

#define ENABLE_MEM_PROFILING
#ifdef ENABLE_MEM_PROFILING

extern "C" {

//###################################################################################################################
//###################################### SYNC            ############################################################
//###################################################################################################################

static std::recursive_mutex gpu_alloc_lock;
typedef std::lock_guard<decltype(gpu_alloc_lock)> gpu_lock_guard_t;

static std::recursive_mutex alloc_lock;
typedef std::lock_guard<decltype(alloc_lock)> lock_guard_t;

static std::recursive_mutex opencl_alloc_lock;
typedef std::lock_guard<decltype(alloc_lock)> opencl_lock_guard_t;

//###################################################################################################################
//###################################### GPU SIDE : CUDA ############################################################
//###################################################################################################################

typedef int cudaError_t ;
typedef void * cudaArray_t ;

#define ADD_CUDA_ALLOCATION2(FunName , Arg1 , Arg2 , ArgName1,  ArgName2 , Ptr, Size )  \
static cudaError_t load_and_call_libcuda_##FunName(	Arg1 , Arg2 ) ;                       \
typedef cudaError_t (*FunName##_t)(Arg1 , Arg2);                                          \
static FunName##_t libcuda_##FunName = load_and_call_libcuda_##FunName;                   \
static cudaError_t load_and_call_libcuda_##FunName(	Arg1 , Arg2) {                        \
	libcuda_##FunName = (FunName##_t)dlsym(RTLD_NEXT, #FunName);                          \
	if (!libcuda_##FunName) {                                                             \
		MEM_ERROR("Bad day for %s.\n", #FunName);                                         \
		exit(1);                                                                          \
	}                                                                                     \
	return libcuda_##FunName(ArgName1,ArgName2);                                          \
}                                                                                         \
cudaError_t FunName (Arg1 , Arg2) __throw()                                               \
{                                                                                         \
	gpu_lock_guard_t lock(gpu_alloc_lock);                                                \
	cudaError_t error = libcuda_##FunName(ArgName1,ArgName2);                             \
	MEM_DEBUG("Call %s.\n", #FunName);                                                    \
	if(slambench::memory::MemoryProfiler::SingletonAvailable()) {                         \
		slambench::memory::MemoryProfiler::GetSingleton().AddAllocationGPU(Ptr, Size);    \
	}                                                                                     \
	return error;                                                                         \
};                                                                                        \
                                                                                          \

#define ADD_CUDA_ALLOCATION4(FunName , Arg1 , Arg2 , Arg3 , Arg4 , ArgName1,  ArgName2 ,  ArgName3,  ArgName4 , Ptr, Size )  \
static cudaError_t load_and_call_libcuda_##FunName(	Arg1 , Arg2 ,  Arg3 , Arg4 ) ;                                             \
typedef cudaError_t (*FunName##_t)(Arg1 , Arg2 ,  Arg3 , Arg4);                                                                \
static FunName##_t libcuda_##FunName = load_and_call_libcuda_##FunName;                                                        \
static cudaError_t load_and_call_libcuda_##FunName(	Arg1 , Arg2 ,  Arg3 , Arg4) {                                              \
	libcuda_##FunName = (FunName##_t)dlsym(RTLD_NEXT, #FunName);                                                               \
	if (!libcuda_##FunName) {                                                                                                  \
		MEM_ERROR("Bad day for %s.\n", #FunName);                                                                              \
		exit(1);                                                                                                               \
	}                                                                                                                          \
	return libcuda_##FunName(ArgName1,ArgName2 ,  ArgName3,  ArgName4);                                                        \
}                                                                                                                              \
cudaError_t FunName (Arg1 , Arg2 ,  Arg3 , Arg4) __throw()                                                                     \
{                                                                                                                              \
	gpu_lock_guard_t lock(gpu_alloc_lock);                                                                                     \
	cudaError_t error = libcuda_##FunName(ArgName1,ArgName2 ,  ArgName3,  ArgName4);                                           \
	MEM_DEBUG("Call %s.\n", #FunName);                                                    \
	if(slambench::memory::MemoryProfiler::SingletonAvailable()) {                                                              \
		slambench::memory::MemoryProfiler::GetSingleton().AddAllocationGPU(Ptr, Size);                                         \
	}                                                                                                                          \
	return error;                                                                                                              \
};



#define CATCH_CUDA_FUNC4(FunName , Arg1 , Arg2 , Arg3 , Arg4 , ArgName1,  ArgName2 ,  ArgName3,  ArgName4 )                    \
static cudaError_t load_and_call_libcuda_##FunName(	Arg1 , Arg2 ,  Arg3 , Arg4 ) ;                                             \
typedef cudaError_t (*FunName##_t)(Arg1 , Arg2 ,  Arg3 , Arg4);                                                                \
static FunName##_t libcuda_##FunName = load_and_call_libcuda_##FunName;                                                        \
static cudaError_t load_and_call_libcuda_##FunName(	Arg1 , Arg2 ,  Arg3 , Arg4) {                                              \
	libcuda_##FunName = (FunName##_t)dlsym(RTLD_NEXT, #FunName);                                                               \
	if (!libcuda_##FunName) {                                                                                                  \
		MEM_ERROR("Bad day for %s.\n", #FunName);                                                                              \
		exit(1);                                                                                                               \
	}                                                                                                                          \
	return libcuda_##FunName(ArgName1,ArgName2 ,  ArgName3,  ArgName4);                                                        \
}                                                                                                                              \
cudaError_t FunName (Arg1 , Arg2 ,  Arg3 , Arg4) __throw()                                                                     \
{                                                                                                                              \
	gpu_lock_guard_t lock(gpu_alloc_lock);                                                                                     \
	cudaError_t error = libcuda_##FunName(ArgName1,ArgName2 ,  ArgName3,  ArgName4);                                           \
	MEM_DEBUG("Call %s.\n", #FunName);                                                                                         \
	return error;                                                                                                              \
};

#define CATCH_CUDA_FUNC5(FunName , Arg1 , Arg2 , Arg3 , Arg4 , Arg5 , ArgName1,  ArgName2 ,  ArgName3,  ArgName4 , ArgName5 )  \
static cudaError_t load_and_call_libcuda_##FunName(	Arg1 , Arg2 ,  Arg3 , Arg4 , Arg5) ;                                       \
typedef cudaError_t (*FunName##_t)(Arg1 , Arg2 ,  Arg3 , Arg4, Arg5);                                                          \
static FunName##_t libcuda_##FunName = load_and_call_libcuda_##FunName;                                                        \
static cudaError_t load_and_call_libcuda_##FunName(	Arg1 , Arg2 ,  Arg3 , Arg4, Arg5) {                                        \
	libcuda_##FunName = (FunName##_t)dlsym(RTLD_NEXT, #FunName);                                                               \
	if (!libcuda_##FunName) {                                                                                                  \
		MEM_ERROR("Bad day for %s.\n", #FunName);                                                                              \
		exit(1);                                                                                                               \
	}                                                                                                                          \
	return libcuda_##FunName(ArgName1,ArgName2 ,  ArgName3,  ArgName4 , ArgName5);                                             \
}                                                                                                                              \
cudaError_t FunName (Arg1 , Arg2 ,  Arg3 , Arg4, Arg5) __throw()                                                               \
{                                                                                                                              \
	gpu_lock_guard_t lock(gpu_alloc_lock);                                                                                     \
	cudaError_t error = libcuda_##FunName(ArgName1,ArgName2 ,  ArgName3,  ArgName4 , ArgName5);                                \
	MEM_DEBUG("Call %s.\n", #FunName);                                                                                         \
	return error;                                                                                                              \
};




// cudaError_t cudaMalloc ( void** devPtr, size_t size )
//********************************************************

ADD_CUDA_ALLOCATION2(cudaMalloc ,
		void** devPtr, size_t size,
		devPtr,  size ,
		(void*) *devPtr, size );

// cudaError_t cudaMallocHost ( void** ptr, size_t size )
//********************************************************

ADD_CUDA_ALLOCATION2(cudaMallocHost ,
		void** devPtr, size_t size,
		devPtr,  size ,
		(void*) *devPtr, size );


// cudaError_t cudaMallocPitch ( void** devPtr, size_t* pitch, size_t width, size_t height )
//********************************************************

ADD_CUDA_ALLOCATION4(cudaMallocPitch ,
		void** devPtr, size_t size,  size_t width, size_t height ,
		devPtr,  size , width, height ,
		(void*) *devPtr,width * height );


// __host__ ​cudaError_t cudaMallocArray ( cudaArray_t* array, const cudaChannelFormatDesc* desc_, size_t width, size_t height = 0, unsigned int  flags = 0 )
//   Allocate an array on the device.
//********************************************************
/*
CATCH_CUDA_FUNC5(cudaMallocArray ,
		cudaArray_t* array, const void* desc_, size_t width, size_t height , unsigned int  flags
		, array,  desc_ ,  width,  height , flags )   ;
*/



// cudaError_t cudaFree ( void* devPtr )
//********************************************************

static cudaError_t load_and_call_libcuda_cudaFree(	 void* devPtr) ;

typedef cudaError_t (*cudaFree_t)(	 void* devPtr);

static cudaFree_t     libcuda_cudaFree       = load_and_call_libcuda_cudaFree;

static cudaError_t load_and_call_libcuda_cudaFree(	 void* devPtr) {

	libcuda_cudaFree = (cudaFree_t) dlsym(RTLD_NEXT, "cudaFree");
	if (!libcuda_cudaFree) {
		MEM_ERROR("The memory profiler did not find cudaFree. You may need to recompile without memory profiling.\n");
		exit(1);
	}

	cudaError_t res = libcuda_cudaFree(devPtr);

	return res;
}

cudaError_t cudaFree (  void* devPtr) __throw()
{

	gpu_lock_guard_t lock(gpu_alloc_lock);


	if(slambench::memory::MemoryProfiler::SingletonAvailable()) {
		slambench::memory::MemoryProfiler::GetSingleton().FreeAllocationGPU(devPtr);
		return libcuda_cudaFree(devPtr);
	} else {
		return libcuda_cudaFree(devPtr);
	}

}

// cudaError_t cudaFreeHost ( void* devPtr)
//********************************************************

static cudaError_t load_and_call_libcuda_cudaFreeHost(	 void* devPtr) ;

typedef cudaError_t (*cudaFreeHost_t)(	 void* devPtr);

static cudaFreeHost_t     libcuda_cudaFreeHost       = load_and_call_libcuda_cudaFreeHost;

static cudaError_t load_and_call_libcuda_cudaFreeHost(	 void* devPtr) {
	libcuda_cudaFreeHost = (cudaFreeHost_t)dlsym(RTLD_NEXT, "cudaFreeHost");
	if (!libcuda_cudaMalloc) {
		MEM_ERROR("Bad day for cudaFreeHost.\n");
		exit(1);
	}
	return libcuda_cudaFreeHost(devPtr);
}

cudaError_t cudaFreeHost (  void* devPtr) __throw()
{

	gpu_lock_guard_t lock(gpu_alloc_lock);

	if(slambench::memory::MemoryProfiler::SingletonAvailable()) {
		slambench::memory::MemoryProfiler::GetSingleton().FreeAllocationGPU(devPtr);
		return libcuda_cudaFreeHost(devPtr);
	} else {
		return libcuda_cudaFreeHost(devPtr);
	}

}


//###################################################################################################################
//###################################### GPU SIDE : OPENCL ##########################################################
//###################################################################################################################

// We provide our own clCreateBuffer/clReleaseMemObject
#include <stdint.h>
typedef int32_t         cl_int      __attribute__((aligned(4)));
typedef uint64_t        cl_ulong    __attribute__((aligned(8)));

typedef cl_ulong                    cl_bitfield;
typedef struct _cl_context *        cl_context;
typedef cl_bitfield                 cl_mem_flags;
typedef struct _cl_mem *            cl_mem;

static cl_mem load_and_call_libopencl_clCreateBuffer(	cl_context a,  	cl_mem_flags b,  	size_t c,  	void *d,  	cl_int *e) ;
static cl_int load_and_call_libopencl_clReleaseMemObject(cl_mem a) ;

typedef cl_mem (*clCreateBuffer_t)(	cl_context ,  	cl_mem_flags ,  	size_t ,  	void *,  	cl_int *);
typedef cl_int (*clReleaseMemObject_t)(cl_mem);

static clCreateBuffer_t     libopencl_clCreateBuffer       = load_and_call_libopencl_clCreateBuffer;
static clReleaseMemObject_t libopencl_clReleaseMemObject   = load_and_call_libopencl_clReleaseMemObject;

static cl_mem load_and_call_libopencl_clCreateBuffer(	cl_context a,  	cl_mem_flags b,  	size_t c,  	void *d,  	cl_int *e) {
	libopencl_clCreateBuffer = (clCreateBuffer_t)dlsym(RTLD_NEXT, "clCreateBuffer");

	return libopencl_clCreateBuffer(a,b,c,d,e);
}
static cl_int load_and_call_libopencl_clReleaseMemObject(cl_mem a) {
	libopencl_clReleaseMemObject = (clReleaseMemObject_t)dlsym(RTLD_NEXT, "clReleaseMemObject");
	return libopencl_clReleaseMemObject(a);
}


cl_mem clCreateBuffer ( 	cl_context a ,  	cl_mem_flags b ,  	size_t c ,  	void * d,  	cl_int * e) __throw()
{
	if(slambench::memory::MemoryProfiler::SingletonAvailable()) {
		lock_guard_t lock(opencl_alloc_lock);
		cl_mem ptr = libopencl_clCreateBuffer(a,b,c,d,e);
		slambench::memory::MemoryProfiler::GetSingleton().AddAllocationGPU((void*) ptr, c);
		
		return ptr;
	} else {
		return libopencl_clCreateBuffer(a,b,c,d,e);
	}
}

cl_int clReleaseMemObject ( 	cl_mem ptr )__throw()
{	
	if(slambench::memory::MemoryProfiler::SingletonAvailable()) {
		lock_guard_t lock(opencl_alloc_lock);
		slambench::memory::MemoryProfiler::GetSingleton().FreeAllocationGPU((void*) ptr);
		return libopencl_clReleaseMemObject(ptr);
	} else {
		return libopencl_clReleaseMemObject(ptr);
	}
}

//###################################################################################################################
//###################################### CPU SIDE       #############################################################
//###################################################################################################################

// We provide our own malloc/calloc/realloc/free, which are then used
// by whatever we link against (i.e., SLAM implementations).

static void *load_and_call_libc_malloc(size_t);
static void load_and_call_libc_free(void*);
static void *load_and_call_libc_calloc(size_t, size_t);
static void *load_and_call_libc_realloc(void*, size_t);

typedef void *(*malloc_t)(size_t);
typedef void (*free_t)(void*);
typedef void *(*realloc_t)(void*, size_t);
typedef void *(*calloc_t)(size_t, size_t);

// TODO: make this thread-safe

static malloc_t libc_malloc = load_and_call_libc_malloc;
static free_t libc_free = load_and_call_libc_free;
static calloc_t libc_calloc = load_and_call_libc_calloc;
static realloc_t libc_realloc = load_and_call_libc_realloc;

static void* load_and_call_libc_malloc(size_t t) {
	libc_malloc = (malloc_t)dlsym(RTLD_NEXT, "malloc");
	return libc_malloc(t);
}
static void load_and_call_libc_free(void *t) {
	libc_free = (free_t)dlsym(RTLD_NEXT, "free");
	libc_free(t);
}

static void* load_and_call_libc_calloc(size_t nmemb, size_t size) {
	libc_calloc = (calloc_t)dlsym(RTLD_NEXT, "calloc");
	return libc_calloc(nmemb, size);
}
static void *load_and_call_libc_realloc(void *t, size_t size) {
	libc_realloc = (realloc_t)dlsym(RTLD_NEXT, "realloc");
	return libc_realloc(t, size);
}

//define a scratch space for calloc
static char calloc_scratch[4096];
static char* calloc_ptr = calloc_scratch;


void *malloc(size_t size) __throw()
{
	lock_guard_t lock(alloc_lock);
	void *ptr = libc_malloc(size);
	
	if(slambench::memory::MemoryProfiler::SingletonAvailable()) {
		slambench::memory::MemoryProfiler::GetSingleton().AddAllocation(ptr, size);
	}
		
	return ptr;
}

void free(void* ptr) __throw()
{
	lock_guard_t lock(alloc_lock);
	
	if(slambench::memory::MemoryProfiler::SingletonAvailable()) {
		if(ptr > calloc_scratch && ptr < calloc_scratch+sizeof(calloc_scratch)) {
			// do nothing
		}
		else
		{
			libc_free(ptr);
			slambench::memory::MemoryProfiler::GetSingleton().FreeAllocation(ptr);
		}
	} else {
		libc_free(ptr);
	}
}

void *realloc(void *ptr, size_t newsize) __throw() {
	lock_guard_t lock(alloc_lock);
	
	if(slambench::memory::MemoryProfiler::SingletonAvailable()) {
		if(ptr > calloc_scratch && ptr < calloc_scratch+sizeof(calloc_scratch)) {
			abort();
		}

		auto newptr = libc_realloc(ptr, newsize);

		slambench::memory::MemoryProfiler::GetSingleton().FreeAllocation(ptr);
		slambench::memory::MemoryProfiler::GetSingleton().AddAllocation(newptr, newsize);

		return newptr;
	} else {
		return libc_realloc(ptr, newsize);
	}
}

void *calloc(size_t nmemb, size_t size) __throw() {
	lock_guard_t lock(alloc_lock);
	
	// need to be careful with calloc since dlsym will use it! need to do this
	// even if the profiler is disabled.
	static bool reentered = false;
	if(reentered) {
		void *ptr = calloc_ptr;
		calloc_ptr += nmemb*size;
		calloc_ptr += 16-(((uintptr_t)calloc_ptr) % 16);
		if(calloc_ptr >= (calloc_scratch + sizeof(calloc_scratch))) abort();
		return ptr;
	}
	reentered = true;
	
	if(slambench::memory::MemoryProfiler::SingletonAvailable()) {
		void *result = libc_calloc(nmemb, size);
		slambench::memory::MemoryProfiler::GetSingleton().AddAllocation(result, nmemb * size);
		reentered = false;
		return result;
	} else {
		void *ptr = libc_calloc(nmemb, size);
		reentered = false;
		return ptr;
	}
}

}

void *operator new(std::size_t size) {
	lock_guard_t lock(alloc_lock);
	
	void *mem = malloc(size);
	
	if(mem == nullptr) {
		throw std::bad_alloc();
	} else {
		return mem;
	}
}

void *operator new(std::size_t size, const std::nothrow_t &nothrow_value) __throw() {
	(void)nothrow_value;
	try {
		return operator new(size);
	} catch (std::exception &e) {
		return nullptr;
	}
}

void *operator new[](std::size_t size) {
	lock_guard_t lock(alloc_lock);
	
	void *mem = malloc(size);
	
	if(mem == nullptr) {
		throw std::bad_alloc();
	} else {
		return mem;
	}
}

void *operator new[](std::size_t size, const std::nothrow_t &nothrow_value) __throw() {
	(void)nothrow_value;
	try {
		return operator new[](size);
	} catch (std::exception &e) {
		return nullptr;
	}
}

#endif
