/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifdef __APPLE__
    #include <OpenCL/cl.h>
#else
    #include <CL/cl.h>
#endif

#include <stdlib.h>
#include <sys/types.h>
#include <dlfcn.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

#define CL_ENQUEUE_NDRANGE_KERNEL_NAME "clEnqueueNDRangeKernel"
typedef cl_int (*clEnqueueNDRangeKernelFunction)(cl_command_queue command_queue,
		cl_kernel kernel, cl_uint work_dim, const size_t* global_work_offset,
		const size_t* global_work_size, const size_t* local_work_size,
		cl_uint num_events_in_wait_list, const cl_event* event_wait_list,
		cl_event* event);

#define CL_CREATE_COMMAND_QUEUE_NAME "clCreateCommandQueue"
typedef cl_command_queue (*clCreateCommandQueueFunction)(cl_context,
		cl_device_id, cl_command_queue_properties, cl_int*);

#define checkError(err,str) if (err != CL_SUCCESS)  {std::cout << str << std::endl; exit(err);}

unsigned long int computeEventDuration(cl_event* event) {
	if (event == NULL)
		throw std::runtime_error(
				"Error computing event duration. \
                              Event is not initialized");
	cl_int errorCode;
	cl_ulong start, end;
	errorCode = clGetEventProfilingInfo(*event, CL_PROFILING_COMMAND_START,
			sizeof(cl_ulong), &start, NULL);
	checkError(errorCode, "Error querying the event start time");
	errorCode = clGetEventProfilingInfo(*event, CL_PROFILING_COMMAND_END,
			sizeof(cl_ulong), &end, NULL);
	checkError(errorCode, "Error querying the event end time");
	return static_cast<unsigned long>(end - start);
}

std::string getKernelName(cl_kernel kernel) {
	size_t nameSize;
	cl_int errorCode = clGetKernelInfo(kernel, CL_KERNEL_FUNCTION_NAME, 0, NULL,
			&nameSize);
	checkError(errorCode, "Error querying the kernel name size");
	char* name = new char[nameSize];

	errorCode = clGetKernelInfo(kernel, CL_KERNEL_FUNCTION_NAME, nameSize, name,
	NULL);
	checkError(errorCode, "Error querying the kernel name");

	std::string nameString(name);
	delete[] name;

	return nameString;
}

void enqueueKernel(cl_command_queue command_queue, cl_kernel kernel,
		cl_uint work_dim, const size_t* global_work_offset,
		const size_t* global_work_size, const size_t* local_work_size,
		cl_uint num_events_in_wait_list, const cl_event* event_wait_list,
		cl_event* event, const std::string& kernelName) {

	static const char* kernel_to_change = getenv("KERNEL");
	size_t new_local_work_size[3];
	size_t new_global_work_size[3];
	const size_t * pnew_local_work_size = local_work_size;
	const size_t * pnew_global_work_size = global_work_size;

	if (kernel_to_change != NULL) {
		if (kernelName == kernel_to_change) {

			pnew_local_work_size = (const size_t *) &new_local_work_size;
			for (unsigned int index = 0; index < work_dim; ++index) {
				std::stringstream varname;
				varname << "ls" << index;
				char* stringws = getenv(varname.str().c_str());
				if (stringws) {
					std::istringstream(stringws) >> new_local_work_size[index];
				} else {
					pnew_local_work_size = local_work_size;
					break;
				};
			}
			if (pnew_local_work_size == (const size_t *) &new_local_work_size) {
				std::cout << "New localsize used :" << pnew_local_work_size[0];
				if (work_dim > 1)
					std::cout << "x" << pnew_local_work_size[1];
				if (work_dim > 2)
					std::cout << "x" << pnew_local_work_size[2];
				std::cout << std::endl;
			}

			pnew_global_work_size = (const size_t *) &new_global_work_size;
			for (unsigned int index = 0; index < work_dim; ++index) {
				std::stringstream varname;
				varname << "gs" << index;
				char* stringws = getenv(varname.str().c_str());
				if (stringws) {
					std::istringstream(stringws) >> new_global_work_size[index];
				} else {
					pnew_global_work_size = global_work_size;
					break;
				};
			}
			if (pnew_global_work_size
					== (const size_t *) &new_global_work_size) {
				std::cout << "New globalsize used :"
						<< pnew_global_work_size[0];
				if (work_dim > 1)
					std::cout << "x" << pnew_global_work_size[1];
				if (work_dim > 2)
					std::cout << "x" << pnew_global_work_size[2];
				std::cout << std::endl;
			}

		}

	}

	// Get pointer to original function calls.
	clEnqueueNDRangeKernelFunction originalclEnqueueKernel;
	*(void **) (&originalclEnqueueKernel) = dlsym(RTLD_NEXT,
	CL_ENQUEUE_NDRANGE_KERNEL_NAME);

	cl_int errorCode = 0;

	errorCode = originalclEnqueueKernel(command_queue, kernel, work_dim,
			global_work_offset, global_work_size, pnew_local_work_size,
			num_events_in_wait_list, event_wait_list, event);

	checkError(errorCode, "Error enqueuing the original kernel");
	clFinish(command_queue);
	cl_int eventStatus = clWaitForEvents(1, event);

	std::cerr << kernelName;
	int totalglobalsize = 1;
	int totallocalsize = 1;
	for (unsigned int index = 0; index < work_dim; ++index) {
		totalglobalsize *= global_work_size[index];
		if (pnew_local_work_size) {
			totallocalsize *= pnew_local_work_size[index];
		} else {
			totallocalsize = 0;
		}

	}

	std::cerr << " " << totalglobalsize << " " << totallocalsize;

	if (eventStatus == -5) {
		std::cerr << " 0" << std::endl;
	} else {
		std::cerr << " " << computeEventDuration(event) << std::endl;
	}
	checkError(errorCode, "Error releasing the event");

}

// overload function
cl_int clEnqueueNDRangeKernel(cl_command_queue command_queue, cl_kernel kernel,
		cl_uint work_dim, const size_t* global_work_offset,
		const size_t* global_work_size, const size_t* local_work_size,
		cl_uint num_events_in_wait_list, const cl_event* event_wait_list,
		cl_event* event) {

	// Setup the event to measure the kernel execution time.
	bool isEventNull = (event == NULL);
	if (isEventNull) {
		event = new cl_event();
		clRetainEvent(*event);
	}

	std::string kernelName = getKernelName(kernel);

	enqueueKernel(command_queue, kernel, work_dim, global_work_offset,
			global_work_size, local_work_size, num_events_in_wait_list,
			event_wait_list, event, kernelName);

	if (isEventNull) {
		clReleaseEvent(*event);
		delete event;
	}

	return CL_SUCCESS;
}

// overload function
cl_command_queue clCreateCommandQueue(cl_context context, cl_device_id device,
		cl_command_queue_properties properties, cl_int* errcode_ret) {

	// Get pointer to original function calls.
	clCreateCommandQueueFunction originalclCreateCommandQueue;
	*(void **) (&originalclCreateCommandQueue) = dlsym(RTLD_NEXT,
	CL_CREATE_COMMAND_QUEUE_NAME);

	properties = properties | CL_QUEUE_PROFILING_ENABLE;

	return originalclCreateCommandQueue(context, device, properties,
			errcode_ret);
}
