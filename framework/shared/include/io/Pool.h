/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_POOL_H
#define IO_POOL_H

#include <list>

#include <cassert>
#include <sys/mman.h>

namespace slambench {
	namespace io {
		template<typename T> class ObjectPool {
		public:
			typedef T object_t;
			
			T *New() = delete;
			void Delete(T*) = delete;
		};
		
		template<typename T> class StdAllocObjectPool : public ObjectPool<T> {
		public:
			T *New() { return new T(); }
			void Delete(T* t) { delete t; }
		};
		
		template<typename T> class ListObjectPool : public ObjectPool<T> {
		public:
			T *New() { _data.emplace_front(); return &_data.front(); }
			void Delete(T* t) { _data.erase(t); }
			
		private:
			std::list<T> _data;
		};
		
		class DataPool {
		public:
			void *Malloc(size_t size) = delete;
			void Free(char *ptr) = delete;
		};
		
		template<size_t page_size = 4096> class PageDataPool : public DataPool {
		private:
			char *_open_page, *_open_page_ptr;
		public:		
			PageDataPool() : _open_page(nullptr), _open_page_ptr(nullptr) {}
			
			void *Malloc(size_t size) {
				assert(size <= page_size);
				if((_open_page_ptr == nullptr) || (_open_page_ptr + size >= (_open_page + page_size))) {
					_open_page = (char*)mmap(nullptr, page_size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
					_open_page_ptr = _open_page;
				}
				
				char *rval = _open_page_ptr;
				_open_page_ptr += size;
				return rval;
			}
			
			void Free(void* t) {
				// todo
			}
		};
				
		template<typename T, size_t page_size=4096> class PageObjectPool : public ObjectPool<T> {
		
			static_assert(sizeof(T) <= page_size, "Value type is too big to fit into a page");
			
		private:
			PageDataPool<page_size> _datapool;
		public:
			T *New() { return new (_datapool.Malloc(sizeof(T))) T (); }
			void Delete(T *t) { /* todo */ }
		};
	}
}

#endif /* IO_POOL_H */

