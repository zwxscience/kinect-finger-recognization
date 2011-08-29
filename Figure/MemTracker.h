//头文件
#ifndef OPENCV_MEM_TRACKER_H
#define OPENCV_MEM_TRACKER_H

#include <stdio.h>
#include <vector>

// 内存泄漏追踪类
class MemTracker
{
public:
	MemTracker(void);
	~MemTracker(void);

private:

	// 登记分配/释放的内存
	void regAlloc(void *ptr, size_t size);
	void regFree(void *ptr);

	// 输出泄漏的内存
	int output();

private:

	// 分配内存
	static void* alloc_func(size_t size, void *userdata);

	// 释放内存
	static int free_func(void *ptr, void *userdata);

private:

	struct Ptr
	{
		void *ptr;      // 内存地址
		size_t size;   // 内存大小
		int   id;

		Ptr(void *ptr, size_t size, int id)
		{
			this->ptr = ptr;
			this->size = size;
			this->id = id;
		}
	};

	// 记录当前使用中的内存
	std::vector<Ptr>   m_memTracker;

	// alloc_func对应的编号
	int   m_id;
};

#endif   // OPENCV_MEM_TRACKER_H