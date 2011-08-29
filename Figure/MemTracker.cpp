//cpp文件
#include "stdafx.h"
#include "MemTracker.h"

#include <assert.h>
#include <opencv2/legacy/blobtrack.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

MemTracker::MemTracker(void)
{
	m_id = 0;

	// 注册管理函数
	cvSetMemoryManager(alloc_func, free_func, (void*)this);
}

MemTracker::~MemTracker(void)
{
	// 取消管理函数

	cvSetMemoryManager(NULL, NULL, NULL);

	// 输出结果

	this->output();
}

// 登记分配/释放的内存

void MemTracker::regAlloc(void *ptr, size_t size)
{
	// 每次记录一个新的m_id
	m_memTracker.push_back(Ptr(ptr, size, m_id++));
}

void MemTracker::regFree(void *ptr)
{
	int i;
	for(i = 0; i < m_memTracker.size(); ++i)
	{
		// 删除记录
		if(m_memTracker[i].ptr == ptr)
		{
			m_memTracker[i] = m_memTracker[m_memTracker.size()-1];
			m_memTracker.pop_back();
			return;
		}
	}
}

// 输出泄漏的内存

int MemTracker::output()
{
	int n = m_memTracker.size();
	int i;

	for(i = 0; i < n; ++i)
	{
		printf( "%d: %p, %u\n", m_memTracker[i].id, m_memTracker[i].ptr, m_memTracker[i].size);
	}

	return n;
}

// 分配内存

void* MemTracker::alloc_func(size_t size, void *userdata)
{
	assert(size > 0 && userdata != NULL);

	// 分配内存

	void *ptr = malloc(size);
	if(!ptr) return NULL;

	// 登记
	MemTracker *tracker = (MemTracker*)userdata;
	tracker->regAlloc(ptr, size);

	//

	return ptr;
}

// 释放内存

int MemTracker::free_func(void *ptr, void *userdata)
{
	assert(ptr != NULL && userdata != NULL);

	// 释放内存

	free(ptr);

	// 登记

	MemTracker *tracker = (MemTracker*)userdata;
	tracker->regFree(ptr);

	// CV_OK == 0

	return 0;
}