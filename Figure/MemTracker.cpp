//cpp�ļ�
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

	// ע�������
	cvSetMemoryManager(alloc_func, free_func, (void*)this);
}

MemTracker::~MemTracker(void)
{
	// ȡ��������

	cvSetMemoryManager(NULL, NULL, NULL);

	// ������

	this->output();
}

// �ǼǷ���/�ͷŵ��ڴ�

void MemTracker::regAlloc(void *ptr, size_t size)
{
	// ÿ�μ�¼һ���µ�m_id
	m_memTracker.push_back(Ptr(ptr, size, m_id++));
}

void MemTracker::regFree(void *ptr)
{
	int i;
	for(i = 0; i < m_memTracker.size(); ++i)
	{
		// ɾ����¼
		if(m_memTracker[i].ptr == ptr)
		{
			m_memTracker[i] = m_memTracker[m_memTracker.size()-1];
			m_memTracker.pop_back();
			return;
		}
	}
}

// ���й©���ڴ�

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

// �����ڴ�

void* MemTracker::alloc_func(size_t size, void *userdata)
{
	assert(size > 0 && userdata != NULL);

	// �����ڴ�

	void *ptr = malloc(size);
	if(!ptr) return NULL;

	// �Ǽ�
	MemTracker *tracker = (MemTracker*)userdata;
	tracker->regAlloc(ptr, size);

	//

	return ptr;
}

// �ͷ��ڴ�

int MemTracker::free_func(void *ptr, void *userdata)
{
	assert(ptr != NULL && userdata != NULL);

	// �ͷ��ڴ�

	free(ptr);

	// �Ǽ�

	MemTracker *tracker = (MemTracker*)userdata;
	tracker->regFree(ptr);

	// CV_OK == 0

	return 0;
}