//ͷ�ļ�
#ifndef OPENCV_MEM_TRACKER_H
#define OPENCV_MEM_TRACKER_H

#include <stdio.h>
#include <vector>

// �ڴ�й©׷����
class MemTracker
{
public:
	MemTracker(void);
	~MemTracker(void);

private:

	// �ǼǷ���/�ͷŵ��ڴ�
	void regAlloc(void *ptr, size_t size);
	void regFree(void *ptr);

	// ���й©���ڴ�
	int output();

private:

	// �����ڴ�
	static void* alloc_func(size_t size, void *userdata);

	// �ͷ��ڴ�
	static int free_func(void *ptr, void *userdata);

private:

	struct Ptr
	{
		void *ptr;      // �ڴ��ַ
		size_t size;   // �ڴ��С
		int   id;

		Ptr(void *ptr, size_t size, int id)
		{
			this->ptr = ptr;
			this->size = size;
			this->id = id;
		}
	};

	// ��¼��ǰʹ���е��ڴ�
	std::vector<Ptr>   m_memTracker;

	// alloc_func��Ӧ�ı��
	int   m_id;
};

#endif   // OPENCV_MEM_TRACKER_H