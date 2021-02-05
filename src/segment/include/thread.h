#ifndef _THREAD_H_
#define _THREAD_H_

#include <pthread.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory.h>




#define	THREAD_WAIT_MILLISECOND		100

namespace skywell{

class Thread
{
	public:
		Thread(void);
		virtual ~Thread(void);
	public:
		virtual int Process(void);	//返回-99就挂起，所以如果没有这个需要，就不要返回-99

		pthread_t ThreadId(void);
		int GetRunFlag(void);
	public:
		int ThreadStart(void);
		int ThreadStop(void);
	private:
		int ThreadInit(void);
		int ThreadFree(void);
		int PrivateThreadRun(void);
	private:
		int ThreadLock(void);
		int ThreadUnLock(void);
		int ThreadWait(void);
		int ThreadSignal(void);
	private:
		static void *ThreadRunWork(void *arg);
	private:
		pthread_t			thread_id;
		pthread_attr_t		thread_attr;

		pthread_mutex_t	thread_mutex;
		pthread_cond_t		thread_cond;

		int	thread_cond_num;
		int	runflag;	//0:停止	1:停止成功	2:运行	3:退出	4:退出成功		(外部只能操作：0、2、3	内部只能操作：1、2、4
		int	checkflag;	//0:未检查	1:已检查
};


}
#endif

