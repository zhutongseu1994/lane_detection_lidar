#include <unistd.h>
#include <stdio.h>

#include "thread.h"


namespace skywell{

Thread::Thread(void)
{
	ThreadInit();
};

Thread::~Thread(void)
{
	ThreadFree();
};

int Thread::ThreadInit(void)
{
	int _rf;
	pthread_cond_init(&thread_cond, NULL);
	pthread_mutex_init(&thread_mutex, NULL);
	pthread_attr_init(&thread_attr); 
	pthread_attr_setdetachstate(&thread_attr, PTHREAD_CREATE_DETACHED); 
	pthread_create(&thread_id, &thread_attr, ThreadRunWork, this);

	while (GetRunFlag() != 1)	//确保线程已经初始化完成
	{
		usleep(THREAD_WAIT_MILLISECOND);
	};

	return 0;
};

int Thread::ThreadFree(void)
{
	int _rf;
	char _LogBuf[1024];

	_rf = -1;
	while (1)
	{
		ThreadLock();
		if (checkflag == 0)
		{
			_rf = 0;
		}
		else
		{
			_rf = -1;
			if (runflag == 1)
			{
				_rf = 1;
			}
			checkflag = 0;
			runflag = 3;
		}
		ThreadUnLock();
		if (_rf == 0)
		{
			usleep(THREAD_WAIT_MILLISECOND);						//======
		}
		else
		{
			if (_rf == 1)
			{
				ThreadSignal();
			}
			break;
		}
	}

	while (1)
	{
		_rf = 0;
		ThreadLock();
		if (checkflag == 1)
		{
			_rf = runflag;
		}
		ThreadUnLock();

		if (_rf == 4)
		{
			break;
		}
		usleep(THREAD_WAIT_MILLISECOND);						//======
	}
	//释放
	pthread_cond_destroy(&thread_cond);
	pthread_mutex_destroy(&thread_mutex);


	return 0;
};

int Thread::Process(void)
{
	return 0;
};

int Thread::PrivateThreadRun(void)
{
	int _rf = -1;

	//开始运行，立即挂起
	ThreadLock();
	runflag = 1;
	checkflag = 1;
	ThreadWait();
	ThreadUnLock();

	while (1)
	{
		ThreadLock();
		checkflag = 1;
		_rf = runflag;
		if (_rf == 0)
		{
			runflag = 1;
			ThreadWait();	//挂起时不能进行退出操作
		}
		ThreadUnLock();
		if (_rf == 3)
		{
			break;
		}
		else
		{
			if (_rf == 2)
			{
				_rf = Process();
				if (_rf == -99)
				{
					ThreadLock();
					if (checkflag == 1)
					{
						checkflag = 0;
						runflag = 0;
					}
					ThreadUnLock();
				}
			}
		}
	}
	ThreadLock();
	checkflag = 1;
	runflag = 4;
	ThreadUnLock();
	return 0;
};

void *Thread::ThreadRunWork(void *arg)
{
	Thread * BaseThread = (Thread *)arg;
	BaseThread->PrivateThreadRun();
	return NULL;
};

int Thread::ThreadStart(void)
{
	int _rf = -1;
	while (1)
	{
		ThreadLock();
		if (checkflag == 1)
		{
			if (runflag == 2)
			{
				checkflag = 0;//确保本次的操作还能运行
				_rf = -1;
			}
			else
			{
				_rf = 2;
				checkflag = 0;
				runflag = 2;
			}
		}
		else
		{
			if (runflag != 2)
			{
				_rf = 0;
			}
			else	//如果当前已经在运行，并且还没有检查标志，就不需要重新设置
			{
				_rf = -1;
			}
		}
		ThreadUnLock();
		if (_rf == 0)
		{
			usleep(THREAD_WAIT_MILLISECOND);						//======
		}
		else
		{
			break;
		}
	}

	if (_rf == 2)
	{
		ThreadSignal();
	}

	return 0;
};

int Thread::ThreadStop(void)
{
	int _rf = -1;
	while (1)
	{
		ThreadLock();
		if ((runflag != 0) && (runflag != 1))
		{
			if (checkflag == 0)
			{
				_rf = 0;
			}
			else
			{
				_rf = -1;
				if (runflag == 2)//只有在运行的时候，才需要停止
				{
					checkflag = 0;
					runflag = 0;
				}
			}
		}
		else
		{
			_rf = -1;
		}
		ThreadUnLock();
		if (_rf == 0)
		{
			usleep(THREAD_WAIT_MILLISECOND);						//======
		}
		else
		{
			break;
		}
	}

	while (1)
	{
		_rf = -1;
		ThreadLock();
		if (checkflag == 0)
		{
			_rf = 0;
		}
		else
		{
			_rf = runflag;
		}
		ThreadUnLock();
		if (_rf == 0)
		{
			usleep(THREAD_WAIT_MILLISECOND);						//======
		}
		else
		{
			break;
		}
	}
	return 0;
};

pthread_t Thread::ThreadId(void)
{
	pthread_t _r;
	ThreadLock();
	_r = thread_id;
	ThreadUnLock();
	return _r;
};

int Thread::GetRunFlag(void)
{
	int _r;
	ThreadLock();
	_r = runflag;
	ThreadUnLock();
	return _r;
};

int Thread::ThreadLock(void)
{
	pthread_mutex_lock(&thread_mutex); 
	return 0;
};

int Thread::ThreadUnLock(void)
{
	pthread_mutex_unlock(&thread_mutex);
	return 0;
};

int Thread::ThreadWait(void)
{
	//之前必须是加锁状态
	thread_cond_num = thread_cond_num + 1;
	pthread_cond_wait(&thread_cond, &thread_mutex);
	return 0;
};

int Thread::ThreadSignal(void)
{
	pthread_cond_signal(&thread_cond);
	thread_cond_num = thread_cond_num - 1;
	return 0;
};


}
