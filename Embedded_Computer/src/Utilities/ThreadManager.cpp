#include <boost/uuid/uuid_generators.hpp>
#include <pair>
#include <unistd.h>
#include <sched.h>
#include <cstdio>

#include "logger.h"
#include "ThreadManager.h"

ThreadManager::ThreadManager()
{

}

~ThreadManager::ThreadManager()
{
    for (auto it = _threads.begin(); it != _threads.end(); ++it)
    {
       stop(it->first);
    }
}

boost::uuids::uuid ThreadManager::create(unsigned int priority, auto function)
{
    Logger::GetInstance(Logger::LogLvl::DEBUG) << "Creating new thread with priority : " << priority << std::endl;
    boost::uuids::uuid thread_id = boost::uuids::random_generator()();
    boost::thread* newThread = new boost::thread(function); // TODO
    _threads.insert(std::pair<boost::uuids::uuid, boost::thread>(thread_id, newThread));

    boost::condition_variable cond; 
    _cond_variables.insert(std::pair<boost::uuids::uuid, boost::cond_variable>(thread_id, cond));

    int retcode;
    int policy;
    struct sched_param param;
    pthread_t threadID = (pthread_t) newThread->native_handle();
   
    if ((retcode = pthread_getschedparam(threadID, &policy, &param)) != 0)
    {
        Logger::GetInstance(Logger::LogLvl::ERROR) << "ThreadManager.cpp : Error in function pthread_getschedparam -- " << retcode << std::endl;
        exit(EXIT_FAILURE);
    }

    // SCHED_RR policy
    // Each thread is allowed to run for a limited time period. If the thread exceeds that time, it is returned to the list for its priority.
    // Priority :
    // Threads have a priority from 1 to 99, and higher priority threads always preempt lower priority threads
    policy = SCHED_RR;
    param.sched_priority = priority;

    if ((retcode = pthread_setschedparam(threadID, policy, &param)) != 0)
    {
        Logger::GetInstance(Logger::LogLvl::ERROR) << "ThreadManager.cpp : Error in function pthread_setschedparam -- " << retcode << std::endl;
        exit(EXIT_FAILURE);
    }

    if ((res = sched_setscheduler(getpid(), policy, &param)) == -1)
    {
        Logger::GetInstance(Logger::LogLvl::ERROR) << "ThreadManager.cpp : Error in function sched_setscheduler -- " << retcode << std::endl;
        exit(EXIT_FAILURE);
    }
    return thread_id;
}

void ThreadManager::stop(boost::uuids::uuid thread_id)
{
    Logger::GetInstance(Logger::LogLvl::DEBUG) << "ThreadManager.cpp : Stopping thread" << std::endl;
    _threads[thread_id]->second->interrupt();
    _threads[thread_id]->second->join();
}

void ThreadManager::attach(boost::uuids::uuid thread_id)
{
    Logger::GetInstance(Logger::LogLvl::DEBUG) << "ThreadManager.cpp : Joining thread" << std::endl;
    _threads[thread_id]->second->join();
}

void ThreadManager::resume(boost::uuids::uuid thread_id)
{
    Logger::GetInstance(Logger::LogLvl::DEBUG) << "ThreadManager.cpp : Resuming thread" << std::endl;
    _cond_variables[thread_id]->second.notify_one(); 
}
