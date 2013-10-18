#include <boost/lexical_cast.hpp>
#include <utility> // std::pair
#include <unistd.h>
#include <sched.h>
#include <cstdio>

#include "logger.h"
#include "ThreadManager.h"

ThreadManager::ThreadManager()
{

}

ThreadManager::~ThreadManager()
{
    for (auto it = _threads.begin(); it != _threads.end(); ++it)
    {
       stop(it->first);
    }
}

boost::thread::id ThreadManager::create(unsigned int priority, std::string thread_name, const boost::function0<void>& thread_func)
{
    Logger::getInstance(Logger::LogLvl::DEBUG) << "Creating new thread with priority : " << priority << std::endl;
    boost::thread* newThread = new boost::thread(thread_func);
    _threads.insert(std::make_pair(newThread->get_id(), newThread));

    boost::condition_variable* cond = new boost::condition_variable(); 
    _cond_variables.insert(std::make_pair(newThread->get_id(), cond));

    _names.insert(std::make_pair(thread_name, newThread->get_id()));

    int retcode;
    int policy;
    struct sched_param param;
    pthread_t threadID = (pthread_t) newThread->native_handle();
   
    if ((retcode = pthread_getschedparam(threadID, &policy, &param)) != 0)
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "ThreadManager.cpp : Error in function pthread_getschedparam -- " << retcode << std::endl;
        exit(EXIT_FAILURE);
    }

    // SCHED_RR policy
    // Each thread is allowed to run for a limited time period. If the thread exceeds that time, it is returned to the list for its priority.
    // Priority :
    // Threads have a priority from 1 to 99 and higher priority threads always preempt lower priority threads
    policy = SCHED_RR;
    param.sched_priority = priority;

    if ((retcode = pthread_setschedparam(threadID, policy, &param)) != 0)
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "ThreadManager.cpp : Error in function pthread_setschedparam -- " << retcode << std::endl;
        exit(EXIT_FAILURE);
    }
    return newThread->get_id();
}

void ThreadManager::stop(boost::thread::id thread_id)
{
    Logger::getInstance(Logger::LogLvl::DEBUG) << "ThreadManager.cpp : Stopping thread " << boost::lexical_cast<std::string>(thread_id) << std::endl;
    _threads[thread_id]->interrupt();
    _threads[thread_id]->join();
}

void ThreadManager::stop(std::string thread_name)
{
    stop(_names[thread_name]);
}

void ThreadManager::attach(boost::thread::id thread_id)
{
    Logger::getInstance(Logger::LogLvl::DEBUG) << "ThreadManager.cpp : Joining thread " << boost::lexical_cast<std::string>(thread_id) << std::endl;
    _threads[thread_id]->join();
}

void ThreadManager::attach(std::string thread_name)
{
    attach(_names[thread_name]);
}

void ThreadManager::resume(boost::thread::id thread_id)
{
    Logger::getInstance(Logger::LogLvl::DEBUG) << "ThreadManager.cpp : Resuming thread " << boost::lexical_cast<std::string>(thread_id) << std::endl;
    _cond_variables[thread_id]->notify_one(); 
}

void ThreadManager::resume(std::string thread_name)
{
    resume(_names[thread_name]);
}

void ThreadManager::wait()
{
    Logger::getInstance(Logger::LogLvl::DEBUG) << "ThreadManager.cpp : Pausing thread " << boost::lexical_cast<std::string>(boost::this_thread::get_id()) << std::endl;
    boost::mutex mut;
    boost::unique_lock<boost::mutex> lock(mut);
    _cond_variables[boost::this_thread::get_id()]->wait(lock);     
}

int calculate_the_answer_to_life_the_universe_and_everything()
{
    return 42;
}
