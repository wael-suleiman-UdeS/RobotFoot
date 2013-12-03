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

boost::thread::id ThreadManager::create(unsigned int priority, const boost::function0<void>& thread_func, Task task)
{
    try
    {
        Logger::getInstance(Logger::LogLvl::DEBUG) << "Creating new thread with priority : " << priority << std::endl;
        std::shared_ptr<boost::thread> newThread(new boost::thread(thread_func));
        _threads.insert(std::make_pair(newThread->get_id(), newThread));

        std::shared_ptr<boost::condition_variable> cond(new boost::condition_variable()); 
        _cond_variables.insert(std::make_pair(newThread->get_id(), std::make_pair(true, cond)));

        if (task != Task::UNKNOW)
            _tasks.insert(std::make_pair(task, newThread->get_id()));

        int retcode;
        int policy;
        struct sched_param param;
        pthread_t threadID = (pthread_t) newThread->native_handle();

        if ((retcode = pthread_getschedparam(threadID, &policy, &param)) != 0)
        {
            Logger::getInstance(Logger::LogLvl::ERROR) << "ThreadManager.cpp : Error in function pthread_getschedparam -- " << retcode << std::endl;
            std::exit(1);
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
            std::exit(1);
        }
        return newThread->get_id();
    }
    catch (std::exception& e)
    {
        Logger::getInstance(Logger::LogLvl::ERROR) << "Exception while setting thread priorities : " << e.what() << std::endl;
        std::exit(1);
    }
}

void ThreadManager::stop(boost::thread::id thread_id)
{
    Logger::getInstance(Logger::LogLvl::DEBUG) << "ThreadManager.cpp : Stopping thread " << boost::lexical_cast<std::string>(thread_id) << std::endl;
    
    if (_threads.find(thread_id) != _threads.end())
    {
        _threads[thread_id]->interrupt();
        _threads[thread_id]->join();
    }
}

void ThreadManager::stop(Task task)
{
    if (_tasks.find(task) != _tasks.end())
        stop(_tasks[task]);
}

void ThreadManager::attach(boost::thread::id thread_id)
{
    Logger::getInstance(Logger::LogLvl::DEBUG) << "ThreadManager.cpp : Joining thread " << boost::lexical_cast<std::string>(thread_id) << std::endl;
    if (_threads.find(thread_id) != _threads.end())
        _threads[thread_id]->join();
}

void ThreadManager::attach(Task task)
{
    if (_tasks.find(task) != _tasks.end())
        attach(_tasks[task]);
}

bool ThreadManager::resume(boost::thread::id thread_id)
{
    if (_cond_variables.find(thread_id) != _cond_variables.end())
    {
        if (!_cond_variables[thread_id].first)
        {
            _cond_variables[thread_id].second->notify_one();
            return true;
        }
    }
    return false; 
}

bool ThreadManager::resume(Task task)
{
    if (_tasks.find(task) != _tasks.end())
    {
        return resume(_tasks[task]);
    }
    return false;
}

void ThreadManager::wait()
{
    if (_cond_variables.find(boost::this_thread::get_id()) != _cond_variables.end() &&
        _cond_variables[boost::this_thread::get_id()].first)
    {
        Logger::getInstance(Logger::LogLvl::DEBUG) << "ThreadManager.cpp : Pausing thread " << boost::lexical_cast<std::string>(boost::this_thread::get_id()) << std::endl;
        boost::mutex mut;
        boost::unique_lock<boost::mutex> lock(mut);
        _cond_variables[boost::this_thread::get_id()].first = false;
        _cond_variables[boost::this_thread::get_id()].second->wait(lock);     
    }
}

void ThreadManager::end()
{
    if (_threads.find(boost::this_thread::get_id()) != _threads.end())
    {
        _cond_variables.erase(boost::this_thread::get_id());
        _threads.erase(boost::this_thread::get_id());
    }
}

int calculate_the_answer_to_life_the_universe_and_everything()
{
    return 42;
}
