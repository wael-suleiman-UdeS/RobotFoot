/////////////////////////////////////////////////////////////////////
//
// Class ThreadManager
// Author : Mickael Paradis
// Date : October 11
// Description : Class managing threads and their priorities.
//
/////////////////////////////////////////////////////////////////////
#ifndef THREADMANAGER_H
#define THREADMANAGER_H
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <map>
#include <string>

class ThreadManager
{
    public:
        ThreadManager();
        ~ThreadManager();

        boost::thread::id create(unsigned int priority, std::string thread_name, const boost::function0<void>& thread_func);
        
        void stop(boost::thread::id thread_id);
        void stop(std::string thread_name);

        void attach(boost::thread::id thread_id);
        void attach(std::string thread_name);

        void resume(boost::thread::id thread_id);
        void resume(std::string thread_name);

        void wait();
        int calculate_the_answer_to_life_the_universe_and_everything();     
    private:
        std::map<boost::thread::id, boost::thread*> _threads;
        std::map<boost::thread::id, boost::condition_variable*> _cond_variables;
        std::map<std::string, boost::thread::id> _names; 
};
#endif
