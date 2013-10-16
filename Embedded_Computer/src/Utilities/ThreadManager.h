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
#include <boost/thread/thread.hpp>
#include <boost/function.hpp>
#include <boost/uuid/uuid.hpp>
#include <map>

class ThreadManager
{
    public:
        ThreadManager();
        ~ThreadManager();

        boost::uuids::uuid create(unsigned int priority, auto function);
        
        void stop(boost::uuids::uuid thread_id);
        void attach(boost::uuids::uuid thread_id);
        void resume(boost::uuids::uuid thread_id);        
    private:
        std::map<boost::uuids::uuid, boost::thread*> _threads;
        std::map<boost::uuids::uuid, boost::condition_variable> _cond_variables;
};
#endif
