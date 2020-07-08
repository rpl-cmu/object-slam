/******************************************************************************
 * File:             thread_data.h
 *
 * Author:           Akash Sharma
 * Created:          05/11/20
 * Description:      Thread synchronisation data
 *****************************************************************************/
#ifndef OSLAM_THREAD_DATA_H
#define OSLAM_THREAD_DATA_H

#include <condition_variable>
#include <mutex>
namespace oslam
{
    /*! \class ThreadSyncVar
     *  \brief Thread synchronisation helper variable with mutex and condition variable
     *
     *  Detailed description
     */
    template<typename T>
    class ThreadSyncVar
    {
       public:
        ThreadSyncVar(){};

        ThreadSyncVar(T init_value) : m_var(init_value), m_var_copy(init_value) {}
        virtual ~ThreadSyncVar() = default;

        void assign_value(T value)
        {
            std::scoped_lock<std::mutex> lock(m_mutex);
            m_var      = value;
            m_var_copy = value;
        }

        void assign_notify_all(T value)
        {
            {
                std::scoped_lock<std::mutex> lock(m_mutex);
                m_var = value;
            }
            m_cv.notify_all();
        }

        void assign_notify_one(T value)
        {
            {
                std::scoped_lock<std::mutex> lock(m_mutex);
                m_var = value;
            }
            m_cv.notify_one();
        }

        void notify_all()
        {
            m_cv.notify_all();
        }
        void notify_one()
        {
            m_cv.notify_one();
        }

        void wait(T val)
        {
            std::unique_lock<std::mutex> lock(m_mutex);
            m_cv.wait(lock, [&]{ return (m_var == val); });
        }
       protected:
        T m_var;
        T m_var_copy;
        std::mutex m_mutex;
        std::condition_variable m_cv;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_THREAD_DATA_H */
