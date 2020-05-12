/******************************************************************************
 * File:             threadclass.h
 *
 * Author:           Akash Sharma
 * Created:          05/11/20
 * Description:      Thread Wrapper class for safety
 *****************************************************************************/
#ifndef OSLAM_THREAD
#define OSLAM_THREAD

#include <cassert>
#include <condition_variable>
#include <thread>
#include <mutex>
#include <iostream>

#include <spdlog/spdlog.h>

namespace oslam {

class Thread
{
  public:
    Thread(std::string threadname) :m_threadname(threadname), is_ready(false) {}

    // Non-copyable
    Thread(const Thread&) = delete;
    Thread& operator=(const Thread&) = delete;

    virtual ~Thread() = default;

    virtual void start()
    {
        std::scoped_lock<std::mutex> lock(m_mutex);
        is_ready = true;
        run();
    }


  protected:
    void run(void)
    {
        spdlog::info("Thread ({},{}) started", m_threadname, std::this_thread::get_id());
        //TODO: Add running conditional variable

        while (process())
        {
            //TODO: Add Timer
        }

        spdlog::info("Thread ({}, {}), ended", m_threadname, std::this_thread::get_id());
    }

    virtual bool process(void) = 0;

    /* data */
    std::string m_threadname;
    //TODO: Reconsider this to conditional variable
    std::mutex m_mutex;
    bool is_ready;
};
}// namespace oslam
#endif /* ifndef OSLAM_THREAD */
