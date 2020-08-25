/******************************************************************************
 * File:             threadclass.h
 *
 * Author:           Akash Sharma
 * Created:          05/11/20
 * Description:      Thread Wrapper class for safety
 *****************************************************************************/
#ifndef OSLAM_THREAD
#define OSLAM_THREAD

#include <atomic>
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
    Thread(const std::string &r_threadname) :m_threadname(r_threadname) {}

    // Non-copyable
    Thread(const Thread&) = delete;
    Thread& operator=(const Thread&) = delete;

    virtual ~Thread() = default;

    virtual bool start()
    {
        is_running = true;
        return run();
    }


  protected:
    bool run(void)
    {
        spdlog::info("Thread ({},{}) started", m_threadname, std::this_thread::get_id());
        //TODO: Add running conditional variable

        while (process())
        {
            //TODO: Add Timer
        }
        is_running = false;
        spdlog::info("Thread ({}, {}), ended", m_threadname, std::this_thread::get_id());
        return false;
    }

    virtual bool process(void) = 0;

    /* data */
    std::string m_threadname;
    //TODO: Reconsider this to conditional variable
    std::atomic_bool is_running = {false};
};
}// namespace oslam
#endif /* ifndef OSLAM_THREAD */
