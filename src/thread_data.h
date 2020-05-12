/******************************************************************************
* File:             thread_data.h
*
* Author:           Akash Sharma
* Created:          05/11/20
* Description:      Thread synchronisation data
*****************************************************************************/
#ifndef OSLAM_THREAD_DATA_H
#define OSLAM_THREAD_DATA_H

namespace oslam {
class ThreadData
{
public:
    static ThreadData& get(void)
    {
        static ThreadData data;
        return data;
    }
    virtual ~ThreadData ();

private:
    ThreadData();
    /* data */
};
}
#endif /* ifndef OSLAM_THREAD_DATA_H */
