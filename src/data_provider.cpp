/******************************************************************************
* File:             data_provider.cpp
*
* Author:           Akash Sharma
* Created:          05/15/20
* Description:      Data Provider module implementation
*****************************************************************************/
#include "data_provider.h"

namespace oslam {
    DataProvider::DataProvider(InputQueue* p_input_queue)
        : SIMO(p_input_queue, "DataProvider")
    {
    }

    //Simply pass the input to all output callbacks
    DataProvider::OutputUniquePtr DataProvider::run_once(Frame::UniquePtr input)
    {
        return input;
    }
}
