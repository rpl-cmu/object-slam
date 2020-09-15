/******************************************************************************
* File:             visualizer.cpp
*
* Author:           Akash Sharma
* Created:          08/25/20
* Description:      Pangolin based visualizer implementation
*****************************************************************************/
#include "visualizer.h"
#include <mutex>
#include <spdlog/spdlog.h>

namespace oslam {
    //! TODO: Akash
    PangolinVisualizer::PangolinVisualizer()
    {
        spdlog::debug("CONSTRUCT: PangolinVisualizer");
    }
    PangolinVisualizer::~PangolinVisualizer()
    {
        spdlog::debug("DESTRUCT: PangolinVisualizer");
    }

    void PangolinVisualizer::requestTermination()
    {
        std::scoped_lock<std::mutex> lock(termination_mtx_);
        termination_requested_ = true;
    }

    bool PangolinVisualizer::isTerminated() const
    {
        std::scoped_lock<std::mutex> lock(termination_mtx_);
        return is_terminated_;
    }



} /* namespace oslam */

