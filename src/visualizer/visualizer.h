/******************************************************************************
* File:             visualizer.h
*
* Author:           Akash Sharma
* Created:          08/25/20
* Description:      Pangolin based Visualizer
*****************************************************************************/
#ifndef OSLAM_PANGOLIN_VISUALIZER_H
#define OSLAM_PANGOLIN_VISUALIZER_H

#include <mutex>

#include ""

namespace oslam
{
    /*! \class viewer
     *  \brief Brief class description
     *
     *  Detailed description
     */
    class PangolinVisualizer
    {
    public:
        explicit PangolinVisualizer(std::shared_ptr<oslam::Controller>& controller);
        virtual ~PangolinVisualizer();

        //! Main rendering loop to be run in main thread
        void run();

        //! Request to terminate the viewer abruptly
        void requestTermination();

        //! Check if the visualizer is terminated or not
        [[nodiscard]] bool isTerminated() const;
    private:


        mutable std::mutex termination_mtx_;
        bool termination_requested_ = false;
        bool is_terminated_ = true;
    };
} /* oslam */
#endif /* ifndef OSLAM_PANGOLIN_VISUALIZER_H */

