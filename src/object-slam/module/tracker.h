/******************************************************************************
 * File:             tracker.h
 *
 * Author:           Akash Sharma
 * Created:          05/10/20
 * Description:      Tracker Thread
 *****************************************************************************/
#ifndef OSLAM_TRACKER_H
#define OSLAM_TRACKER_H

#include <Cuda/Camera/PinholeCameraIntrinsicCuda.h>
#include <Cuda/Geometry/ImageCuda.h>
#include <Cuda/Registration/RegistrationCuda.h>
#include <Eigen/src/Core/util/Memory.h>

#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "object-slam/utils/macros.h"
#include "object-slam/utils/pipeline_module.h"
#include "object-slam/utils/thread_safe_queue.h"

#include "object-slam/payload/renderer_payload.h"
#include "object-slam/payload/tracker_payload.h"

#include "object-slam/struct/map.h"

namespace oslam
{
    /*! \class Tracker
     *  \brief Tracks the incoming camera frame via frame-to-model to obtain relative camera pose
     */
    class Tracker : public MISOPipelineModule<TrackerInput, TrackerOutput>
    {
       public:
        OSLAM_POINTER_TYPEDEFS(Tracker);
        OSLAM_DELETE_COPY_CONSTRUCTORS(Tracker);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using MISO            = MISOPipelineModule<TrackerInput, TrackerOutput>;
        using ModelInputQueue = ThreadsafeQueue<Model::UniquePtr>;
        using FrameInputQueue = ThreadsafeQueue<Frame::UniquePtr>;

        explicit Tracker(OutputQueue* output_queue);
        virtual ~Tracker() = default;

        void fillFrameQueue(Frame::UniquePtr frame) { frame_queue_.push(std::move(frame)); }
        void fillModelQueue(Model::UniquePtr model_payload) { model_queue_.push(std::move(model_payload)); }

        virtual OutputUniquePtr runOnce(InputUniquePtr input) override;

        virtual bool hasWork() const override { return (curr_timestamp_ < max_timestamp_); }
        virtual void setMaxTimestamp(Timestamp timestamp) { max_timestamp_ = timestamp; }

       private:
        virtual InputUniquePtr getInputPacket() override;

        virtual void shutdownQueues() override;

        //! Input Queues which are to be synchronised
        FrameInputQueue frame_queue_;
        ModelInputQueue model_queue_;

        Timestamp curr_timestamp_ = 0;
        Timestamp max_timestamp_  = std::numeric_limits<Timestamp>::max();

        Eigen::Matrix4d prev_camera_pose;  //!< T_camera_to_world_ at prev timestep

        cuda::PinholeCameraIntrinsicCuda intrinsic_cuda_;
    };
}  // namespace oslam

#endif /* ifndef OSLAM_TRACKER_H */
