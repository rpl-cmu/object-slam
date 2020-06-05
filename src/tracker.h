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

#include <limits>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "map.h"
#include "tracker_payload.h"
#include "utils/macros.h"
#include "utils/pipeline_module.h"
#include "utils/thread_safe_queue.h"

namespace oslam
{
  /*! \class Tracker
   *  \brief Brief class description
   *
   *  Detailed description
   */
  class Tracker : public MISOPipelineModule<TrackerPayload, NullPipelinePayload>
  {
   public:
    OSLAM_POINTER_TYPEDEFS(Tracker);
    OSLAM_DELETE_COPY_CONSTRUCTORS(Tracker);

    using MISO             = MISOPipelineModule<TrackerPayload, NullPipelinePayload>;
    using MaskedImageQueue = ThreadsafeQueue<MaskedImage::UniquePtr>;

    explicit Tracker(MaskedImageQueue* p_masked_image_queue, OutputQueue* p_output_queue);
    virtual ~Tracker() = default;

    void fill_frame_queue(Frame::Ptr p_frame) { m_frame_queue.push(std::make_unique<Frame>(*p_frame)); }

    virtual OutputUniquePtr run_once(InputUniquePtr p_input) override;

    virtual bool has_work() const override { return (m_curr_timestamp < m_max_timestamp); }

    virtual void set_max_timestamp(Timestamp timestamp) { m_max_timestamp = timestamp; }

   private:
    virtual InputUniquePtr get_input_packet() override;

    virtual void shutdown_queues() override;
    bool m_first_run = { true };

    //! Input Queues which are to be synchronised
    ThreadsafeQueue<Frame::UniquePtr> m_frame_queue;
    MaskedImageQueue* mp_masked_image_queue;
    MaskedImage::UniquePtr mp_prev_masked_image;

    //! Reference to the global map
    GlobalMap& mr_global_map;

    //! Current timestamp being processed
    Timestamp m_curr_timestamp = 0;
    Timestamp m_max_timestamp  = std::numeric_limits<Timestamp>::max();

    //! Vector of trajectory poses w.r.t world coordinate
    std::vector<Eigen::Matrix4d> mv_T_camera_2_world;
    //! Camera intrinsics
    cuda::PinholeCameraIntrinsicCuda mc_intrinsic;

    cuda::ImageCuda<ushort, 1> mc_curr_depth_raw;
    cuda::ImageCuda<uchar, 3> mc_curr_color;

    //! TODO(Akash): Consider coarse-to-fine ICP
    cuda::ImageCuda<float, 3> mc_curr_vertex_map;
    cuda::ImageCuda<float, 3> mc_curr_normal_map;

    //! Previous frame vertex and normal map in global coordinates
    cuda::ImageCuda<float, 3> mc_g_prev_vertex_map;
    cuda::ImageCuda<float, 3> mc_g_prev_normal_map;
    cuda::ImageCuda<uchar, 3> mc_g_prev_color;
  };
}  // namespace oslam

#endif /* ifndef OSLAM_TRACKER_H */
