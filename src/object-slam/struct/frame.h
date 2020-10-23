/******************************************************************************
 * File:             frame.h
 *
 * Author:
 * Created:          04/07/20
 * Description:      Frame header: Logical RGBD Frame object
 *****************************************************************************/
#ifndef OSLAM_FRAME_H
#define OSLAM_FRAME_H

#include <Open3D/Open3D.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

#include <utility>
#include <vector>

#include "object-slam/utils/macros.h"
#include "object-slam/utils/pipeline_payload.h"

namespace oslam
{
    using namespace open3d;

    /*! \class Frame
     *  \brief Encapsulation data structure for RGB + Depth information
     */
    struct Frame : public PipelinePayload
    {
       public:
        OSLAM_POINTER_TYPEDEFS(Frame);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit Frame(Timestamp timestamp,
                       const cv::Mat& color,
                       const cv::Mat& depth,
                       const camera::PinholeCameraIntrinsic& intrinsic,
                       bool is_keyframe   = false,
                       float depth_factor = 1000.0f,
                       float max_depth    = 3.0f)
            : PipelinePayload(timestamp),
              width_(color.cols),
              height_(color.rows),
              color_(color),
              depth_(depth),
              intrinsic_(intrinsic),
              is_keyframe_(is_keyframe),
              depth_factor_(depth_factor),
              max_depth_(max_depth){};

        ~Frame() override = default;

        //! Copy constructor and assignment
        Frame(const Frame& frame)
            : PipelinePayload(frame.timestamp_),
              width_(frame.width_),
              height_(frame.height_),
              color_(frame.color_),
              depth_(frame.depth_),
              intrinsic_(frame.intrinsic_),
              is_keyframe_(frame.is_keyframe_),
              depth_factor_(frame.depth_factor_),
              max_depth_(frame.max_depth_){};

       public:
        //! Frame size
        const int width_  = -1;
        const int height_ = -1;

        //! Color and depth images
        const cv::Mat color_;
        const cv::Mat depth_;
        const camera::PinholeCameraIntrinsic intrinsic_;
        const bool is_keyframe_   = { false };  //!< False if frame doesn't have segmentation
        const float depth_factor_ = 1000.0f;
        const float max_depth_    = 3.0f;
        Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();  //!< Pose of the frame to current local submap TODO: Required?
    };

    //! TODO: Possibly add a new file for this struct
    struct Render
    {
       public:
        OSLAM_POINTER_TYPEDEFS(Render);
        Render(cv::Mat color_map, cv::Mat vertex_map, cv::Mat normal_map)
            : color_map_(std::move(color_map)), vertex_map_(std::move(vertex_map)), normal_map_(std::move(normal_map))
        {
        }

        ~Render() = default;
        cv::Mat color_map_;
        cv::Mat vertex_map_;
        cv::Mat normal_map_;
    };

    using Renders          = std::vector<std::pair<ObjectId, Render>>;
    using RendersUniquePtr = std::unique_ptr<Renders>;

    struct ObjectRenders : public PipelinePayload
    {
       public:
        OSLAM_POINTER_TYPEDEFS(ObjectRenders);
        ObjectRenders(Timestamp timestamp, const Renders& renders) : PipelinePayload(timestamp), renders_(renders) {}
        ~ObjectRenders() = default;
        Renders renders_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_FRAME_H */
