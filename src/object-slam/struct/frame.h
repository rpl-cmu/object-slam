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
                       bool is_maskframe = false)
            : PipelinePayload(timestamp),
              width_(color.cols),
              height_(color.rows),
              color_(color),
              depth_(depth),
              intrinsic_(intrinsic),
              is_maskframe_(is_maskframe){};

        ~Frame() override = default;

        //! Copy constructor and assignment
        Frame(const Frame& frame)
            : PipelinePayload(frame.timestamp_),
              width_(frame.width_),
              height_(frame.height_),
              color_(frame.color_),
              depth_(frame.depth_),
              intrinsic_(frame.intrinsic_),
              is_maskframe_(frame.is_maskframe_){};
        Frame& operator=(const Frame& frame) = default;

        //! Move constructor and assignment TODO: Required?
        Frame(Frame&& frame) = default;
        Frame& operator=(Frame&& frame) = default;

       public:
        //! Frame size
        const int width_  = -1;
        const int height_ = -1;
        //! Color and depth images
        const cv::Mat color_;
        const cv::Mat depth_;
        const camera::PinholeCameraIntrinsic intrinsic_;
        const bool is_maskframe_ = { false };                 //!< Does frame have instance segmentation
        Eigen::Matrix4d pose_ = Eigen::Matrix4d::Identity();  //!< Pose of the frame to current local submap TODO: Required?
    };
}  // namespace oslam
#endif /* ifndef OSLAM_FRAME_H */
