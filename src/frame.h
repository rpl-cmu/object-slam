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
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>

#include <utility>
#include <vector>

#include "utils/macros.h"
#include "utils/pipeline_payload.h"

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

        explicit Frame(Timestamp timestamp, const cv::Mat &r_color, const cv::Mat &r_depth,
                       const camera::PinholeCameraIntrinsic &r_intrinsic, bool is_maskframe = false)
            : PipelinePayload(timestamp),
              m_width(r_color.cols),
              m_height(r_color.rows),
              m_color(r_color),
              m_depth(r_depth),
              m_intrinsic(r_intrinsic),
              m_is_maskframe(is_maskframe){};

        ~Frame() override = default;

        //! Copy constructor
        Frame(const Frame &r_frame)
            : PipelinePayload(r_frame.m_timestamp),
              m_width(r_frame.m_width),
              m_height(r_frame.m_height),
              m_color(r_frame.m_color),
              m_depth(r_frame.m_depth),
              m_intrinsic(r_frame.m_intrinsic),
              m_is_maskframe(r_frame.m_is_maskframe){};

       public:
        //! Frame size
        const int m_width  = -1;
        const int m_height = -1;
        //! Color and depth images
        const cv::Mat m_color;
        const cv::Mat m_depth;

        //! Camera intrinsics
        const camera::PinholeCameraIntrinsic m_intrinsic;

        //! We decide at read whether a frame needs instance segmentation
        const bool m_is_maskframe = { false };

        //! Pose of the frame
        Eigen::Matrix4d m_pose = Eigen::Matrix4d::Identity();
    };
}  // namespace oslam
#endif /* ifndef OSLAM_FRAME_H */
