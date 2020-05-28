/******************************************************************************
 * File:             frame.h
 *
 * Author:
 * Created:          04/07/20
 * Description:      Frame header: Logical RGBD Frame object
 *****************************************************************************/
#ifndef OSLAM_FRAME_H
#define OSLAM_FRAME_H

#include <Eigen/Eigen>
#include <Open3D/Open3D.h>
#include <vector>

#include "utils/macros.h"
#include "utils/pipeline_payload.h"
#include "masked_image.h"

namespace oslam {

    using namespace open3d;

struct Odometry
{
    Eigen::Matrix4d transform;
    Eigen::Matrix6d information;

    Odometry(const Eigen::Matrix4d &r_transform = Eigen::Matrix4d::Identity(),
      const Eigen::Matrix6d &r_information = Eigen::Matrix6d::Identity())
      : transform(r_transform), information(r_information)
    {}
};

/*! \class Frame
 *  \brief Encapsulation data structure for a single RGBD frame
 */
class Frame : public PipelinePayload
{
  public:
    OSLAM_POINTER_TYPEDEFS(Frame);

    //! Constructor
    //! Frame maintains a reference to the images.
    //! Required to clone if frame needs to own the images
    //! TODO(Akash): Is it safe to maintain a reference to non-owned Image objects?
    //! TODO(Akash): Open3D does not provide clone API for images..
    explicit Frame(std::uint64_t frame_id,
      camera::PinholeCameraIntrinsic intrinsic,
      const geometry::Image &r_color,
      const geometry::Image &r_depth,
      bool is_maskframe = false,
      const geometry::Image &r_gt_mask = geometry::Image(),
      const std::vector<unsigned int> &r_gt_labels = {},
      const std::vector<double> &r_gt_scores = {});

    virtual ~Frame() = default;

    //! Copy constructor
    Frame(const Frame &r_frame)
      : PipelinePayload(r_frame.m_timestamp), m_intrinsic(r_frame.m_intrinsic),
        m_color(r_frame.m_color), m_depth(r_frame.m_depth), m_depthf(r_frame.m_depthf), m_is_maskframe(r_frame.m_is_maskframe),
        m_gt_mask(r_frame.m_gt_mask)
    {
        mp_rgbd =
          geometry::RGBDImage::CreateFromColorAndDepth(m_color, m_depth, 1000, 3.0, false);
    };

    //! Required for image transport
    inline geometry::Image get_color(void) const { return m_color; }
    inline geometry::Image get_depth(void) const { return m_depth; }
    inline geometry::Image get_depthf(void)const { return m_depthf; }

    inline camera::PinholeCameraIntrinsic get_intrinsics(void) const { return m_intrinsic; }

    //! Required for Odometry functions
    inline std::shared_ptr<geometry::RGBDImage> get_rgbd(void) const { return mp_rgbd; }

    inline Eigen::Matrix4d get_pose(void) const { return m_pose; }
    inline void set_pose(Eigen::Matrix4d pose) { m_pose = std::move(pose); }

    inline bool is_maskframe() const { return m_is_maskframe; }


    //! TODO(Akash): Reconsider when required
    /* std::shared_ptr<Odometry> odometry(std::shared_ptr<Frame> p_target_frame, */
    /*   const Eigen::Matrix4d &r_init_transformation = Eigen::Matrix4d::Identity()); */
    /* void process_mask(std::unique_ptr<oslam::MaskedImage> p_masked_image); */
    /* void visualize(); */
    //! Required for object masks generated from masked image
    /* std::vector<unsigned int> m_labels; */
    /* std::vector<double> m_scores; */
    /* std::vector<std::shared_ptr<geometry::RGBDImage>> mv_object_rgbd; */
    /* inline bool has_segmentation(void) const { return mp_masked_image ? true : false; } */
    /* inline void set_segmentation(MaskedImage::UniquePtr p_masked_image) */
    /* { */
    /*     mp_masked_image = std::move(p_masked_image); */
    /* } */

  private:
    //! Camera intrinsics
    camera::PinholeCameraIntrinsic m_intrinsic;

    //! Color and depth images
    geometry::Image m_color;
    geometry::Image m_depth;
    geometry::Image m_depthf;

    //! We decide at read whether a frame needs instance segmentation
    const bool m_is_maskframe = { false };

    //! Groundtruth segmentation mask and labels/scores from dataset if available
    MaskedImage m_gt_mask;

    //! Pose of the frame
    Eigen::Matrix4d m_pose = Eigen::Matrix4d::Identity();

    //! RGBD object of the frame constructed from color and depth images
    std::shared_ptr<geometry::RGBDImage> mp_rgbd;
};
}// namespace oslam
#endif /* ifndef OSLAM_FRAME_H */
