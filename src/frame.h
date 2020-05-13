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

namespace oslam {

struct ImageProperties
{
    int width;
    int height;
    int num_of_channels;
    int bytes_per_channel;
};

struct MaskedImage
{
    OSLAM_POINTER_TYPEDEFS(MaskedImage);
    open3d::geometry::Image image;
    std::vector<unsigned int> labels;
    std::vector<double> scores;
};

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
class Frame
{
  public:
    OSLAM_POINTER_TYPEDEFS(Frame);

    explicit Frame(std::size_t frame_id,
          open3d::camera::PinholeCameraIntrinsic intrinsic,
          open3d::geometry::Image color,
          open3d::geometry::Image depth,
          open3d::geometry::Image gt_mask,
          bool is_keyframe = false,
          const std::vector<unsigned int> & r_gt_labels = {},
          const std::vector<double> &r_gt_scores = {}
         );
    virtual ~Frame() = default;

    std::shared_ptr<Odometry> odometry(std::shared_ptr<Frame> p_target_frame,
      const Eigen::Matrix4d &r_init_transformation = Eigen::Matrix4d::Identity());
    void process_mask(std::unique_ptr<oslam::MaskedImage> p_masked_image);

    void visualize();

    inline std::shared_ptr<open3d::geometry::RGBDImage> get_rgbd(void) const { return mp_rgbd; }
    /* inline std::vector<std::shared_ptr<open3d::geometry::RGBDImage>> get_objects(void) { return mv_object_rgbd; } */
    inline Eigen::Matrix4d get_pose(void) const { return m_pose; }
    inline void set_pose(Eigen::Matrix4d pose) { m_pose = std::move(pose); }

    inline open3d::geometry::Image get_color_image(void) const { return m_color; }

    inline bool is_keyframe() const { return m_is_keyframe; }
    inline bool has_segmentation(void) const { return mp_masked_image? true : false;}
    inline void set_segmentation(MaskedImage::UniquePtr p_masked_image)
    {
        mp_masked_image = std::move(p_masked_image);
    }

    std::vector<unsigned int> m_labels;
    std::vector<double> m_scores;
    std::vector<std::shared_ptr<open3d::geometry::RGBDImage>> mv_object_rgbd;
  private:
    std::size_t m_frame_id;
    open3d::camera::PinholeCameraIntrinsic m_intrinsic;
    //TODO(Akash): Should this be constant? For now atleast
    const bool m_is_keyframe = {false};

    open3d::geometry::Image m_color;
    open3d::geometry::Image m_depth;
    open3d::geometry::Image m_gt_mask;
    std::vector<unsigned int> m_gt_labels;
    std::vector<double> m_gt_scores;

    MaskedImage::UniquePtr mp_masked_image = {nullptr};

    Eigen::Matrix4d m_pose;

    std::shared_ptr<open3d::geometry::RGBDImage> mp_rgbd;
    std::shared_ptr<open3d::geometry::PointCloud> mp_pcd;
};
}// namespace oslam
#endif /* ifndef OSLAM_FRAME_H */
