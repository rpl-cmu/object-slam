/******************************************************************************
 * File:             masked_image.h
 *
 * Author:           Akash Sharma
 * Created:          05/16/20
 * Description:      Masked Image payload
 *****************************************************************************/
#ifndef OSLAM_MASKED_IMAGE_H
#define OSLAM_MASKED_IMAGE_H

#include <Open3D/Open3D.h>
#include <opencv2/core/mat.hpp>

#include "utils/pipeline_payload.h"

namespace oslam {

/*! \class MaskedImage
 *  \brief Brief class description
 *
 *  Detailed description
 */
struct MaskedImage : public PipelinePayload
{
    typedef std::vector<std::shared_ptr<open3d::geometry::RGBDImage>> ObjectRGBDVector;

  public:
    OSLAM_POINTER_TYPEDEFS(MaskedImage);

    explicit MaskedImage(const Timestamp timestamp,
      const open3d::geometry::Image &r_image,
      const std::vector<unsigned int> &r_labels,
      const std::vector<double> &r_scores);
    ~MaskedImage() = default;

    MaskedImage(const MaskedImage &r_masked_image)
      : PipelinePayload(r_masked_image.m_timestamp), m_image(r_masked_image.m_image),
        m_labels(r_masked_image.m_labels), m_scores(r_masked_image.m_scores)
    {}

    //! Generate separate object masks from the masked image
    std::vector<cv::Mat> process(void);

  public:
    open3d::geometry::Image m_image;
    std::vector<unsigned int> m_labels;
    std::vector<double> m_scores;
};
}// namespace oslam

#endif /* ifndef OSLAM_MASKED_IMAGE_H */
