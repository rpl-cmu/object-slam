/******************************************************************************
 * File:             mapper.h
 *
 * Author:           Akash Sharma
 * Created:          06/26/20
 * Description:      Mapper module
 *****************************************************************************/
#ifndef OSLAM_MAPPER_H
#define OSLAM_MAPPER_H

#include <Eigen/Eigen>
#include <Eigen/src/Core/util/Memory.h>

#include <vector>

#include "map.h"
#include "tracker_payload.h"
#include "utils/macros.h"
#include "utils/pipeline_module.h"
namespace oslam
{
    /*! \class mapper
     *  \brief Brief class description
     *
     *  Detailed description
     */
    class Mapper : public SISOPipelineModule<TrackerOutput, NullPipelinePayload>
    {
       public:
        OSLAM_POINTER_TYPEDEFS(Mapper);
        OSLAM_DELETE_COPY_CONSTRUCTORS(Mapper);

        using SISO = SISOPipelineModule<TrackerOutput, NullPipelinePayload>;

        Mapper(InputQueue* p_input_queue, OutputQueue* p_output_queue);
        virtual ~Mapper() = default;

        virtual OutputUniquePtr run_once(InputUniquePtr p_input) override;

       private:
        Timestamp m_curr_timestamp = 0;

        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> mv_T_camera_2_world;
        GlobalMap& mr_global_map;

        gtsam::NonlinearFactorGraph m_factor_graph;

        gtsam::Values m_values;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_MAPPER_H */
