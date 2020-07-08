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
#include <limits>
#include <vector>

#include "map.h"
#include "mapper_payload.h"
#include "renderer_payload.h"
#include "utils/macros.h"
#include "utils/pipeline_module.h"

namespace oslam
{
    /*! \class mapper
     *  \brief Brief class description
     *
     *  Detailed description
     */
    class Mapper : public MISOPipelineModule<MapperInput, RendererInput>
    {
       public:
        OSLAM_POINTER_TYPEDEFS(Mapper);
        OSLAM_DELETE_COPY_CONSTRUCTORS(Mapper);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using MISO = MISOPipelineModule<MapperInput, RendererInput>;
        using TrackerOutputQueue = ThreadsafeQueue<TrackerOutput::UniquePtr>;
        using TransportOutputQueue = ThreadsafeQueue<ImageTransportOutput::UniquePtr>;

        Mapper(
            TrackerOutputQueue* p_tracker_output_queue,
            TransportOutputQueue* p_transport_output_queue,
            OutputQueue* p_output_queue);

        virtual ~Mapper() = default;

        virtual OutputUniquePtr run_once(InputUniquePtr p_input) override;

        virtual bool has_work() const override { return (m_curr_timestamp < m_max_timestamp); }
        virtual void set_max_timestamp(Timestamp timestamp) { m_max_timestamp = timestamp; }

       private:
        virtual InputUniquePtr get_input_packet() override;

        TrackerOutputQueue* mp_tracker_output_queue;
        TransportOutputQueue* mp_transport_output_queue;
        ImageTransportOutput::UniquePtr mp_prev_transport_output;

        GlobalMap& mr_global_map;

        Timestamp m_curr_timestamp = 0;
        Timestamp m_prev_maskframe_timestamp = 0;
        Timestamp m_max_timestamp = std::numeric_limits<Timestamp>::max();

        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> mv_T_camera_2_world;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_MAPPER_H */
