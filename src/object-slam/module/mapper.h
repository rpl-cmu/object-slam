/******************************************************************************
 * File:             mapper.h
 *
 * Author:           Akash Sharma
 * Created:          06/26/20
 * Description:      Mapper module
 *****************************************************************************/
#ifndef OSLAM_MAPPER_H
#define OSLAM_MAPPER_H

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <Eigen/Eigen>
#include <limits>
#include <vector>

#include "object-slam/utils/macros.h"
#include "object-slam/utils/pipeline_module.h"

#include "object-slam/struct/map.h"

#include "object-slam/payload/mapper_payload.h"
#include "object-slam/payload/renderer_payload.h"

namespace oslam
{
    /*! \class mapper
     *  \brief Incrementally builds the map from the incoming frames, segmentation information
     *  and the tracked camera pose.
     *  Also creates a posegraph with the object volumes, which is then optimized regularly
     */
    class Mapper : public MISOPipelineModule<MapperInput, RendererInput>
    {
       public:
        OSLAM_POINTER_TYPEDEFS(Mapper);
        OSLAM_DELETE_COPY_CONSTRUCTORS(Mapper);
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using MISO                 = MISOPipelineModule<MapperInput, RendererInput>;
        using TrackerOutputQueue   = ThreadsafeQueue<TrackerOutput::UniquePtr>;
        using TransportOutputQueue = ThreadsafeQueue<ImageTransportOutput::UniquePtr>;

        Mapper(Map::Ptr map,
               TrackerOutputQueue* tracker_output_queue,
               TransportOutputQueue* transport_output_queue,
               OutputQueue* output_queue);

        virtual ~Mapper() = default;

        virtual OutputUniquePtr runOnce(InputUniquePtr input) override;

        virtual bool hasWork() const override { return (curr_timestamp_ < max_timestamp_); }
        virtual void setMaxTimestamp(Timestamp timestamp) { max_timestamp_ = timestamp; }

       private:
        constexpr static double SCORE_THRESHOLD      = 0.5;
        constexpr static float IOU_OVERLAP_THRESHOLD = 0.2F;
        constexpr static int MASK_AREA_THRESHOLD     = 2500;
        constexpr static int BACKGROUND_RESOLUTION   = 256;
        constexpr static int OBJECT_RESOLUTION       = 128;

        virtual InputUniquePtr getInputPacket() override;

        bool shouldCreateNewBackground(Timestamp timestamp);
        static TSDFObject::Ptr createBackground(const Frame& frame, const Eigen::Matrix4d& camera_pose);
        static TSDFObject::Ptr createObject(const Frame& frame,
                                            const InstanceImage& instance_image,
                                            const Eigen::Matrix4d& camera_pose);

        void raycastMapObjects(std::vector<std::pair<ObjectId, cv::Mat>>& object_raycasts,
                               const Frame& frame,
                               const Eigen::Matrix4d& camera_pose);

        static InstanceImages::const_iterator associateObjects(const ObjectId& id,
                                                               const cv::Mat& object_raycast,
                                                               const InstanceImages& instance_images,
                                                               std::vector<bool>& instance_matches);

        Map::Ptr map_;
        ObjectId active_bg_id_;

        TrackerOutputQueue* tracker_output_queue_;
        TransportOutputQueue* transport_output_queue_;
        ImageTransportOutput::UniquePtr prev_transport_output_;

        Timestamp curr_timestamp_           = 0;
        Timestamp prev_maskframe_timestamp_ = 0;
        Timestamp max_timestamp_            = std::numeric_limits<Timestamp>::max();

        std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
            T_camera_to_world_;  //!< Camera trajectory pose w.r.t first submap

        gtsam::NonlinearFactorGraph object_pose_graph_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_MAPPER_H */
