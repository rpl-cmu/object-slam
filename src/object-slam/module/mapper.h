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
#include "object-slam/utils/thread_safe_queue.h"

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
        using ObjectRendersQueue   = ThreadsafeQueue<ObjectRenders::UniquePtr>;

        Mapper(Map::Ptr map,
               TrackerOutputQueue* tracker_output_queue,
               TransportOutputQueue* transport_output_queue,
               OutputQueue* output_queue);

        virtual ~Mapper() = default;

        virtual OutputUniquePtr runOnce(InputUniquePtr input) override;

        virtual bool hasWork() const override { return (curr_timestamp_ < max_timestamp_); }
        virtual void setMaxTimestamp(Timestamp timestamp) { max_timestamp_ = timestamp; }
        void fillRendersQueue(ObjectRenders::UniquePtr renders) { object_renders_queue_.push(std::move(renders)); }

       private:
        constexpr static double SCORE_THRESHOLD           = 0.6;
        constexpr static double IOU_OVERLAP_THRESHOLD      = 0.2;
        constexpr static float HIGH_IOU_OVERLAP_THRESHOLD = 0.3F;
        constexpr static int MASK_AREA_THRESHOLD          = 2500;
        constexpr static int BACKGROUND_RESOLUTION        = 256;
        constexpr static int OBJECT_RESOLUTION            = 128;

        virtual InputUniquePtr getInputPacket() override;

        virtual void shutdownQueues() override;
        void initializeMapAndGraph(const Frame& frame,
                                   const InstanceImages& instance_images,
                                   const Eigen::Matrix4d& camera_pose);

        void projectInstanceImages(const Timestamp& keyframe_timestamp,
                                   const Frame& frame,
                                   const InstanceImages& instance_images,
                                   InstanceImages& projected_instance_images);

        InstanceImage createBgInstanceImage(const Frame& frame,
                                            const Renders& object_renders,
                                            const InstanceImages& instance_images) const;

        bool shouldCreateNewBackground(Timestamp timestamp);
        static TSDFObject::UniquePtr createBackground(const Frame& frame, const Eigen::Matrix4d& camera_pose);
        static TSDFObject::UniquePtr createObject(const Frame& frame,
                                                  const InstanceImage& instance_image,
                                                  const Eigen::Matrix4d& camera_pose);

        InstanceImages::const_iterator associateObjects(const ObjectId& id,
                                                        const cv::Mat& object_render_color,
                                                        const InstanceImages& instance_images,
                                                        std::vector<bool>& instance_matches) const;

        void updateMap(const gtsam::Values& values);

        std::vector<bool> integrateObjects(const Renders& object_renders,
                                           const InstanceImages& frame_instance_images,
                                           const Frame& frame,
                                           const Eigen::Matrix4d& camera_pose);

        unsigned int createUnmatchedObjects(const std::vector<bool>& instance_matches,
                                            const InstanceImages& instance_images,
                                            const Frame& frame,
                                            const Eigen::Matrix4d& camera_pose);

        void addPriorFactor(const Timestamp& timestamp, const Eigen::Matrix4d& camera_pose);
        void addCameraCameraBetweenFactor(const Timestamp& time_source_camera,
                                          const Timestamp& time_target_camera,
                                          const Eigen::Matrix4d& T_source_camera_to_target_camera);

        void addObjectCameraBetweenFactor(const gtsam::Key& object_key,
                                          const Timestamp& camera_timestamp,
                                          const Eigen::Matrix4d& T_object_to_camera);

        void addCameraValue(const Timestamp& timestamp, const Eigen::Matrix4d& camera_pose);
        void addObjectValue(const gtsam::Key& object_key, const Eigen::Matrix4d& object_pose);


        Map::Ptr map_;
        ObjectId active_bg_id_;

        TrackerOutputQueue* tracker_output_queue_;
        TransportOutputQueue* transport_output_queue_;
        ObjectRendersQueue object_renders_queue_;
        ImageTransportOutput::UniquePtr prev_transport_output_;

        Timestamp curr_timestamp_ = 0;
        std::vector<Timestamp> keyframe_timestamps_;

        Timestamp max_timestamp_ = std::numeric_limits<Timestamp>::max();

        gtsam::NonlinearFactorGraph pose_graph_;
        gtsam::Values pose_values_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_MAPPER_H */
