/******************************************************************************
 * File:             display.h
 *
 * Author:           Akash Sharma
 * Created:          09/01/20
 * Description:      Display 3D and 2D data
 *****************************************************************************/
#ifndef OSLAM_DISPLAY_H
#define OSLAM_DISPLAY_H

#include <limits>
#include <opencv2/viz.hpp>
#include <string>

#include "object-slam/payload/display_payload.h"
#include "object-slam/payload/renderer_payload.h"
#include "object-slam/struct/frame.h"
#include "object-slam/utils/macros.h"
#include "object-slam/utils/pipeline_module.h"
#include "object-slam/utils/thread_safe_queue.h"

namespace oslam
{
    /*! \class Display
     *  \brief Brief class description
     *
     *  Detailed description
     */
    class Display : public MISOPipelineModule<DisplayInput, NullPipelinePayload>
    {
       public:
        OSLAM_POINTER_TYPEDEFS(Display);
        OSLAM_DELETE_COPY_CONSTRUCTORS(Display);
        OSLAM_DELETE_MOVE_CONSTRUCTORS(Display);

        using MISO                = MISOPipelineModule<DisplayInput, NullPipelinePayload>;
        using DisplayInputQueue   = ThreadsafeQueue<DisplayInput::UniquePtr>;
        using FrameInputQueue     = ThreadsafeQueue<Frame::UniquePtr>;

        explicit Display(const std::string& window_name);
        virtual ~Display();

        virtual OutputUniquePtr runOnce(InputUniquePtr input) override;

        inline void fillFrameQueue(Frame::UniquePtr frame) { frame_queue_.push(std::move(frame)); }
        inline void fillDisplay3dQueue(DisplayInput::UniquePtr display_input)
        {
            display_3d_queue_.push(std::move(display_input));
        }

        virtual bool hasWork() const override { return (curr_timestamp_ < max_timestamp_); }
        inline void setMaxTimestamp(Timestamp timestamp) { max_timestamp_ = timestamp; }

       private:
        virtual InputUniquePtr getInputPacket() override;

        virtual void shutdownQueues() override;

        void show2dWindow(const std::vector<NamedImage>& images_to_display) const;
        void show3dWindow(const WidgetsMap& widgets_map);

        Timestamp curr_timestamp_ = 0;
        Timestamp max_timestamp_  = std::numeric_limits<Timestamp>::max();
        const std::string window_name_;
        cv::viz::Viz3d window_3d_;  //! 3D visualization window
        cv::viz::Color background_color_;

        FrameInputQueue frame_queue_;
        DisplayInputQueue display_3d_queue_;
        DisplayInputQueue display_input_queue_;
    };
}  // namespace oslam
#endif /* ifndef OSLAM_DISPLAY_H */
