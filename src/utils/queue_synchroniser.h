/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   QueueSynchronizer.h
 * @brief  Implements temporal synchronization of queues.
 * @author Antoni Rosinol
 */

#pragma once

#include <spdlog/spdlog.h>

#include <memory>
#include <numeric>  // for numeric_limits
#include <string>

#include "macros.h"
#include "pipeline_payload.h"
#include "thread_safe_queue.h"
#include "timer.h"

namespace oslam
{
    // TODO: Move this to types.h file
    using Timestamp = std::uint64_t;
    /**
     * @brief The QueueSynchronizer class: a singleton class meant to
     * synchronize threadsafe queues (ThreadsafeQueue).
     */
    template<class T>
    class QueueSynchronizerBase
    {
       public:
        OSLAM_POINTER_TYPEDEFS(QueueSynchronizerBase);
        /**
         * @brief Utility function to synchronize threadsafe queues.
         *
         * @param[in] timestamp Timestamp of the payload we want to retrieve from the
         * queue
         * @param[in] queue Threadsafe queue templated on a POINTER to a class that
         * is derived from PipelinePayload (otw we cannot query what is its timestamp)
         * @param[out] pipeline_payload Returns payload to be found in the given queue
         * at the given timestamp.
         * @param[in] max_iterations Number of times we try to find the payload at the
         * given timestamp in the given queue.
         * @param[in] callback User defined function to be called on each successful
         * retrieval of a payload in the queue, the callback should be lighting fast!
         * @return a boolean indicating whether the synchronizing was successful (i.e.
         * we found a payload with the requested timestamp) or we failed because a
         * payload with an older timestamp was retrieved.
         */
        virtual bool syncQueue(const Timestamp &timestamp,
                               ThreadsafeQueue<T> *queue,
                               T *pipeline_payload,
                               std::string name_id,
                               int max_iterations                       = 10,
                               std::function<void(const T &)> *callback = nullptr) = 0;
        virtual ~QueueSynchronizerBase()                                           = default;
    };

    template<class T>
    class SimpleQueueSynchronizer : public QueueSynchronizerBase<T>
    {
       public:
        OSLAM_POINTER_TYPEDEFS(SimpleQueueSynchronizer);
        OSLAM_DELETE_COPY_CONSTRUCTORS(SimpleQueueSynchronizer);

        /**
         * @brief getInstance of a SimpleQueueSynchronizer
         * @return a unique ptr to a simple queue synchronizer which can be casted
         * to its base class for common interface as a QueueSynchronizer.
         */
        static SimpleQueueSynchronizer<T> &getInstance()
        {
            static SimpleQueueSynchronizer<T> synchronizer_instance_;
            return synchronizer_instance_;
        }

        /**
         * @brief Utility function to synchronize threadsafe queues.
         * For now we are doing a very simple naive sync approach:
         * Just loop over the messages in the queue until you find the matching
         * timestamp. If we are at a timestamp greater than the queried one
         *
         * @param[in] timestamp Timestamp of the payload we want to retrieve from the
         * queue
         * @param[in] queue Threadsafe queue templated on a POINTER to a class that
         * is derived from PipelinePayload (otw we cannot query what is its timestamp)
         * @param[out] pipeline_payload Returns payload to be found in the given queue
         * at the given timestamp.
         * @param[in] max_iterations Number of times we try to find the payload at the
         * given timestamp in the given queue.
         * @param[in] callback User defined function to be called on each successful
         * retrieval of a payload in the queue, the callback should be lighting fast!
         * @return a boolean indicating whether the synchronizing was successful (i.e.
         * we found a payload with the requested timestamp) or we failed because a
         * payload with an older timestamp was retrieved.
         */
        bool syncQueue(const Timestamp &timestamp,
                       ThreadsafeQueue<T> *queue,
                       T *pipeline_payload,
                       std::string name_id,
                       int max_iterations                       = 10,
                       std::function<void(const T &)> *callback = nullptr)
        {
            if (!queue)
            {
                spdlog::error("Queue is null");
            }
            if (!pipeline_payload)
            {
                spdlog::error("pipeline_payload is null");
            }
            static_assert(std::is_base_of<PipelinePayload, typename std::pointer_traits<T>::element_type>::value,
                          "T must be a pointer to a class that derives from PipelinePayload.");
            // Look for the synchronized packet in payload queue
            Timestamp payload_timestamp = std::numeric_limits<Timestamp>::min();
            // Loop over payload timestamps until we reach the query timestamp
            // or we are past the asked timestamp (in which case, we failed).
            int i                              = 0;
            static constexpr size_t timeout_ms = 100000U;  // Wait 1500ms at most!
            for (; i < max_iterations && timestamp > payload_timestamp; ++i)
            {
                // TODO(Toni): add a timer to avoid waiting forever...
                if (!queue->popBlockingWithTimeout(*pipeline_payload, timeout_ms))
                {
                    spdlog::error("Queue sync failed for module: {} with queue: {}\n Reason: {}",
                                  name_id,
                                  queue->queue_id_,
                                  (queue->isShutdown() ? "Shutdown" : "Timeout"));
                    return false;
                }
                if (*pipeline_payload)
                {
                    payload_timestamp = (*pipeline_payload)->timestamp_;
                    // Call any user defined callback at this point (should be fast!!).
                    if (callback)
                        (*callback)(*pipeline_payload);
                }
                else
                {
                    spdlog::warn("Payload synchronization failed. Missing payload for Module: {}", name_id);
                }
            }
            if (timestamp != payload_timestamp)
            {
                spdlog::error(
                    "Syncing queue {} in module {} failed\n Could not retrieve exact timestamp requested "
                    "- Requested timestamp {}\n- Actual timestamp {}\n",
                    queue->queue_id_,
                    name_id,
                    timestamp,
                    payload_timestamp);
                if (i >= max_iterations)
                    spdlog::error("Reached max number of sync attempts: {}", max_iterations);
            }
            return true;
        }

       private:
        // TODO(Toni): don't make this guy a singleton... Rather send it around.
        /// Non-constructible class, so make ctors private (copy ctors are deleted):
        SimpleQueueSynchronizer()  = default;
        ~SimpleQueueSynchronizer() = default;
    };

}  // namespace oslam
