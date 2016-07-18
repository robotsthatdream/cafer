//
// Created by phlf on 07/07/16.
//

#ifndef CAFER_CORE_DB_MANAGER_H
#define CAFER_CORE_DB_MANAGER_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <condition_variable>
#include <thread>
#include <atomic>

#include "cafer_core/cafer_core.hpp"
#include "_wave.hpp"
#include "_db_filesystem.hpp"
#include "_write_worker.hpp"

#include <cafer_core/db_manager_request.h>
#include <cafer_core/db_manager_status.h>

namespace cafer_core {

    class DatabaseManager : public cafer_core::Component {
    public:
        enum class Request : uint8_t {
            RECORD_DATA, STOP_RECORDING, REQUEST_DATA
        };

        DatabaseManager(std::string&&, std::string&&, double&&);

        ~DatabaseManager();

        void init() override;

        void client_connect_to_ros() override;

        void client_disconnect_from_ros() override;

        void update() override
        { };

    private:

        cafer_core::shared_ptr<cafer_core::Publisher> _status_publisher;
        std::unique_ptr<cafer_core::Subscriber> _request_subscriber;

        std::unique_ptr<std::thread> _send_data_thread;
        std::condition_variable _signal_send_data_thread;
        std::mutex _signal_send_data_mutex;

        std::map<uint32_t, _details::_Wave> _connected_waves;

        void _request_cb(const cafer_core::db_manager_requestConstPtr& request_msg);

        void _send_data();

        void _record_data(const uint32_t&);

        void _stop_recording(const uint32_t&);
    };
}

#endif //CAFER_CORE_DB_MANAGER_H
