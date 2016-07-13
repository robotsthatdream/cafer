//
// Created by phlf on 07/07/16.
//

#ifndef DREAM_BABBLING_DB_MANAGER_H
#define DREAM_BABBLING_DB_MANAGER_H

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <condition_variable>
#include <thread>
#include <atomic>

#include "db_filesystem.hpp"

#include <dream_babbling/pose_goalAction.h>
#include "dream_babbling/db_manager_request.h"
#include "dream_babbling/db_manager_status.h"
#include "dream_babbling/rgbd_motion_data.h"
#include "dream_babbling/soi_classifier.h"

#include "cafer_core/cafer_core.hpp"

class DatabaseManager : public cafer_core::Component {
public:
    enum class Request : uint8_t {
        RECORD_DATA, STOP_RECORDING, REQUEST_DATA
    };

    DatabaseManager(uint32_t&, std::string&&, std::string&&, double&&);

    ~DatabaseManager();

    void init() override;

    void client_connect_to_ros() override;

    void client_disconnect_from_ros() override;

    void update() override
    { };

private:
    class _Wave {
    public:
        std::string name;
        bool sequential;
        std::map<std::string, std::string> data_topics;
        std::map<std::string, std::string> data_structure;

        std::atomic<bool> is_active{false};
        FilesystemManager fs_manager;
        std::vector<std::unique_ptr<cafer_core::Manager>> managers;

        _Wave(std::string&);

        _Wave(std::string&, std::map<std::string, cafer_core::Manager>&);

        bool no_data_left();
    };

    class _WriteWorker {
    public:
        _WriteWorker();

        ~_WriteWorker();

        void _awake_worker();

        void _link_to_wave(_Wave& wave);

    private:
        std::unique_ptr<_Wave> _wave;
        std::unique_ptr<std::thread> _processing_thread;
        std::condition_variable _signal_processing_thread;
        std::mutex _signal_process_mutex;

        void _processing();
    };

    std::unique_ptr<cafer_core::Publisher> _status_publisher;
    std::unique_ptr<cafer_core::Subscriber> _request_subscriber;

    std::unique_ptr<std::thread> _send_data_thread;
    std::condition_variable _signal_send_data_thread;
    std::mutex _signal_send_data_mutex;

    std::vector<_WriteWorker> _write_requests_workers_pool;
    std::map<uint32_t, _Wave> _connected_waves;

    void _request_cb(const dream_babbling::db_manager_requestConstPtr& request_msg);

    void _send_data();

    void _record_data(uint32_t&);

    void _stop_recording(uint32_t&);
};

#endif //DREAM_BABBLING_DB_MANAGER_H
