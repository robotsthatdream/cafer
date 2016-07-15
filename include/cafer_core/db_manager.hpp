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
#include <chrono>
#include <fstream>
#include <boost/filesystem.hpp>

#include "cafer_core/cafer_core.hpp"
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
        class _WriteWorker;

        class _FilesystemManager;

        class _Wave {
        public:
            std::string name;
            bool sequential;
            std::map<std::string, std::string> data_topics;
            std::map<std::string, std::string> data_structure;

            _FilesystemManager fs_manager;
            std::vector<std::unique_ptr<cafer_core::Manager>> managers;
            std::shared_ptr<cafer_core::Publisher> status_publisher;

            _Wave(std::string&, cafer_core::Publisher&);

            void add_manager(cafer_core::Manager&);

            bool no_data_left();

            void connect();

            void disconnect();

        private:
            _WriteWorker _write_worker;

        };

        class _WriteWorker {
        public:
            _WriteWorker();

            _WriteWorker(_Wave*);

            ~_WriteWorker();

            void awake_worker();

            void pause_worker();

            void link_to_wave(_Wave*);

        private:
            std::shared_ptr<_Wave> _wave;

            std::atomic<bool> _is_active{false};
            std::unique_ptr<std::thread> _processing_thread;
            std::condition_variable _signal_processing_thread;
            std::mutex _signal_process_mutex;

            void _processing();
        };

        /**
        * Class manipulating the filesystem.
        */
        class _FilesystemManager {
        public:
            _FilesystemManager(_Wave*);

            ~_FilesystemManager();

            void new_records();

            void close_records();

            void save_data(std::map<std::string, std::string>&);

        private:
            std::shared_ptr<_Wave> _wave;

            uint32_t _counter = 0;
            std::map<std::string, std::ofstream> _records;
            boost::filesystem::path _ros_home;


        };

        std::shared_ptr<cafer_core::Publisher> _status_publisher;
        std::unique_ptr<cafer_core::Subscriber> _request_subscriber;

        std::unique_ptr<std::thread> _send_data_thread;
        std::condition_variable _signal_send_data_thread;
        std::mutex _signal_send_data_mutex;

        std::map<uint32_t, _Wave> _connected_waves;

        void _request_cb(const cafer_core::db_manager_requestConstPtr& request_msg);

        void _send_data();

        void _record_data(uint32_t&);

        void _stop_recording(uint32_t&);
    };
}

#endif //CAFER_CORE_DB_MANAGER_H
