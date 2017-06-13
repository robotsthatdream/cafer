//
// Created by phlf on 07/07/16.
//

#ifndef CAFER_CORE_DB_MANAGER_H
#define CAFER_CORE_DB_MANAGER_H

#include <ros/ros.h>

#include <condition_variable>
#include <thread>
#include <atomic>
#include <fstream>
#include <memory>

#include <boost/filesystem.hpp>

#include <cafer_core/DBManager.h>

#include "component.hpp"
#include "data.hpp"
#include "manager.hpp"

namespace cafer_core {

    /**
     * A class representing a Database Manager object acting as a long-term memory.
     */
    class DatabaseManager : public Component {
    public:
        /**
         * An enum representing request values handled by the Database Manager.
         */
        enum class Request : uint8_t {
            RECORD_DATA, STOP_RECORDING, REQUEST_DATA, ASK_STATUS
        };

        /**
         * An enum representing values of answer send by the Database Manager.
         */
        enum class Response : uint8_t {
            STATUS_READY, STATUS_ACTIVE, ERROR, DATA
        };

        class _Wave;

        using Component::Component;

        typedef std::shared_ptr<DatabaseManager> Ptr;
        typedef std::shared_ptr<const DatabaseManager> ConstPtr;

        /**
         * Wait for the _send_data_thread to terminate and destroys the object.
         */
        ~DatabaseManager();

        void init() override;

        void client_connect_to_ros() override;

        void client_disconnect_from_ros() override;

        void update() override
        {}

        /**
         * Connects the DatabaseManager to a new wave.
         * @param name The name of the wave to add. It must matches an actual absolute(including ns) ROS node name.
         */
        bool add_wave(std::string name);

        /**
         * Search in the _connected_waves map for a wave with the matching name.
         * @param name The name of the wave to look for (absolute ROS node name).
         * @param waves_ptr A pointer referring to the found _Wave object.
         * @return True if a wave was found, false otherwise.
         */
        bool find_wave_by_name(std::string name, shared_ptr<_Wave>& wave_ptr);

    private:

        /**
        * @brief A class representing an interface to the filesystem to store records.
        * @details This class use files as records.
        * It could be replaced by a class representing a DB client (SQL Client for instance), using tables as records.
        */
        class _DBFileSystem {
        public:
            /**
             * Constructs a _DBFileSystem object, links it to a _Wave object and get $ROS_HOME as the DB location.
             * @param parent The _Wave object to bind to.
             */
            _DBFileSystem(_Wave* parent);

            /**
             * Closes all opened filestreams and destroys the _DBFileSystem object.
             */
            ~_DBFileSystem();

            /**
             * Creates and opens a new filestream for each entry defined in the datastructure parameter of the binded _Wave.
             */
            void new_records();

            /**
             * Close all opened filestreams.
             */
            void close_records();

            /**
             * Stores the content of a Data object in the corresponding filestreams.
             */
            void save_data(std::unique_ptr<cafer_core::Data>);

        private:
            _Wave* _wave;

            uint32_t _counter = 0;
            std::map<std::string, std::ofstream> _records;
            boost::filesystem::path _ros_home;

            std::string _save_metadata(XmlRpc::XmlRpcValue &metadata);

        };

        /**
         * A class representing a worker to handle the binded _Wave's data.
         */
        class _WriteWorker {
        public:
            /**
             * Constructs a _WriteWorker object.
             */
            _WriteWorker();

            /**
             * Constructs a _WriteWorker object and links it to a _Wave object.
             * @param parent The _Wave object to bind to.
             */
            _WriteWorker(_Wave* parent);

            /**
             * Waits for the processing thread to end and destroys the object.
             */
            ~_WriteWorker();

            /**
             * Awakes the processing thread.
             */
            void awake_worker();

            /**
             * Pauses the processing thread. All remaining data will be processed before suspending the thread.
             */
            void pause_worker();

            /**
             * Binds the _WriteWorker to a _Wave object and creates a new thread to process the wave's data.
             * @param parent The _Wave object to bind to.
             */
            void link_to_wave(_Wave* parent);

        private:
            _Wave* _wave;

            std::atomic<bool> _is_active{false};
            std::atomic<bool> _finish{false};
            std::unique_ptr<std::thread> _processing_thread;
            std::condition_variable _signal_processing_thread;
            std::mutex _signal_process_mutex;

            void _processing();
        };
    protected:
        uint32_t _requester_id;
        std::string _data_request;

        cafer_core::shared_ptr<cafer_core::Publisher> _status_publisher;
        std::unique_ptr<cafer_core::Subscriber> _request_subscriber;

        std::unique_ptr<std::thread> _send_data_thread;
        std::condition_variable _signal_send_data_thread;
        std::mutex _signal_send_data_mutex;

        std::map<uint32_t, cafer_core::shared_ptr<_Wave>> _connected_waves;

        /**
         * The ROS callback to handle request for a DatabaseManager object.
         * @param request_msg The request to be handled.
         */
        void _request_cb(const DBManagerConstPtr& request_msg);

        /**
         * The function used by the thread _send_data_thread dedicated to reply to the requests.
         */
        void _send_data();

        /**
         * Records data of the requesting wave.
         * @param id Id of the wave.
         */
        bool _record_data(const uint32_t& id);

        /**
         * Stops recording data of the requesting wave.
         * @param id Id of the wave.
         */
        bool _stop_recording(const uint32_t& id);

        /**
         * Sends the DatabaseManager status to the requesting wave.
         * @param id Id of the wave.
         */
        bool _status_request(const uint32_t& id);

        /**
         * Search in the _connected_waves map for waves of the matching type.
         * @param type The type of the wave as defined in CAFER interface.
         * @param waves_uris A vector of strings to be filled with the found waves' URIs (ROS namespaces).
         * @return True if waves were found, false otherwise.
         */
        bool _find_waves_by_type(std::string& type, std::vector<std::string>& waves_uris);

    public:

        /**
         * A class representing a _Wave object.
         * This object is used as an interface for the DatabaseManager to handle waves.
         */
        class _Wave {
        public:
            uint32_t id;
            const std::string name;
            std::string type;
            bool sequential;
            bool ready;
            std::map<std::string, std::string> data_topics;
            std::map<std::string, std::string> data_structure;

            _DBFileSystem fs_manager;
            std::vector<std::unique_ptr<cafer_core::IManager>> managers;

            /**
             * Move constructor for the _Wave class.
             * @param moved_wave The moved _Wave instance from which to contruct a new one.
             */
            _Wave(_Wave&& moved_wave);

            /**
             * Constructs a _Wave object from an existing wave.
             * @param id_ The id of the wave as an existing Component.
             * @param wave_name The name of the wave to look for (absolute ROS node name).
             * @param publisher A pointer referring to the DatabaseManager _status_publisher.
             */
            _Wave(uint32_t id_, std::string& wave_name);

            /**
             * Adds a manager to listen to a data topic.
             * @param manager An pointer to an object implementing the IManager interface.
             * @param topic The topic to link the manager to.
             */
            void add_manager(cafer_core::IManager* manager, std::string topic = "");

            /**
             * Checks if there is still data to be processed.
             * @return True if there is data remaining, false otherwise.
             */
            bool no_data_left();

            /**
             * Connects all the _Wave object's managers to ROS and awakes the worker.
             */
            void connect();

            /**
             * Disconnects all the _Waves object's managers from ROS and suspends the worker.
             */
            void disconnect();

        private:
            _WriteWorker _write_worker;

        };

    };
}

#endif //CAFER_CORE_DB_MANAGER_H
