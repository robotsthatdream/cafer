//
// Created by phlf on 11/04/16.
//
#include "db_manager.h"

using namespace cafer_core;

DatabaseManager::DatabaseManager(uint32_t& workers_pool_size, std::string&& management_topic,
                                 std::string&& cafer_type, double&& freq) : cafer_core::Component(management_topic,
                                                                                                  cafer_type, freq)
{
    for (uint32_t i = 0; i < workers_pool_size; ++i) {
        _write_requests_workers_pool.push_back(_WriteWorker());
    }
}

DatabaseManager::~DatabaseManager()
{
    client_disconnect_from_ros();

    _send_data_thread.reset();

    _status_publisher.reset();
    _request_subscriber.reset();
}

void DatabaseManager::init()
{
    //Publisher
    _status_publisher.reset(new Publisher(ros_nh->advertise<dream_babbling::db_manager_status>("status", 10)));
    //Subscriber
    _request_subscriber.reset(new Subscriber(
            ros_nh->subscribe<dream_babbling::db_manager_request>("request", 10,
                                                                  boost::bind(&DatabaseManager::_request_cb, this,
                                                                              _1))));

    _send_data_thread.reset(new std::thread(&DatabaseManager::_send_data, this));
    _is_init = true;
}

void DatabaseManager::client_connect_to_ros()
{
    for (const auto& manager:_managers) {
        manager.second->listen_to(manager.first);
    }
//    _joints_value_manager->listen_to(params["robot_controller_feedback_topic"]);
//    _rgbd_motion_data_manager->listen_to(params["motion_detector_topic"]);
//    _soi_classifier_manager->listen_to(params["soi_classifier_topic"]);
}

void DatabaseManager::client_disconnect_from_ros()
{
    for (const auto& manager: _managers) {
        manager.second->disconnect_from_ros();
    }
//    _joints_value_manager->disconnect_from_ros();
//    _rgbd_motion_data_manager->disconnect_from_ros();
//    _soi_classifier_manager->disconnect_from_ros();
}

void DatabaseManager::_send_data()
{
    std::unique_lock<std::mutex> lock(_signal_process_mutex);

    //Processing loop
    while (ros::ok()) {
        //Release _signal_process_mutex and blocks the thread.
        _signal_processing_thread.wait(lock);
        //Thread notified: acquires _signal_process_mutex and resume.

        //Define task here.
    }
}

void DatabaseManager::_request_cb(const dream_babbling::db_manager_requestConstPtr& request_msg)
{
    switch (static_cast<Request>(request_msg->request)) {
        case Request::RECORD_DATA:
            _record_data(request_msg->id);
            break;
        case Request::STOP_RECORDING:
            _stop_recording(request_msg->id);
            break;
        case Request::REQUEST_DATA:
        default:
            ROS_WARN_STREAM("Received unknown request.");
    }
}

void DatabaseManager::_record_data(uint32_t& id)
{
    //Listen to the data topics
    client_connect_to_ros();

}

void DatabaseManager::_stop_recording(uint32_t& id)
{
    //Stop listening to the data topics
    client_disconnect_from_ros();
    _connected_waves[id].is_active.store(false);
}

std::string parse_arg(int& argc, char **& argv, const std::string& default_val)
{
    std::string key;
    std::string value;
    std::string temp_str;
    std::string::size_type res;
    key = "__name:=";
    for (unsigned short i = 0; i < argc; ++i) {
        temp_str = argv[i];
        res = temp_str.find(key);
        if (res != std::string::npos) {
            value = temp_str.erase(res, key.length());
            break;
        }
        else if (i == argc - 1) {
            value = default_val;
        }
    }
    return value;
}

int main(int argc, char **argv)
{
    std::string management_topic;
    std::string cafer_type;
    std::string node_name;
    std::string freq;


    DatabaseManager db_manager(management_topic, cafer_type, std::stod(freq));
    db_manager.wait_for_init();
    while (ros::ok() && !(db_manager.get_terminate())) {
        db_manager.spin();
        db_manager.update();
        //db_manager.sleep();
    }

    return 0;
}


