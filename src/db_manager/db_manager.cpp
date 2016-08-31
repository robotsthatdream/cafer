//
// Created by phlf on 11/04/16.
//

#include "cafer_core/db_manager.hpp"

using namespace cafer_core;

DatabaseManager::~DatabaseManager()
{
    client_disconnect_from_ros();

    _signal_send_data_mutex.lock();
    _signal_send_data_thread.notify_one();
    _signal_send_data_mutex.unlock();
    _send_data_thread->join();

    _send_data_thread.reset();

    _status_publisher.reset();
    _request_subscriber.reset();
}

void DatabaseManager::init()
{
    //Publisher
    _status_publisher.reset(new Publisher(ros_nh->advertise<DBManager>("status", 10)));
    //Subscriber
    _request_subscriber.reset(new Subscriber(
            ros_nh->subscribe<DBManager>("request", 10, boost::bind(&DatabaseManager::_request_cb, this, _1))));

    _send_data_thread.reset(new std::thread(&DatabaseManager::_send_data, this));

    _is_init = true;
}

void DatabaseManager::client_connect_to_ros()
{
    for (auto& wave:_connected_waves) {
        wave.second->connect();
    }
}

void DatabaseManager::client_disconnect_from_ros()
{
    for (auto& wave:_connected_waves) {
        wave.second->disconnect();
    }
}

void DatabaseManager::_send_data()
{
    std::unique_lock<std::mutex> lock(_signal_send_data_mutex);
    DBManager db_manager_response;
    std::vector<std::string> waves_uris;

    //Processing loop
    while (ros::ok()) {
        waves_uris.clear();
        //Release _signal_process_mutex and blocks the thread.
        _signal_send_data_thread.wait(lock);
        //Thread notified: acquires _signal_process_mutex and resume.

        db_manager_response.id = _requester_id;
        if (_find_waves_by_type(_data_request, waves_uris)) {
            db_manager_response.type = static_cast<uint8_t>(Response::DATA);
            db_manager_response.data = waves_uris;
        }
        else {
            db_manager_response.type = static_cast<uint8_t>(Response::ERROR);
            db_manager_response.data.emplace_back(std::string("No wave of requested type: " + _data_request + " !"));
        }
        _status_publisher->publish(db_manager_response);

    }
}

void DatabaseManager::_request_cb(const DBManagerConstPtr& request_msg)
{
    switch (static_cast<Request>(request_msg->type)) {
        case Request::RECORD_DATA:
            ROS_INFO_STREAM("Received record request from Wave " << request_msg->id);
            _record_data(request_msg->id);
            break;
        case Request::STOP_RECORDING:
            ROS_INFO_STREAM("Received stop recording request from Wave " << request_msg->id);
            _stop_recording(request_msg->id);
            break;
        case Request::REQUEST_DATA:
            ROS_INFO_STREAM("Received data request from Wave " << request_msg->id);

            _signal_send_data_mutex.lock();
            _requester_id = request_msg->id;
            _data_request = request_msg->data[0];
            _signal_send_data_thread.notify_one();
            _signal_send_data_mutex.unlock();
            break;
        case Request::ASK_STATUS:
            ROS_INFO_STREAM("Received status request from Wave " << request_msg->id);
            _status_request(request_msg->id);
            break;
        default:
            ROS_WARN_STREAM("Received unknown request.");
    }
}

void DatabaseManager::_record_data(const uint32_t& id)
{
    //TODO: In a dynamic scenario call add_waves() for new waves that connect to the DB Manager.
    auto wave = _connected_waves.find(id);
    if (wave == _connected_waves.end()) {
        ROS_WARN_STREAM("Unable to find wave " << id);
    }
    else {
        wave->second->connect();
    }
}

void DatabaseManager::_stop_recording(const uint32_t& id)
{
    auto wave = _connected_waves.find(id);
    if (wave == _connected_waves.end()) {
        ROS_WARN_STREAM("Unable to find wave " << id);
    }
    else {
        wave->second->disconnect();
    }
}

void DatabaseManager::_status_request(const uint32_t& id)
{
    DBManager db_status_msg;
    db_status_msg.id = id;

    auto wave = _connected_waves.find(id);
    if (wave == _connected_waves.end()) {
        ROS_WARN_STREAM("Unable to find wave " << id);
    }
    else {
        if (wave->second->ready) {
            db_status_msg.type = static_cast<uint8_t>(Response::STATUS_READY);
        }
        else {
            db_status_msg.type = static_cast<uint8_t>(Response::STATUS_ACTIVE);
        }

        _status_publisher->publish(db_status_msg);
    }
}

bool DatabaseManager::add_wave(std::string name)
{
    bool succeed = false;
    ClientDescriptor descriptor;
    shared_ptr<_Wave> wave_ptr;

    if (find_by_name(name, descriptor)) {
        auto wave_it = _connected_waves.find(descriptor.id);

        if (wave_it == _connected_waves.end()) {
            wave_ptr.reset(new _Wave(descriptor.id, name));
            _connected_waves.emplace(descriptor.id, wave_ptr);
            succeed = true;
        }
        else {
            ROS_WARN_STREAM("The wave " << name << " is already linked to the DB_Manager!");
        }
    }
    else {
        ROS_WARN_STREAM("The wave " << name << " doesn't exist in the experiment!");
    }

    return succeed;
}

bool DatabaseManager::_find_waves_by_type(std::string& type, std::vector<std::string>& waves_uris)
{
    bool succeed = false;

    for (const auto& wave:_connected_waves) {
        if (wave.second->type == type) {
            waves_uris.push_back(wave.second->name);
            succeed = true;
        }
    }

    return succeed;
}

bool DatabaseManager::find_wave_by_name(std::string name, shared_ptr<DatabaseManager::_Wave>& wave_ptr)
{
    bool found = false;
    wave_ptr = nullptr;
    for (auto& wave:_connected_waves) {
        if (wave.second->name == name) {
            wave_ptr = wave.second;
        }
    }
    return found;
}


