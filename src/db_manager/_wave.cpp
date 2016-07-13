//
// Created by phlf on 13/07/16.
//

#include "db_manager.h"

//DatabaseManager::_Wave::_Wave(std::string& wave_name) : name(wave_name), sequential(false)
//{
//    XmlRpc::XmlRpcValue wave_data;
//    if (cafer_core::ros_nh->searchParam(wave_name, wave_data)) {
//        sequential = wave_data["sequential"];
//        data_structure = wave_data["data_structure"];
//        for (auto& topic:wave_data["topics"]["data"]) {
//            data_topics[topic.first] = topic.second;
//TODO: Defines a unique ROS message for data manager to allow dynamic managers binding.
//            _managers.push_back(std::unique_ptr(new cafer_core::ManagerQueue));
//        }
//    }
//    else {
//        ROS_WARN_STREAM("Wave " << wave_name << " not found");
//    }
//}

DatabaseManager::_Wave::_Wave(std::string& wave_name, std::map<std::string, cafer_core::Manager>& managers) : name(
        wave_name), sequential(false)
{
    XmlRpc::XmlRpcValue wave_data;
    if (cafer_core::ros_nh->searchParam(wave_name, wave_data)) {
        sequential = wave_data["sequential"];

        for (auto& topic:wave_data["topics"]["data"]) {
            data_topics[topic.first] = topic.second;

        }
    }
    else {
        ROS_WARN_STREAM("Wave " << wave_name << " not found");
    }
}

bool DatabaseManager::_Wave::no_data_left()
{
    bool no_data_left = true;

    for (const auto& manager:managers) {
        no_data_left = no_data_left && manager.second->data_size() == 0;
    }

    return no_data_left;
}

void DatabaseManager::_Wave::client_connect_to_ros()
{
    for (const auto& manager:managers) {
        manager.second->listen_to(manager.first);
    }
//    _joints_value_manager->listen_to(params["robot_controller_feedback_topic"]);
//    _rgbd_motion_data_manager->listen_to(params["motion_detector_topic"]);
//    _soi_classifier_manager->listen_to(params["soi_classifier_topic"]);
}

void DatabaseManager::_Wave::client_disconnect_from_ros()
{
    for (const auto& manager:managers) {
        manager.second->disconnect_from_ros();
    }
//    _joints_value_manager->disconnect_from_ros();
//    _rgbd_motion_data_manager->disconnect_from_ros();
//    _soi_classifier_manager->disconnect_from_ros();
}