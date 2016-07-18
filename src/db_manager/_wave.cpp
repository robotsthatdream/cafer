//
// Created by phlf on 13/07/16.
//

#include "cafer_core/db_manager/_wave.hpp"

using namespace cafer_core;

//_details::_Wave::_Wave(std::string& wave_name) : name(wave_name), sequential(false)
//{
//    XmlRpc::XmlRpcValue wave_data;
//    if (cafer_core::ros_nh->searchParam(wave_name, wave_data)) {
//        sequential = wave_data["sequential"];
//        data_structure = wave_data["data_structure"];
//        for (auto& topic:wave_data["topics"]["data"]) {
//            data_topics[topic.first] = topic.second;
//TODO: Defines a unique ROS message for the data manager to allow dynamic managers binding.
//            _managers.push_back(std::unique_ptr(new cafer_core::ManagerQueue));
//        }
//    }
//    else {
//        ROS_WARN_STREAM("Wave " << wave_name << " not found");
//    }
//}

_details::_Wave::_Wave(std::string& wave_name, cafer_core::Publisher& publisher) : name(wave_name),
                                                                                   sequential(false),
                                                                                   fs_manager(this),
                                                                                   status_publisher(publisher)
{
    XmlRpc::XmlRpcValue wave_data;
    if (cafer_core::ros_nh->searchParam(wave_name, wave_data)) {
        sequential = wave_data["sequential"];

        for (auto& topic:wave_data["topics"]["data"]) {
            data_topics[topic.first] = static_cast<std::string>(topic.second);
        }
        for (auto& record:wave_data["data_structure"]) {
            data_structure[record.first] = static_cast<std::string>(record.second);
        }

    }
    else {
        ROS_WARN_STREAM("Wave " << wave_name << " not found");
    }

    _write_worker.link_to_wave(this);
}

void _details::_Wave::add_manager(cafer_core::IManager& manager)
{
    managers.push_back(std::unique_ptr<cafer_core::IManager>(&manager));
}

bool _details::_Wave::no_data_left()
{
    bool no_data_left = true;

    for (const auto& manager:managers) {
        no_data_left = no_data_left && manager->data_size() == 0;
    }

    return no_data_left;
}

void _details::_Wave::connect()
{
    for (const auto& manager:managers) {
        manager->listen_to();
    }
    _write_worker.awake_worker();
}

void _details::_Wave::disconnect()
{
    for (const auto& manager:managers) {
        manager->disconnect_from_ros();
    }
    _write_worker.pause_worker();
}