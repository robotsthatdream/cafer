//
// Created by phlf on 07/07/16.
//

#include "cafer_core/db_manager.hpp"

using namespace cafer_core;

DatabaseManager::_DBFileSystem::_DBFileSystem(_Wave* parent) : _wave(parent)
{
    std::string ros_home;
    ros::get_environment_variable(ros_home, "ROS_HOME");

    _ros_home = ros_home;
}

DatabaseManager::_DBFileSystem::~_DBFileSystem()
{
    close_records();
}

void DatabaseManager::_DBFileSystem::close_records()
{
    for (auto& record:_records) {
        if (record.second.is_open()) {
            record.second.close();
        }
    }
}

void DatabaseManager::_DBFileSystem::new_records()
{
    boost::filesystem::path path;

    if (_wave->sequential) {
        do {
            path = _ros_home /
                   boost::filesystem::path("cafer_db/" + _wave->name + "/iteration_" + std::to_string(_counter) + "/");
            ++_counter;
        }
        while (boost::filesystem::exists(path));
        boost::filesystem::create_directories(path);

        for (const auto& data:_wave->data_structure) {
            _records[data.first] = std::ofstream(path.string() + data.second, std::ios::out);
        }
    }
    else {
        path = _ros_home / boost::filesystem::path("cafer_db/" + _wave->name + "/");
        if (!boost::filesystem::exists(path)) {
            boost::filesystem::create_directories(path);
        }

        for (const auto& data:_wave->data_structure) {
            _records[data.first] = std::ofstream(path.string() + data.second, std::ios::out);
        }
    }
}

void DatabaseManager::_DBFileSystem::save_data(std::unique_ptr<cafer_core::Data> data)
{
    for (auto& data_to_save:data->get_serialized_data()) {
        _records[data_to_save.first] << data_to_save.second;
    }
}

