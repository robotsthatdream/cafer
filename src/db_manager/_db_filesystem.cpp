//
// Created by phlf on 07/07/16.
//

#include "cafer_core/db_manager.hpp"

#include <yaml-cpp/yaml.h>

using namespace cafer_core;

DatabaseManager::_DBFileSystem::_DBFileSystem(_Wave* parent) : _wave(parent)
{
    std::string ros_home;
    ros::get_environment_variable(ros_home, "ROS_HOME");
    _ros_home = ros_home;


    //create directory for the wave and save the wave metadata into it.
    boost::filesystem::path path = _ros_home / boost::filesystem::path("cafer_db" + _wave->name);
    ROS_INFO_STREAM("DB_MANAGER : Write directory : " << path.c_str());
    if(!boost::filesystem::exists(path))
        boost::filesystem::create_directories(path);

    std::string key;
    XmlRpc::XmlRpcValue wave_metadata;
    ros_nh->searchParam(_wave->name,key);
    ros_nh->getParam(key,wave_metadata);
    path = path / "wave_metadata.yml";
    std::ofstream ofs(path.c_str());
    ofs << _save_metadata(wave_metadata);
    ofs.close();
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

std::string DatabaseManager::_DBFileSystem::_save_metadata(XmlRpc::XmlRpcValue &metadata){
    YAML::Emitter emitter;

    std::function<void(XmlRpc::XmlRpcValue &)> rec_loop =
    [&](XmlRpc::XmlRpcValue &md){
        if(md.getType() == XmlRpc::XmlRpcValue::Type::TypeInvalid)
            return;

        emitter << YAML::BeginMap;

        for(auto it = md.begin(); it != md.end(); ++it){

            emitter << YAML::Key << it->first;

            if(it->second.getType() == XmlRpc::XmlRpcValue::Type::TypeStruct)
                rec_loop(it->second);
            else if(it->second.getType() == XmlRpc::XmlRpcValue::Type::TypeString)
                emitter << YAML::Value << static_cast<std::string>(it->second);
            else if(it->second.getType() == XmlRpc::XmlRpcValue::Type::TypeDouble)
                emitter << YAML::Value << static_cast<double>(it->second);
            else if(it->second.getType() == XmlRpc::XmlRpcValue::Type::TypeInt)
                emitter << YAML::Value << static_cast<int>(it->second);
            else if(it->second.getType() == XmlRpc::XmlRpcValue::Type::TypeBoolean)
                emitter << YAML::Value << static_cast<bool>(it->second);
        }
        emitter << YAML::EndMap;
    };
    rec_loop(metadata);

    return emitter.c_str();
}
