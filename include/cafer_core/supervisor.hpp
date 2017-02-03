//Created by
//phlf on
//20/05/16.

#ifndef SUPERVISOR_HPP
#define SUPERVISOR_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include "db_manager.hpp"
#include "component.hpp"
#include "manager.hpp"
#include <cafer_core/DBManager.h>

#include <thread>
#include <chrono>
#include <random>

namespace cafer_core {
class Supervisor : public Component {
public:

    Supervisor(std::string mgmt_topic, std::string _type, double freq,std::string uuid)
            : Component(mgmt_topic, _type, freq, uuid), db_done_recording(false), _start_time(std::chrono::steady_clock::now()), _id_counter(0)
    {

    }

    ~Supervisor()
    {
        std::cout << ("Supervisor destructor.");
        client_disconnect_from_ros();
    }

    virtual void database_manager_cb(const DBManagerConstPtr& status_msg) = 0;


    void client_connect_to_ros() override
    {   
        ros_nh->getParam(ros::this_node::getNamespace(), wave_param);

        _pub_db_manager.reset(new Publisher(ros_nh->advertise<DBManager>("/cafer_core/db_manager_node/request", 10)));
        _sub_db_manager.reset(new Subscriber(ros_nh->subscribe<DBManager>("/cafer_core/db_manager_node/status", 10,
                                                 boost::bind(&Supervisor::database_manager_cb, this, _1))));

        ros_nh->getParam("components", _components);

        for (auto& packages:_components) {
            for (auto& package:packages.second) {
                for (auto& node:package.second) {
                    _parse_node_dep(node.first);
                }
            }
        }
    }

    void client_disconnect_from_ros() override
    {
        ClientDescriptor descriptor;
        std::string uuid;

        for (const auto& comp: uuid_launched_components) {
            uuid = comp.first;
            if (find_by_uuid(uuid, descriptor)) {
                send_complete_node_death(descriptor.ns, descriptor.id);
            }
        }
    }


    void init() override
    {
        client_connect_to_ros();
    }

    bool is_initialized() override
    {
        return Component::is_initialized();
    }

    XmlRpc::XmlRpcValue wave_param;

protected:
    std::unique_ptr<Publisher> _pub_db_manager;
    std::unique_ptr<Subscriber> _sub_db_manager;
    bool db_done_recording;
    std::map<std::string, std::string> uuid_launched_components;

private:
    std::vector<std::string> _launch_files_called;
    std::chrono::steady_clock::time_point _start_time;
    uint32_t _id_counter;
    XmlRpc::XmlRpcValue _components;

    void _parse_node_dep(std::string node_name)
    {
        std::string launch_file_path;
        std::string uuid;
        std::map<std::string, std::string> external_args = {{"__ns", "/"}};

        for (auto& packages:_components) {
            for (auto& package:packages.second) {
                for (auto& node:package.second) {
                    if (node_name == node.first &&
                        std::find(_launch_files_called.begin(), _launch_files_called.end(), node_name) ==
                        _launch_files_called.end()) {
                        _launch_files_called.push_back(node_name);
                        if (node.second.hasMember("depends_on")) {
                            for (const auto& dependency:_split(node.second["depends_on"], ',')) {
                                _parse_node_dep(dependency);
                            }
                        }
                        launch_file_path = ros::package::getPath(package.first) + "/launch/" +
                                           static_cast<std::string>(node.second["launch"]);
                        if ("externals" == packages.first) {
                            call_external_launch_file(launch_file_path, external_args);
                        }
                        else if ("managed" == packages.first) {
                            uuid = _gen_uuid();
                            if ("<Failed>" != call_launch_file(launch_file_path, package.first, _mgmt_topic, uuid)) {
                                uuid_launched_components[uuid] = launch_file_path;
                            }
                            else {
                                ROS_ERROR_STREAM("Failed to launch " << static_cast<std::string>(node.second["launch"]) << "!");
                            }
                        }
                    }
                }
            }
        }
    }

    std::vector<std::string>& _split(const std::string& s, char delim, std::vector<std::string>& elems)
    {
        std::stringstream ss(s);
        std::string item;
        while (getline(ss, item, delim)) {
            elems.push_back(item);
        }
        return elems;
    }

    std::vector<std::string> _split(const std::string& s, char delim)
    {
        std::vector<std::string> elems;
        _split(s, delim, elems);
        return elems;
    }

    std::string _gen_uuid()
    {
        std::stringstream uuid;
        uint32_t time_id;
        std::string id_end(4, 0);
        auto now = std::chrono::steady_clock::now();

        auto randchar = []() -> char
        {
            const char charset[] =
                    "0123456789"
                            "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                            "abcdefghijklmnopqrstuvwxyz";
            const size_t max_index = (sizeof(charset) - 1);

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(1, max_index);

            return charset[dis(gen) % max_index];
        };

        time_id = std::chrono::duration_cast<std::chrono::microseconds>(now - _start_time).count() % 1000;

        std::generate_n(id_end.begin(), 4, randchar);

        uuid << time_id << ++_id_counter << id_end;

        return uuid.str();
    }

};
}

#endif //SUPERVISOR_HPP
