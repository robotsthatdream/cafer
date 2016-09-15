//| This file is a part of the CAFER framework developped within
//| the DREAM project (http://www.robotsthatdream.eu/).
//| Copyright 2015, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s):
//|   * Stephane Doncieux, stephane.doncieux@isir.upmc.fr
//|
//|
//| This experiment allows to generate neural networks for simple
//| navigation tasks (obstacle avoidance and maze navigation).
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

#include <ros/ros.h>
#include <ros/spinner.h>

#include "cafer_core/cafer_core.hpp"

#include <tbb/concurrent_hash_map.h>

#include "component_python.cpp"

namespace cafer_core {

// Hashing for strings
    struct MyHashCompare {
        static size_t hash(const std::string& x)
        {
            size_t h = 0;
            for (const char* s = x.c_str(); *s; ++s) {
                h = (h * 17) ^ *s;
            }
            return h;
        }

        //! True if strings are equal
        static bool equal(const std::string& x, const std::string& y)
        {
            return x.compare(y) == 0;
        }
    };


// set of node groups allocated to this sferes run (to clean it up at the end)
//tbb::concurrent_hash_map<std::string, std::string, MyHashCompare> allocated_node_group;


    shared_ptr <ros::NodeHandle> ros_nh;

    void init(int argc, char** argv, std::string node_name)
    {
        ros::init(argc, argv, node_name);
        ros_nh.reset(new ros::NodeHandle("~"));
        ROS_INFO_STREAM("Initialising ROS. Node name: " << node_name << std::endl);
    }

    void python_init(std::string node_name)
    {
        int _argc = 0;
        char** _argv = NULL;
        init(_argc, _argv, node_name);
    }

    std::string python_get_node_name()
    {
        return ros::this_node::getName();
    }

    bool operator==(ClientDescriptor const& cd1, ClientDescriptor const& cd2)
    {
        return (cd1.ns == cd2.ns) && (cd1.id == cd2.id);
    }


}

using namespace cafer_core;


Component::Component(std::string mgmt_topic, std::string _type, double freq, std::string uuid, bool new_nodehandle)
        : terminate(false), map_watchdog(5)
{
    rate.reset(new ros::Rate(freq));
    if (new_nodehandle) {
        ROS_INFO_STREAM("creation of a dedicated callback queue");
        my_ros_nh.reset(new NodeHandle(*ros_nh.get()));
        my_ros_queue.reset(new ros::CallbackQueue());
        my_ros_nh->setCallbackQueue(my_ros_queue.get());
    }
    else {
        my_ros_nh = ros_nh;
        my_ros_queue.reset();
    }
    if (mgmt_topic == "") {
        std::string default_value = "default_" + descriptor.type;
        my_ros_nh->setParam("management_topic", default_value);
        mgmt_topic = default_value;
    }
    _mgmt_topic=my_ros_nh->resolveName(mgmt_topic);

    ROS_INFO_STREAM("Creating a component connected to management_topic: " << mgmt_topic << " type=" << _type);

    management_p.reset(new Publisher(my_ros_nh->advertise<cafer_core::Management>(mgmt_topic.c_str(), 1000)));
    management_s.reset(
            new Subscriber(my_ros_nh->subscribe(mgmt_topic.c_str(), 1000, &Component::management_cb, this)));
    watchdog.reset(new ros::Timer(
            my_ros_nh->createTimer(ros::Duration(ros::Rate(freq)), &Component::watchdog_cb, this)));

    // We get a new and unique ID for this client
    cafer_core::GetID v;
    v.request.name = "component_id";
    static ros::ServiceClient sclient = my_ros_nh->serviceClient<cafer_core::GetID>("/cafer_core/get_id");
    if (sclient.call(v)) {
        descriptor.id = v.response.id;
    }
    else {
        ROS_ERROR_STREAM("Failed to call service get_id. my namespace is: " << my_ros_nh->getNamespace());
        descriptor.id = -1;
    }
    descriptor.ns = my_ros_nh->getNamespace();
    descriptor.type = _type;
    descriptor.managed_uuid = uuid;

    //If this component has been created through a launch file, retrieve the launch file parameters.
    my_ros_nh->param("creator_id", creator_id, -1);
    my_ros_nh->param("created_ns", created_ns, std::string("<unset>"));
    my_ros_nh->param("creator_ns", creator_ns, std::string("<unset>"));

    //init();
    //ack_creation();
}


std::string
Component::call_launch_file(std::string launch_file, std::string namespace_base, std::string management_topic,
                            std::string managed_uuid)
{
    int32_t exit_value;
    std::string mgmt, cmd, ns, uuid;
    std::string created_namespace = "<Failed>";
    std::ostringstream osf;

    std::future_status status;
    std::future<int> future_ret_val;

    cafer_core::GetID id_msg;
    static ros::ServiceClient clients;

    if (management_topic == "") {
        management_topic = _mgmt_topic;
    }

    if (managed_uuid != "" || managed_uuid != "none") {
        uuid = " managed_uuid:=" + managed_uuid;
    }

    id_msg.request.name = namespace_base;
    clients = my_ros_nh->serviceClient<cafer_core::GetID>("/cafer_core/get_id");

    if (clients.call(id_msg)) {
        created_namespace = namespace_base;//+ "_" + std::to_string(id_msg.response.id);
        ns = "ns:=" + created_namespace;

        osf << "frequency:=" << 1. / rate->expectedCycleTime().toSec()
            << " creator_ns:=" << my_ros_nh->getNamespace()
            << " creator_id:=" << get_id();

        mgmt = "management_topic:=" + management_topic;
        cmd = "roslaunch " + launch_file + " " + ns + " " + osf.str() + " " + mgmt + uuid;

        std::packaged_task<int()> task([cmd]()
                                       {
                                           return std::system(cmd.c_str());
                                       });
        future_ret_val = task.get_future();
        std::thread(std::move(task)).detach();
        status = future_ret_val.wait_for(std::chrono::milliseconds(250));

        switch (status) {
            case std::future_status::timeout:
                break;
            case std::future_status::ready:
                exit_value = future_ret_val.get();

                if (exit_value != EXIT_SUCCESS) {
                    ROS_ERROR_STREAM("Failed to execute roslaunch, exit value: " << exit_value);
                    return created_namespace;
                }
                break;
            default:
                break;
        }
        ROS_INFO_STREAM("Launch file call: " << cmd);
    }
    else {
        ROS_ERROR_STREAM("Failed to call service get_id. my namespace is: " << my_ros_nh->getNamespace());
    }


    return created_namespace;
}

bool
Component::call_external_launch_file(std::string launch_file, const std::map <std::string, std::string>& subst_args)
{
    bool succeed = false;
    int32_t exit_value;
    std::string cmd;
    std::stringstream args_list("");

    std::future_status status;
    std::future<int> future_ret_val;

    for (const auto& arg:subst_args) {
        args_list << " " << arg.first << ":=" << arg.second;
    }

    cmd = "roslaunch " + launch_file + args_list.str();

    std::packaged_task<int()> task([cmd]()
                                   {
                                       return std::system(cmd.c_str());
                                   });
    future_ret_val = task.get_future();
    std::thread(std::move(task)).detach();

    status = future_ret_val.wait_for(std::chrono::milliseconds(250));
    switch (status) {
        case std::future_status::timeout:
            ROS_INFO_STREAM("Launch file call: " << cmd);
            break;
        case std::future_status::ready:
            exit_value = future_ret_val.get();
            if (exit_value != EXIT_SUCCESS) {
                ROS_ERROR_STREAM("Failed to execute roslaunch, exit value: " << exit_value);
            }
            else {
                succeed = true;
            }
            break;
        default:
            break;
    }

    return succeed;
}

void Component::spin()
{
    if (my_ros_queue.get() == NULL) {
        ros::spinOnce();
    }
    else {
        my_ros_queue->callAvailable(ros::WallDuration());
    }

}

unsigned int Component::how_many_client_from_type(std::string _type, bool up_only)
{
    unsigned int nb = 0;
    //ROS_INFO_STREAM("how_many_client_from_type: "<<_type<<" my id="<<get_id());
    for (const auto& v: map_watchdog) {
        if ((v.first.type == _type) && ((!up_only) || (is_it_recent_enough(v.second)))) {
            nb++;
        }
    }
    return nb;
}

void Component::get_connected_client_with_type(std::string _type, std::vector <ClientDescriptor>& vcd, bool up_only)
{
    for (const auto& v: map_watchdog) {
        if ((v.first.type == _type) && ((!up_only) || (is_it_recent_enough(v.second)))) {
            vcd.push_back(v.first);
        }
    }

}

bool Component::is_it_recent_enough(ros::Time t)
{

    ros::Duration d = ros::Time::now() - t;
    return d <= (rate->expectedCycleTime() * 2.0);

}

void Component::wait_for_client(std::string ns, long id)
{

    while (!is_client_up(ns, id)) {
        spin();
        sleep();
    }
    update();
}

void Component::wait_for_init()
{
    init();

    while ((!is_client_up(get_namespace(), get_id())) && (!is_initialized())) {
        ROS_INFO_STREAM("Component id=" << get_id() << " waiting for init.");
        spin();
        sleep();
    }

    ack_creation();

    update();

}

void Component::kill_created_nodes()
{
    for (const auto& v: created_nodes) {
        for (const auto& cd: v.second) {
            send_complete_node_death(cd.ns, cd.id);
        }
    }
}

void Component::management_cb(const Management& mgmt)
{
    //ROS_INFO_STREAM("management_cb my_id="<<get_id()<<" message: "<<std::endl<<mgmt<<std::flush);
    switch (static_cast<MgmtType>(mgmt.type)) {
        case MgmtType::CHG_FREQ:
            ROS_INFO_STREAM("Changing frequency: new frequency=" << mgmt.data_flt << " my_id=" << get_id());
            rate.reset(new ros::Rate(mgmt.data_flt));
            watchdog.reset(new ros::Timer(
                    my_ros_nh->createTimer(ros::Duration(ros::Rate(mgmt.data_flt)), &Component::watchdog_cb,
                                           this)));
            break;
        case MgmtType::LOCAL_CLIENT_DEATH:
            if ((mgmt.dest_node == "all") ||
                ((mgmt.dest_node == my_ros_nh->getNamespace()) && (mgmt.dest_id == get_id()))) {
                ROS_INFO_STREAM("LOCAL_CLIENT_DEATH");
                terminate = true;
            }
            break;
        case MgmtType::COMPLETE_NODE_DEATH:
            if ((mgmt.dest_node == "all") ||
                ((mgmt.dest_node == my_ros_nh->getNamespace()) && (mgmt.dest_id == get_id()))) {
                ROS_INFO_STREAM("COMPLETE_NODE_DEATH called");
                shutdown();
            }
            break;
        case MgmtType::WATCHDOG:
            update_watchdog(mgmt.src_node, mgmt.src_id, mgmt.src_type, mgmt.data_str);
            break;
        case MgmtType::ACK_CREATION:
            if ((mgmt.dest_node == "all") ||
                ((mgmt.dest_node == my_ros_nh->getNamespace()) && (mgmt.dest_id == get_id()))) {
                ROS_INFO_STREAM(
                        "Ack received by the creator: mgmt.dest_node=" << mgmt.dest_node << " mgmt.dest_id=" <<
                                                                       mgmt.dest_node << " my_ns="
                                                                       << my_ros_nh->getNamespace() << " my_id="
                                                                       << get_id() <<
                                                                       " src_ns=" << mgmt.src_node << " src_id="
                                                                       << mgmt.src_id << " src_type=" <<
                                                                       mgmt.src_type);
                ClientDescriptor cd;
                cd.ns = mgmt.src_node;
                cd.id = mgmt.src_id;
                cd.type = mgmt.src_type;
                std::vector<ClientDescriptor>::iterator it = std::find(created_nodes[mgmt.data_str].begin(),
                                                                       created_nodes[mgmt.data_str].end(), cd);
                if (it == created_nodes[mgmt.data_str].end()) {
                    created_nodes[mgmt.data_str].push_back(cd);
                }
            }
            break;
        case MgmtType::ASK_NEW_ACK:
            ack_creation();
            break;
        default:
            ROS_WARN_STREAM("component: received unknown message: type=" << mgmt.type);
    }
}

void Component::ask_new_ack()
{
    cafer_core::Management msg;
    msg.type = static_cast<uint8_t>(MgmtType::ASK_NEW_ACK);
    msg.src_node = my_ros_nh->getNamespace();
    msg.src_id = get_id();
    msg.src_type = get_type();
    msg.dest_node = "all";
    msg.dest_id = -1;
    msg.data_int = 0;
    msg.data_flt = 0;
    msg.data_str = "";
    management_p->publish(msg);
}

void Component::ack_creation()
{
    if (creator_id != -1) {
        cafer_core::Management msg;
        msg.type = static_cast<uint8_t>(MgmtType::ACK_CREATION);
        msg.src_node = my_ros_nh->getNamespace();
        msg.src_id = get_id();
        msg.src_type = get_type();
        msg.dest_node = creator_ns;
        msg.dest_id = creator_id;
        msg.data_int = 0;
        msg.data_flt = 0;
        msg.data_str = created_ns;
        // We may need to wait a bit so that the management publisher is connected.
        ROS_INFO_STREAM(
                "ACK_CREATION: waiting for the connection to the creator (ns=" << creator_ns << " id=" <<
                                                                               creator_id << ").my_id="
                                                                               << get_id());
        wait_for_client(creator_ns, creator_id);
        ROS_INFO_STREAM("ACK_CREATION: connection to the creator OK. my_id=" << get_id());
        management_p->publish(msg);
        ROS_INFO_STREAM("Sending ack after component creation: creator_ns=" << creator_ns << " creator_id=" <<
                                                                            creator_id << " created_ns="
                                                                            << created_ns);
    }
    else {
        ROS_WARN_STREAM(my_ros_nh->getNamespace() << ": No creator id provided, no ack has been sent.");
    }

}

void Component::update_watchdog(std::string ns, long id, std::string _type, std::string _uuid)
{
    ClientDescriptor cd;
    cd.ns = ns;
    cd.id = id;
    cd.type = _type;
    cd.managed_uuid = _uuid;
    map_watchdog[cd] = ros::Time::now();
    //ROS_INFO_STREAM("update_watchdog "<<ns<<" "<<id<<" "<<_type<<" my_id="<<get_id());
}

ros::Time Component::get_watchdog(std::string ns, long id)
{
    ClientDescriptor cd;
    cd.ns = ns;
    cd.id = id;
    cd.type = "undefined";
    if (map_watchdog.find(cd) == map_watchdog.end()) {
        //ROS_INFO_STREAM("get_watchdog uninitialized "<<ns<<" "<<id<<" time="<<ros::Time(0)<<" my id="<<get_id());
        return ros::Time(0);

    }
    //ROS_INFO_STREAM("get_watchdog initialized "<<ns<<" time="<<map_watchdog[cd]<<" id="<<id<<" my id="<<get_id());
    return map_watchdog[cd];

}

void Component::watchdog_cb(const ros::TimerEvent& event)
{
    //ROS_INFO_STREAM("watchdog_cb my_id="<<get_id());
    //ROS_INFO_STREAM("watchdog_cb, management topic="<<management_p->getTopic()<<" nb connected="<<management_p->getNumSubscribers()<<std::flush);
    cafer_core::Management msg;
    msg.type = static_cast<uint8_t>(MgmtType::WATCHDOG);
    msg.src_node = descriptor.ns;
    msg.src_id = get_id();
    msg.src_type = get_type();
    msg.dest_node = "all";
    msg.dest_id = -1;
    msg.data_int = 0;
    msg.data_flt = 0;
    msg.data_str = descriptor.managed_uuid;
    management_p->publish(msg);
}

void Component::send_complete_node_death(std::string ns, long id)
{
    ROS_INFO_STREAM("send_complete_node_death my_id=" << get_id());
    cafer_core::Management msg;
    msg.type = static_cast<uint8_t>(MgmtType::COMPLETE_NODE_DEATH);
    msg.src_node = my_ros_nh->getNamespace();
    msg.src_id = get_id();
    msg.src_type = get_type();
    msg.dest_node = ns;
    msg.dest_id = id;
    msg.data_int = 0;
    msg.data_flt = 0;
    msg.data_str = "";
    management_p->publish(msg);
}

void Component::send_local_node_death(std::string ns, long id)
{

    ROS_INFO_STREAM("send_local_node_death my_id=" << get_id());
    cafer_core::Management msg;
    msg.type = static_cast<uint8_t>(MgmtType::LOCAL_CLIENT_DEATH);
    msg.src_node = my_ros_nh->getNamespace();
    msg.src_id = get_id();
    msg.src_type = get_type();
    msg.dest_node = ns;
    msg.dest_id = id;
    msg.data_int = 0;
    msg.data_flt = 0;
    msg.data_str = "";
    management_p->publish(msg);

}

bool Component::find_by_id(uint32_t& id, ClientDescriptor& returned_descriptor)
{
    bool found = false;

    for (auto& descriptor:map_watchdog) {
        if (descriptor.first.id == id) {
            found = true;

            returned_descriptor.id = id;
            returned_descriptor.ns = descriptor.first.ns;
            returned_descriptor.type = descriptor.first.type;
        }
    }
    return found;
}

bool Component::find_by_name(std::string& name, ClientDescriptor& returned_descriptor)
{
    bool found = false;

    for (auto& descriptor:map_watchdog) {
        if (descriptor.first.ns == name) {
            found = true;

            returned_descriptor.id = descriptor.first.id;
            returned_descriptor.ns = name;
            returned_descriptor.type = descriptor.first.type;
        }
    }
    return found;
}

bool Component::find_by_uuid(std::string& uuid, ClientDescriptor& returned_descriptor)
{
    bool found = false;

    for (auto& descriptor:map_watchdog) {
        if (descriptor.first.managed_uuid == uuid) {
            found = true;

            returned_descriptor.id = descriptor.first.id;
            returned_descriptor.ns = descriptor.first.ns;
            returned_descriptor.type = descriptor.first.type;
            returned_descriptor.managed_uuid = uuid;
        }
    }
    return found;
}

int Component::python_get_created_nodes_number()
{
    return created_nodes.size();
}

void Component::python_print_created_nodes_id()
{
    // std::vector<int> node_id_vector;
    for (auto& v: created_nodes) {
        for (auto& cd: v.second) {
            ROS_INFO_STREAM("Component: id=" << cd.id << " ns=" << cd.ns);
        }
    }
    return;
}

bool Component::python_clients_status(std::string type)
{
    if (type.compare("up") == 0) {
        bool all_up = true;
        for (auto& v: created_nodes) {
            for (auto& cd: v.second) {
                all_up = all_up && this->is_client_up(cd.ns, cd.id);
            }
        }
        return all_up;
    }

    else if (type.compare("down") == 0) {
        bool all_down = true;
        for (auto& v: created_nodes) {
            for (auto cd:v.second) {
                all_down = all_down && !this->is_client_up(cd.ns, cd.id);
            }
        }
        return all_down;
    }

    else {
        ROS_ERROR("python_clients_status was called with a wrong value");
        exit(EXIT_FAILURE);
    }
}

// std::string get_node_group(std::string namespace_base, std::string launch_file, double frequency=30) {
//   std::cout<<"Trying to get a node: "<<namespace_base<<" launch file: "<<launch_file<<std::endl;
//   cafer_server::LaunchNode v;
//   v.request.namespace_base = namespace_base;
//   v.request.launch_file = launch_file;
//   v.request.frequency = frequency;
//   ros::ServiceClient client = ros_nh->serviceClient<cafer_server::LaunchNode>("/cafer_server/get_node_group");
//   std::string ns="<Failed>";
//   if (client.call(v))
//     {
// 	ns=v.response.created_namespace;
// 	ROS_INFO("Obtained node(s): namespace=%s launch file=%s", ns.c_str(), launch_file.c_str());
// 	tbb::concurrent_hash_map<std::string,std::string,MyHashCompare>::accessor a;
// 	allocated_node_group.insert(a,ns);
// 	a->second=namespace_base;
//     }
//   else
//     {
// 	ROS_ERROR("Failed to call service get_node_group");
//     }
//   return ns;
// }

// void release_node_group(std::string namespace_base, std::string gr_namespace) {
//   cafer_server::ReleaseNode v;
//   v.request.namespace_base=namespace_base;
//   v.request. gr_namespace=gr_namespace;
//   ros::ServiceClient client = ros_nh->serviceClient<cafer_server::ReleaseNode>("/cafer_server/release_node_group");
//   std::string ns="<Failed>";
//   if (client.call(v))
//     {
// 	ROS_INFO("Released node group=%s: %s", gr_namespace.c_str(),v.response.ack?"OK":"KO");
//     }
//   else
//     {
// 	ROS_ERROR("Failed to call service release_node_group");
//     }

// }
// void kill_node_group(std::string namespace_base, std::string gr_namespace) {
//   cafer_server::KillNodeGroup v;
//   v.request.namespace_base=namespace_base;
//   v.request.gr_namespace=gr_namespace;
//   ros::ServiceClient client = ros_nh->serviceClient<cafer_server::KillNodeGroup>("/cafer_server/kill_node_group");
//   std::string ns="<Failed>";
//   if (client.call(v))
//     {
// 	ROS_INFO("Kill node group=%s: %s", gr_namespace.c_str(),v.response.ack?"OK":"KO");
//     }
//   else
//     {
// 	ROS_ERROR("Failed to call service kill_node_group");
//     }

// }

// void kill_all_allocated_node_groups(void) {
//   tbb::concurrent_hash_map<std::string,std::string,MyHashCompare>::iterator it,itn;
//   for (it=allocated_node_group.begin();it!=allocated_node_group.end();++it) {
//     kill_node_group(it->second,it->first);
//   }
// }
