#include <boost/python.hpp>
using namespace boost::python;

struct ComponentWrap : cafer_core::Component, wrapper<cafer_core::Component>
{

    ComponentWrap (std::string mgmt_topic, std::string _type, double freq, bool new_nodehandle) : Component(mgmt_topic, _type, freq, new_nodehandle) {}

    void update() {}

    void client_connect_to_ros() {}

    void client_disconnect_from_ros() {}

    void init() {}

    virtual bool is_initialized() 
    {
        if (override n = this->get_override("is_initialized"))
            return n();
        return Component::is_initialized();
    }
    bool default_is_initialized() 
    {
        return this->Component::is_initialized();
    }   

};

BOOST_PYTHON_MODULE(component)
{
    def("python_init", &cafer_core::python_init);
    def("python_get_node_name", &cafer_core::python_get_node_name);

    class_<ComponentWrap, boost::noncopyable>("Component", init<std::string, std::string, double, bool>())        
        .def("update", pure_virtual(&cafer_core::Component::update))
        .def("client_connect_to_ros", pure_virtual(&cafer_core::Component::client_connect_to_ros))
        .def("client_disconnect_from_ros", pure_virtual(&cafer_core::Component::client_disconnect_from_ros))
        .def("init", pure_virtual(&cafer_core::Component::init))
        .def("is_initialized", &cafer_core::Component::is_initialized, &ComponentWrap::default_is_initialized)

        .def("python_get_created_nodes_number", &cafer_core::Component::python_get_created_nodes_number)
        .def("python_print_created_nodes_id", &cafer_core::Component::python_print_created_nodes_id)
        .def("python_clients_status", &cafer_core::Component::python_clients_status)            

        .def("get_terminate", &cafer_core::Component::get_terminate)
        .def("get_type", &cafer_core::Component::get_type)
        .def("get_id", &cafer_core::Component::get_id)
        .def("is_connected_to_ros", &cafer_core::Component::is_connected_to_ros)
        .def("connect_to_ros", &cafer_core::Component::connect_to_ros)
        .def("disconnect_from_ros", &cafer_core::Component::disconnect_from_ros)
        .def("shutdown", &cafer_core::Component::shutdown)
        .def("call_launch_file", &cafer_core::Component::call_launch_file)

        .def("sleep", &cafer_core::Component::sleep)
        .def("spin", &cafer_core::Component::spin)
        .def("how_many_client_from_type", &cafer_core::Component::how_many_client_from_type)
        .def("get_connected_client_with_type", &cafer_core::Component::get_connected_client_with_type)
        .def("is_it_recent_enough", &cafer_core::Component::is_it_recent_enough)
        .def("is_client_up", &cafer_core::Component::is_client_up)
        .def("wait_for_client", &cafer_core::Component::wait_for_client)
        .def("wait_for_init", &cafer_core::Component::wait_for_init)
        .def("get_namespace", &cafer_core::Component::get_namespace)

        // .def("get_created_nodes", &cafer_core::Component::get_created_nodes)
        
        .def("kill_created_nodes", &cafer_core::Component::kill_created_nodes)
        .def("management_cb", &cafer_core::Component::management_cb)
        .def("ask_new_ack", &cafer_core::Component::ask_new_ack)
        .def("ack_creation", &cafer_core::Component::ack_creation)
        .def("update_watchdog", &cafer_core::Component::update_watchdog)
        .def("get_watchdog", &cafer_core::Component::get_watchdog)
        .def("watchdog_cb", &cafer_core::Component::watchdog_cb)
        .def("send_complete_node_death", &cafer_core::Component::send_complete_node_death)
        .def("send_local_node_death", &cafer_core::Component::send_local_node_death)
    ;

}