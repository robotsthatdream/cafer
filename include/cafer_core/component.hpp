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

#ifndef _SFERES_CAFER_HPP
#define _SFERES_CAFER_HPP

#include <unordered_map>
#include <functional>

#include "cafer_core/Management.h"
#include "cafer_core/GetID.h"
#include "cafer_core/aliases.hpp"

#include <ros/ros.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>


namespace cafer_core {

    void init(int argc, char **argv, std::string node_name);
    // std::string get_node_group(std::string namespace_base, std::string launch_file, double frequency);
    // void release_node_group(std::string namespace_base, std::string gr_namespace);
    // void kill_node_group(std::string namespace_base, std::string gr_namespace);
    // void kill_all_allocated_node_groups(void);

    /** Definition of message types
    * - CHG_FREQ: changement of the ROS frequency
    * - LOCAL_CLIENT_DEATH: ask for the death of the component. It is local as a node may have several components and what is asked here is only the death of the component. It should be noted that it only sets a boolean attribute (terminate) to true. Its value has to be taken into accounf for anything to happen...
    * - COMPLETE_NODE_DEATH: ask for the whole node death (not just the component). It results in a ros:shutdown()
    * - WATCHDOG: watchdog message, to tell that the component is still alive
    * - ACK_CREATION: message to tell to a component that has required the creation of a node that the creation is complete. It also allows the creating component to get the id of the created component.
    */
    typedef enum {
        CHG_FREQ = 0, LOCAL_CLIENT_DEATH, COMPLETE_NODE_DEATH, WATCHDOG, ACK_CREATION, ASK_NEW_ACK
    } MgmtType;

    /**
     * @brief class ClientDescriptor. To describe a client by his namspace (ns), an id and a type
     *
     */
    class ClientDescriptor {
    public:
        std::string ns;
        long int id;
        std::string type;
    };

    /**
     * @brief operator == for comparison between client.
     * @return
     */
    bool operator==(ClientDescriptor const&, ClientDescriptor const&);

    struct ClientDescriptorHasher {
        std::size_t operator()(const ClientDescriptor& cd) const
        {

            std::ostringstream oss;
            oss << cd.ns << "_" << cd.id;
            std::hash<std::string> string_hash;

            return string_hash(oss.str());
        }
    };

    using MapWatchDog_t=std::unordered_map<ClientDescriptor, ros::Time, ClientDescriptorHasher>;
    using CreatedNodes_t=std::unordered_map<std::string, std::vector<ClientDescriptor> >;


    /**
     * @brief Abstract class for the Cafer client
     *  Main functionnalities:
     *  \li watchdog
     *  \li frequency management
     *  \li waits for initialization
     */
    class Component {

    public:

        /**
         * @brief constructor of Component
         * @param management topic
         * @param type of this component
         * @param frequence of update. 10 by default.
         * @param true if a new nodehandle must be created. false by default
         */
        Component(std::string mgmt_topic, std::string _type, double freq = 10, bool new_nodehandle = false);


        ~Component(void)
        {
            //disconnect_from_ros();
        }


        /**
         * @brief update
         * Calls the code to take into account what has been received through the subscribers (typically update parameter values by taking into account received values)
         */
        virtual void update() = 0;

        /**
         * @brief connect_to_ros
         * all the definition of subscriber and publisher must be done in this method. WARNING: this is not called by default. It is usually called in the init() method.
         */
        virtual void client_connect_to_ros() = 0;

        /**
         * @brief disconnect_from_ros
         * the purpose of this methode is to free the memory of all subscribers and publishers shared_ptrof the node. WARNING: this is not called by default. Is is usually called in the destructor.
         */
        virtual void client_disconnect_from_ros() = 0;

        /**
         * @brief init
         * the purpose of this method is to make whatever initialization required.
         */
        virtual void init() = 0;

        /**
         * @brief is_initialized
         * @return true if the node is initialized false other wise.
         */
        virtual bool is_initialized()
        { return _is_init; }


        /**
         * Should the client terminate ? This needs to be taken into account in the user code */
        /**
         * @brief get_terminate return true if the client is down and false otherwise
         * @return true or false
         */
        bool get_terminate(void) const
        { return terminate; }

        /**
         * @brief Get type of the client
         * @return the type of client.
         */
        std::string get_type(void) const
        { return type; }


        /**
         * @brief  Accessor to the client id (unique)
         * @return the id of client
         */
        long int get_id(void) const
        { return id; }

        /**
         * @brief this method inform if the subcriber, publisher or server are up or not.
         * @return true if the node is connected to ros
         */
        bool is_connected_to_ros() const
        { return _is_connected_to_ros; }

        /**
         * @brief Connect the client to ROD, i.e. intantiate subscribers, publishers and services.
         */
        void connect_to_ros(void)
        {
            // connection to management topic is done in the constructor only.
            client_connect_to_ros();
            _is_connected_to_ros = true;
        }

        /**
         * @brief Disconnect the client from ROS, i.e. destroy subscribers and publishers
         */
        void disconnect_from_ros(void)
        {
            client_disconnect_from_ros();
            management_s.reset();
            watchdog.reset();
            _is_connected_to_ros = false;
            sleep();
        }

        /**
         * @brief Shutting down the component and the corresponding node
         */
        void shutdown(void)
        {
            disconnect_from_ros();
            ros::shutdown();
        }

        /**
         * @brief Call a launch file. Corresponding nodes are to be launched in a namespace namespace_base_XX where XX is a unique id (provided by the getid service). It will be connected to the management_topic topic (the same than this class if this argument equals "").
         * @param name of the launch file
         * @param base name of general namespace
         * @param name of the management topic
         * @return the namespace created for this Component
         */
        std::string call_launch_file(std::string launch_file, std::string namespace_base,
                                     std::string management_topic = "");


        /**
         * @brief Sleep: should be called by the user code once all the things that have to be done during one iteration of the loop have been done
         */
        void sleep(void)
        {rate->sleep();}

        /**
         * @brief encapsulation of ros::spinOnce()
         */
        void spin(void);

        /**
         * @brief Check the number of client nodes that have been observed up to now (watchdog) and return their number.
         * If up_only is set to true, only the nodes that are up are counted, otherwise, they are all counted.
         * @param _type of components that we want to check
         * @param up_only boolean. true by default.
         * @return number of client still up
         */
        unsigned int how_many_client_from_type(std::string _type, bool up_only = true);


        /**
         * @brief Get the ClientDescriptors of all clients connected to the same management topic and of a certain type
         * @param type of clients
         * @param [out] vector of client descriptor
         * @param if true, consider only the clients that are up. true by default
         */
        void get_connected_client_with_type(std::string _type, std::vector<ClientDescriptor>& vcd, bool up_only = true);

        /** */
        /**
         * @brief Check if a given time is "recent" or not with respect to the node client frequency
         * @param the time to check
         * @return true if condition is fulfill, false otherwise
         */
        bool is_it_recent_enough(ros::Time t);

        /**
         * @brief Check whether a client is up or not (relies on the watchdog functionnality, works only for nodes that are on the same management topic)
         * @param namespace of the client
         * @param id of the client
         * @return true if the client is up , false if not
         */
        bool is_client_up(std::string ns, long int id)
        {
            return is_it_recent_enough(get_watchdog(ns, id));
        }


        /**
         * @brief  Waits until the client is up (it needs to be on the same management topic)
         * @param namespace of client
         * @param id of client
         */
        void wait_for_client(std::string ns, long int id);

        /**
         * @brief  Waits for the initialization from the client specific side
         */
        void wait_for_init(void);

        /**
         * @brief Gets node namespace
         * @return
         */
        std::string get_namespace(void) const
        {return my_ros_nh->getNamespace();}

        /**
         * @brief Gets the created_nodes
         * @return
         */
        CreatedNodes_t& get_created_nodes(void)
        {return created_nodes;}

        /**
         * @brief Gets the created_nodes
         * @param created_ns
         * @return
         */
        std::vector<ClientDescriptor>& get_created_nodes(std::string created_ns)
        {return created_nodes[created_ns];}

        /**
         * @brief kill all nodes created by this component
        */
        void kill_created_nodes(void);

        /**
         * @brief Management callback: what to do when a management message is received
         * @param management message
         */
        void management_cb(const cafer_core::Management& mgmt);

        /**
         * @brief ask for acknowledgement to all nodes that are handled by this Component
         */
        void ask_new_ack();

        /**
         * @brief Creation of an acknowledgement to respond to an ask for acknowledgment
         */
        void ack_creation();

        /**
         * @brief What to do when a watchdog message is received: update the corresponding time in the watchdog map
         * @param namespace of client checked
         * @param id of checked client
         * @param type of checked client
         */
        void update_watchdog(std::string ns, long int id, std::string _type);

        /**
         * @brief Get the time of the last watchdog message for a particular client
         * @param namespace of client
         * @param id of client
         * @return time of last watchdog message received
         */
        ros::Time get_watchdog(std::string ns, long int id);

        /**
         * @brief Send a watchdog message telling that the client is still alive...
         * @param event
         */
        void watchdog_cb(const ros::TimerEvent& event);

        /**
         * @brief send a request of death to a specific client
         * @param namespace of client
         * @param id of client
         */
        void send_complete_node_death(std::string ns, long int id);

        /**
         * @brief send a request for client to disconect him from ros
         * @param namespace of client
         * @param id of client
         */
        void send_local_node_death(std::string ns, long int id);

    public:
        MapWatchDog_t map_watchdog;
        /**< map in which is stored the last watchdog message received from each client connected to the management topic of this client */

        CreatedNodes_t created_nodes;
        /**< map in which are stored all nodes created by a call_launch_file. The key is the required namespace */
        shared_ptr<NodeHandle> my_ros_nh;
        shared_ptr<ros::CallbackQueue> my_ros_queue;


        shared_ptr<Subscriber> management_s;
        /**< subscriber to the management topic */
        shared_ptr<Publisher> management_p;
        /**<publisher to the management topic */

    protected :
        int id;
        std::string type;
        int creator_id;
        std::string creator_ns;
        std::string created_ns;
        std::string _mgmt_topic;
        bool terminate;
        /**< did we receive the order to stop the client ?*/

        shared_ptr<ros::Rate> rate;
        /**< ROS update rate */
        shared_ptr<ros::Timer> watchdog;
        /**< Watchdog */



        bool _is_init = false;
        bool _is_connected_to_ros = false;

    };

}

#endif
