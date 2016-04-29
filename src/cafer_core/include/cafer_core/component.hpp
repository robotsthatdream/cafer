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

#include <ros/ros.h>
#include "cafer_core/Management.h"
#include "cafer_core/GetID.h"
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>
#include <boost/functional/hash.hpp>

namespace cafer_core {

  extern boost::shared_ptr<ros::NodeHandle> ros_nh;

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
  typedef enum {CHG_FREQ=0, LOCAL_CLIENT_DEATH, COMPLETE_NODE_DEATH,WATCHDOG,ACK_CREATION,ASK_NEW_ACK} MgmtType;

  /** Client descriptor */
  class ClientDescriptor{
   public:
    std::string ns;
    long int id;
    std::string type;
  };

  bool operator==(ClientDescriptor const&, ClientDescriptor const&);

  struct ClientDescriptorHasher
  {
    std::size_t operator()(const ClientDescriptor& cd) const
    {

      std::ostringstream oss;
      oss<<cd.ns<<"_"<<cd.id;
      boost::hash<std::string> string_hash;

      return string_hash(oss.str());
    }
  };

  typedef boost::unordered_map<ClientDescriptor, ros::Time, ClientDescriptorHasher> MapWatchDog_t;
  typedef boost::unordered_map<std::string, std::vector<ClientDescriptor> > CreatedNodes_t;


  /** Abstract class for the Cafer client
   *  Main functionnalities:
   *  \li watchdog
   *  \li frequency management
   *  \li waits for initialization
   */
  class Component {

  public:

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
     * the purpose of this methode is to free the memory of all subscribers and publishers shared_ptr of the node. WARNING: this is not called by default. Is is usually called in the destructor.
     */
    virtual void client_disconnect_from_ros() = 0;

    /**
     * @brief init
     * the purpose of this methode is to make whatever initialization required. It is called at the end of the Component constructor.
     */
    virtual void init() = 0;

    /**
     * @brief is_initialized
     * @return true if the node is initialized false other wise.
     */
    virtual bool is_initialized(){return _is_init;}




  protected:
    bool _is_init = false;
    bool _is_connected_to_ros = false;

  public:
    boost::shared_ptr<ros::NodeHandle> my_ros_nh;
    boost::shared_ptr<ros::CallbackQueue> my_ros_queue;

    int id;
    std::string type;
    int creator_id;
    std::string creator_ns;
    std::string created_ns;
    std::string _mgmt_topic;
    bool terminate; /**< did we receive the order to stop the client ?*/

    boost::shared_ptr<ros::Rate> rate; /**< ROS update rate */
    boost::shared_ptr<ros::Timer> watchdog; /**< Watchdog */


    boost::shared_ptr<ros::Subscriber> management_s; /**< subscriber to the management topic */
    boost::shared_ptr<ros::Publisher> management_p; /**<publisher to the management topic */

    MapWatchDog_t map_watchdog; /**< map in which is stored the last watchdog message received from each client connected to the management topic of this client */

    CreatedNodes_t created_nodes; /**< map in which are stored all nodes created by a call_launch_file. The key is the required namespace */


  public:
    Component(std::string mgmt_topic, std::string _type, double freq=10, bool new_nodehandle=false): type(_type),terminate(false),map_watchdog(5) {
      rate.reset(new ros::Rate(freq));
      if (mgmt_topic =="") {
        std::string default_value="default_"+type;
        my_ros_nh->param("management_topic",mgmt_topic,default_value);
      }
      ROS_INFO_STREAM("Creating a component connected to management_topic: "<<mgmt_topic<<" type="<<_type);
      if (new_nodehandle) {
	ROS_INFO_STREAM("     creation of a dedicated callback queue");
	my_ros_nh.reset(new ros::NodeHandle(*ros_nh.get()));
	my_ros_queue.reset(new ros::CallbackQueue());
	my_ros_nh->setCallbackQueue(my_ros_queue.get());
      }
      else {
	my_ros_nh=ros_nh;
	my_ros_queue.reset();
      }

      management_p.reset(new ros::Publisher(my_ros_nh->advertise<cafer_core::Management>(mgmt_topic.c_str(),0)));
      management_s.reset(new ros::Subscriber(my_ros_nh->subscribe(mgmt_topic.c_str(),0,&Component::management_cb,this)));
      watchdog.reset(new ros::Timer(my_ros_nh->createTimer(ros::Duration(ros::Rate(freq)), &Component::watchdog_cb, this)));

      // We get a new and unique ID for this client
      cafer_core::GetID v;
      v.request.name = "component_id";
      static ros::ServiceClient sclient = my_ros_nh->serviceClient<cafer_core::GetID>("/cafer_core/get_id");
      if (sclient.call(v))
	     {
	        id=v.response.id;
	     }
      else
	     {
	        ROS_ERROR_STREAM("Failed to call service get_id. my namespace is: "<<my_ros_nh->getNamespace());
	        id=-1;
	     }


       my_ros_nh->param("creator_id",creator_id,-1);

       my_ros_nh->param("created_ns",created_ns,std::string("<unset>"));
       my_ros_nh->param("creator_ns",creator_ns,std::string("<unset>"));

       //init();
       //ack_creation();

    }

    ~Component(void) {
      //disconnect_from_ros();
    }

    /** Should the client terminate ? This needs to be taken into account in the user code */
    bool get_terminate(void) const { return terminate;}

    /** Type of the client */
    std::string get_type(void) const {return type;}

    /** Accessor to the client id (unique) */
    long int get_id(void) const { return id;}

    bool is_connected_to_ros() const {return _is_connected_to_ros;}

    /** Disconnect the client from ROS, i.e. destroy subscribers and publishers */
    void connect_to_ros(void) {
      // connection to management topic is done in the constructor only.
      client_connect_to_ros();
      _is_connected_to_ros=true;
    }

    /** Disconnect the client from ROS, i.e. destroy subscribers and publishers */
    void disconnect_from_ros(void) {
      client_disconnect_from_ros();
      management_s.reset();
      watchdog.reset();
      _is_connected_to_ros=false;
    }

    /** Shutting down the component and the corresponding node */
    void shutdown(void) {
      disconnect_from_ros();
      ros::shutdown();
    }

    /** Call a launch file. Corresponding nodes are to be launched in a namespace namespace_base_XX where XX is a unique id (provided by the getid service). It will be connected to the management_topic topic (the same than this class if this argument equals "").*/
    std::string call_launch_file(std::string launch_file, std::string namespace_base, std::string management_topic="") {
      std::string created_namespace="<Failed>";

      if (management_topic =="") {
	       management_topic=management_p->getTopic();
      }

      cafer_core::GetID v;
      v.request.name = namespace_base;
      static ros::ServiceClient clients = my_ros_nh->serviceClient<cafer_core::GetID>("/cafer_core/get_id");
      if (clients.call(v))
	{
	  std::ostringstream os, osf;
	  os<<"/"<<namespace_base<<"_"<<v.response.id;
	  created_namespace=os.str();
	  std::string ns="ns:="+os.str();
	  osf<<"frequency:="<<1./rate->expectedCycleTime().toSec()<<" creator_ns:="<<my_ros_nh->getNamespace()<<" creator_id:="<<get_id();
	  std::string mgmt="management_topic:="+management_topic;
	  std::string cmd="roslaunch "+launch_file+" "+ns+" "+osf.str()+" "+mgmt+"&";
	  if (system(cmd.c_str())==-1) {
      ROS_ERROR_STREAM("Failed to execute roslaunch. Called command: "<<cmd);
      return created_namespace;
    }
	  ROS_INFO_STREAM("Launch file call: "<<cmd);

	}
      else
	{
	  ROS_ERROR_STREAM("Failed to call service get_id. my namespace is: "<<my_ros_nh->getNamespace());
	}
      return created_namespace;
    }

    /** Sleep: should be called by the user code once all the things that have to be done during one iteration of the loop have been done */
    void sleep(void) {
      rate->sleep();
    }

    void spin(void) {
      if (my_ros_queue.get()==NULL)
	ros::spinOnce();
      else 
	my_ros_queue->callAvailable(ros::WallDuration());

    }

    /** Check the number of client nodes that have been observed up to now (watchdog) and return their number.
     * If up_only is set to true, only the nodes that are up are counted, otherwise, they are all counted.
     */
    unsigned int how_many_client_from_type(std::string _type, bool up_only=true) {
      unsigned int nb=0;
      ROS_INFO_STREAM("how_many_client_from_type: "<<_type<<" my id="<<get_id());
      BOOST_FOREACH( MapWatchDog_t::value_type& v, map_watchdog ) {
	if ((v.first.type == _type)&&((!up_only)||(is_it_recent_enough(v.second)))) {
	    nb++;
	  }
      }
      return nb;
    }

    /** Get the ClientDescriptors of all clients connected to the same management topic and of a certain type */
    void get_connected_client_with_type(std::string _type, std::vector<ClientDescriptor> &vcd, bool up_only=true) {
      BOOST_FOREACH(MapWatchDog_t::value_type & v, map_watchdog ) {
	if ((v.first.type == _type)&&((!up_only)||(is_it_recent_enough(v.second)))) {
	  vcd.push_back(v.first);
	}
      }

    }

    /** Check if a given time is "recent" or not with respect to the node client frequency */
    bool is_it_recent_enough(ros::Time t) {
      ros::Duration d=ros::Time::now()-t;
      return d<=rate->expectedCycleTime()*2.;

    }

    /** Check whether a client is up or not (relies on the watchdog functionnality, works only for nodes that are on the same management topic) */
    bool is_client_up(std::string ns, long int id) {
      return is_it_recent_enough(get_watchdog(ns, id));
    }



    /** Waits until the client is up (it needs to be on the same management topic) */
    void wait_for_client(std::string ns, long int id) {
      while(!is_client_up(ns,id)) {
	       spin();
	       sleep();
      }
      update();
    }

    /** Waits for the initialization from the client specific side */
    void wait_for_init(void) {
      init();

      while((!is_client_up(get_namespace(),get_id())) && (!is_initialized())) {
	ROS_INFO_STREAM("Component id="<<get_id()<<" waiting for init.");
	spin();
	sleep();
      }

      ack_creation();

      update();
    }

    /** Gets node namespace */
    std::string get_namespace(void) const {
      return my_ros_nh->getNamespace();
    }

    /** Gets the created_nodes */
    CreatedNodes_t &get_created_nodes(void) {
      return created_nodes;
    }

    /** Gets the created_nodes */
    std::vector<ClientDescriptor> &get_created_nodes(std::string created_ns) {
      return created_nodes[created_ns];
    }

    void kill_created_nodes(void) {
      BOOST_FOREACH(CreatedNodes_t::value_type & v, created_nodes) {
        BOOST_FOREACH(ClientDescriptor cd, v.second) {
          send_complete_node_death(cd.ns, cd.id);
        }
      }
    }

    /** Management callback: what to do when a management message is received */
    void management_cb(const cafer_core::Management &mgmt) {
      //ROS_INFO_STREAM("management_cb my_id="<<get_id()<<" message: "<<std::endl<<mgmt<<std::flush);
      switch (mgmt.type) {
      case CHG_FREQ:
	       ROS_INFO_STREAM("Changing frequency: new frequency="<<mgmt.data_flt<<" my_id="<<get_id());
	       rate.reset(new ros::Rate(mgmt.data_flt));
	       watchdog.reset(new ros::Timer(my_ros_nh->createTimer(ros::Duration(ros::Rate(mgmt.data_flt)), &Component::watchdog_cb, this)));
	       break;
      case LOCAL_CLIENT_DEATH:
	       if ((mgmt.dest_node == "all")||((mgmt.dest_node == my_ros_nh->getNamespace())&&(mgmt.dest_id == get_id()))) {
	          ROS_INFO_STREAM("LOCAL_CLIENT_DEATH");
	          terminate=true;
	       }
	       break;
      case COMPLETE_NODE_DEATH:
	       if ((mgmt.dest_node == "all")||((mgmt.dest_node == my_ros_nh->getNamespace())&&(mgmt.dest_id == get_id()))) {
	          ROS_INFO_STREAM("COMPLETE_NODE_DEATH called");
	          shutdown();
	      }
	      break;
      case WATCHDOG:
	      update_watchdog(mgmt.src_node,mgmt.src_id,mgmt.src_type);
	      break;
      case ACK_CREATION:
        if ((mgmt.dest_node == "all")||((mgmt.dest_node == my_ros_nh->getNamespace())&&(mgmt.dest_id == get_id()))) {
	  ROS_INFO_STREAM("Ack received by the creator: mgmt.dest_node="<<mgmt.dest_node<<" mgmt.dest_id="<<mgmt.dest_node<<" my_ns="<<my_ros_nh->getNamespace()<<" my_id="<<get_id()<<" src_ns="<<mgmt.src_node<<" src_id="<<mgmt.src_id<<" src_type="<<mgmt.src_type);
	  ClientDescriptor cd;
	  cd.ns=mgmt.src_node;
	  cd.id=mgmt.src_id;
	  cd.type=mgmt.src_type;
	  std::vector<ClientDescriptor>::iterator it=std::find(created_nodes[mgmt.data_str].begin(),created_nodes[mgmt.data_str].end(),cd);
	  if (it==created_nodes[mgmt.data_str].end())
	    created_nodes[mgmt.data_str].push_back(cd);
        }
        break;
      case ASK_NEW_ACK:
	ack_creation();
	break;
      default:
	     ROS_WARN_STREAM("component: received unknown message: type="<<mgmt.type);
      }
    }

    void ask_new_ack() {
      cafer_core::Management msg;
      msg.type=ASK_NEW_ACK;
      msg.src_node=my_ros_nh->getNamespace();
      msg.src_id=get_id();
      msg.src_type=get_type();
      msg.dest_node="all";
      msg.dest_id=-1;
      msg.data_int=0;
      msg.data_flt=0;
      msg.data_str="";
      management_p->publish(msg);
      
    }    

    void ack_creation() {
      if(creator_id!=-1) {
        cafer_core::Management msg;
        msg.type=ACK_CREATION;
        msg.src_node=my_ros_nh->getNamespace();
        msg.src_id=get_id();
        msg.src_type=get_type();
        msg.dest_node=creator_ns;
        msg.dest_id=creator_id;
        msg.data_int=0;
        msg.data_flt=0;
        msg.data_str=created_ns;
        // We may need to wait a bit so that the management publisher is connected.
        ROS_INFO_STREAM("ACK_CREATION: waiting for the connection to the creator (ns="<<creator_ns<<" id="<<creator_id<<").my_id="<<get_id());
        wait_for_client(creator_ns,creator_id);
        ROS_INFO_STREAM("ACK_CREATION: connection to the creator OK. my_id="<<get_id());
        management_p->publish(msg);
        ROS_INFO_STREAM("Sending ack after component creation: creator_ns="<<creator_ns<<" creator_id="<<creator_id<<" created_ns="<<created_ns);
      }
      else{
        ROS_WARN_STREAM(my_ros_nh->getNamespace()<<": No creator id provided, no ack has been sent.");
      }

    }

    /** What to do when a watchdog message is received: update the corresponding time in the watchdog map */
    void update_watchdog(std::string ns, long int id, std::string _type) {
      ClientDescriptor cd;
      cd.ns=ns;
      cd.id=id;
      cd.type=_type;
      map_watchdog[cd]=ros::Time::now();
      ROS_INFO_STREAM("update_watchdog "<<ns<<" "<<id<<" "<<_type<<" my_id="<<get_id());
    }

    /** Get the time of the last watchdog message for a particular client */
    ros::Time get_watchdog(std::string ns, long int id) {
      ClientDescriptor cd;
      cd.ns=ns;
      cd.id=id;
      cd.type="undefined";
      if (map_watchdog.find(cd) == map_watchdog.end()) {
	ROS_INFO_STREAM("get_watchdog uninitialized "<<ns<<" "<<id<<" time="<<ros::Time(0)<<" my id="<<get_id());
	return ros::Time(0);

      }
      //ROS_INFO_STREAM("get_watchdog initialized "<<ns<<" time="<<map_watchdog[cd]<<" id="<<id<<" my id="<<get_id());
      return map_watchdog[cd];

    }

    /** Send a watchdog message telling that the client is still alive... */
    void watchdog_cb(const ros::TimerEvent& event)
    {
      ROS_INFO_STREAM("watchdog_cb my_id="<<get_id());
      ROS_INFO_STREAM("watchdog_cb, management topic="<<management_p->getTopic()<<" nb connected="<<management_p->getNumSubscribers()<<std::flush);
      cafer_core::Management msg;
      msg.type=WATCHDOG;
      msg.src_node=my_ros_nh->getNamespace();
      msg.src_id=get_id();
      msg.src_type=get_type();
      msg.dest_node="all";
      msg.dest_id=-1;
      msg.data_int=0;
      msg.data_flt=0;
      msg.data_str="";
      management_p->publish(msg);
    }

    void send_complete_node_death(std::string ns, long int id) {
      ROS_INFO_STREAM("send_complete_node_death my_id="<<get_id());
      cafer_core::Management msg;
      msg.type=COMPLETE_NODE_DEATH;
      msg.src_node=my_ros_nh->getNamespace();
      msg.src_id=get_id();
      msg.src_type=get_type();
      msg.dest_node=ns;
      msg.dest_id=id;
      msg.data_int=0;
      msg.data_flt=0;
      msg.data_str="";
      management_p->publish(msg);
    }

    void send_local_node_death(std::string ns, long int id) {
      ROS_INFO_STREAM("send_local_node_death my_id="<<get_id());
      cafer_core::Management msg;
      msg.type=LOCAL_CLIENT_DEATH;
      msg.src_node=my_ros_nh->getNamespace();
      msg.src_id=get_id();
      msg.src_type=get_type();
      msg.dest_node=ns;
      msg.dest_id=id;
      msg.data_int=0;
      msg.data_flt=0;
      msg.data_str="";
      management_p->publish(msg);
    }

  };

}

#endif
