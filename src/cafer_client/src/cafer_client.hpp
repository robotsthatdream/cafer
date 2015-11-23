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
#include "cafer_server/LaunchNode.h"
#include "cafer_server/ReleaseNode.h"
#include "cafer_server/KillNodeGroup.h"
#include "cafer_client/Management.h"
#include "cafer_server/GetID.h"
#include <ros/spinner.h>
#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>
#include <boost/functional/hash.hpp>

namespace cafer_client {
 
  extern boost::shared_ptr<ros::NodeHandle> ros_nh;

  void init(int argc, char **argv, std::string node_name);
  std::string get_node_group(std::string namespace_base, std::string launch_file, double frequency);
  void release_node_group(std::string namespace_base, std::string gr_namespace);
  void kill_node_group(std::string namespace_base, std::string gr_namespace);
  void kill_all_allocated_node_groups(void);

  /** Definition of message types */
  typedef enum {CHG_FREQ=0, LOCAL_CLIENT_DEATH, COMPLETE_NODE_DEATH,WATCHDOG} MgmtType;

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


  /** Template for the Cafer client
   *  Main functionnalities:
   *  \li watchdog
   *  \li frequency management
   *  \li waits for initialization
   */
  template<class Client> class CaferClient {
  private:
    Client client;
    int id;
    std::string type;

    bool terminate; /**< did we receive the order to stop the client ?*/

    boost::shared_ptr<ros::Rate> rate; /**< ROS update rate */
    boost::shared_ptr<ros::Timer> watchdog; /**< Watchdog */


    boost::shared_ptr<ros::Subscriber> management_s; /**< subscriber to the management topic */
    boost::shared_ptr<ros::Publisher> management_p; /**<publisher to the management topic */

    typedef boost::unordered_map<ClientDescriptor, ros::Time, ClientDescriptorHasher> MapWatchDog_t;
    MapWatchDog_t map_watchdog; /**< map in which is stored the last watchdog message received from each client connected to the management topic of this client */
    //boost::unordered_map<std::string, std::string> map_type; /**< map in which the types of the nodes are stored. */


  public:
    CaferClient(std::string mgmt_topic, std::string _type, double freq=10): type(_type),terminate(false),map_watchdog(5) {
      rate.reset(new ros::Rate(freq));
      management_p.reset(new ros::Publisher(ros_nh->advertise<cafer_client::Management>(mgmt_topic.c_str(),10)));
      management_s.reset(new ros::Subscriber(ros_nh->subscribe(mgmt_topic.c_str(),10,&CaferClient::management_cb,this))); 
      watchdog.reset(new ros::Timer(ros_nh->createTimer(ros::Duration(ros::Rate(freq)), &CaferClient::watchdog_cb, this)));

      // We get a new and unique ID for this client
      cafer_server::GetID v;
      v.request.name = "cafer_client_id";
      static ros::ServiceClient sclient = ros_nh->serviceClient<cafer_server::GetID>("/cafer_server/get_id");
      if (sclient.call(v))
	{
	  id=v.response.id;
	}
      else
	{
	  ROS_ERROR_STREAM("Failed to call service get_id. my namespace is: "<<ros_nh->getNamespace());
	  id=-1;
	}


    }

    ~CaferClient(void) {
      disconnect_from_ros();
    }

    /** Should the client terminate ? This needs to be taken into account in the user code */
    bool get_terminate(void) const { return terminate;}
	
    /** Type of the client */
    std::string get_type(void) const {return type;}

    /** Accessor to the client id (unique) */
    long int get_id(void) const { return id;}

    /** Disconnect the client from ROS, i.e. destroy subscribers and publishers */
    void disconnect_from_ros(void) {
      client.disconnect_from_ros();
      management_s.reset();
      watchdog.reset();
    }

    /** Call a launch file. Corresponding nodes are to be launched in a namespace namespace_base_XX where XX is a unique id (provided by the getid service). It will be connected to the management_topic topic (the same than this class if this argument equals "").*/
    std::string call_launch_file(std::string launch_file, std::string namespace_base, std::string management_topic="") {
      std::string created_namespace;

      if (management_topic =="") {
	management_topic=management_p->getTopic();
      }

      cafer_server::GetID v;
      v.request.name = namespace_base;
      static ros::ServiceClient client = ros_nh->serviceClient<cafer_server::GetID>("/cafer_server/get_id");
      if (client.call(v))
	{
	  std::ostringstream os, osf;
	  os<<"/"<<namespace_base<<"_"<<v.response.id;
	  created_namespace=os.str();
	  std::string ns="ns:="+os.str();
	  osf<<"frequency:="<<1./rate->expectedCycleTime().toSec();
	  std::string mgmt="management_topic:="+management_topic;
	  std::string cmd="roslaunch "+launch_file+" "+ns+" "+osf.str()+" "+mgmt+"&";
	  system(cmd.c_str());
	  ROS_INFO("Launch file called: namespace=%s launch file=%s frequency=%f management_topic=%s", created_namespace.c_str(), launch_file.c_str(), (float)(1./rate->expectedCycleTime().toSec()), management_p->getTopic().c_str());
	}
      else
	{
	  ROS_ERROR_STREAM("Failed to call service get_id. my namespace is: "<<ros_nh->getNamespace());
	}
      return created_namespace;
    }
 
    /** Accessor to the client specific code */
    Client &get_client(void) {return client;}
 
    /** Sleep: should be called by the user code once all the things that have to be done during one iteration of the loop have been done */
    void sleep(void) {
      rate->sleep();
    }

    void spin(void) {
      ros::spinOnce();
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
	ros::spinOnce();
	sleep();
      }
      update();
    }

    /** Waits for the initialization from the client specific side */
    void wait_for_init(void) {
      while((!is_client_up(get_namespace(),get_id())) && (!client.is_initialized())) {
	ros::spinOnce();
	sleep();
      }
      update();
    }

    /** Calls the code to take into account what has been received through the subscribers (typically update parameter values by taking into account received values)*/
    void update(void) {
      client.update();
    }

    /** Gets node namespace */
    std::string get_namespace(void) const {
      return ros_nh->getNamespace();
    }

    /** Management callback: what to do when a management message is received */
    void management_cb(const cafer_client::Management &mgmt) {
      ROS_INFO_STREAM("management_cb my_id="<<get_id()<<" type="<<mgmt.type);
      switch (mgmt.type) {
      case CHG_FREQ:
	ROS_INFO_STREAM("Changing frequency: "<<mgmt.data_flt);
	rate.reset(new ros::Rate(mgmt.data_flt));
	watchdog.reset(new ros::Timer(ros_nh->createTimer(ros::Duration(ros::Rate(mgmt.data_flt)), &CaferClient::watchdog_cb, this)));
	break;
      case LOCAL_CLIENT_DEATH:
	if ((mgmt.dest_node == "all")||((mgmt.dest_node == ros_nh->getNamespace())&&(mgmt.dest_id == get_id()))) {
	  ROS_INFO_STREAM("LOCAL_CLIENT_DEATH");
	  terminate=true;
	}	
	break;
      case COMPLETE_NODE_DEATH:
	if ((mgmt.dest_node == "all")||((mgmt.dest_node == ros_nh->getNamespace())&&(mgmt.dest_id == get_id()))) {
	  ROS_INFO_STREAM("COMPLETE_NODE_DEATH called");
	  ros::shutdown();
	}
	break;
      case WATCHDOG:
	update_watchdog(mgmt.src_node,mgmt.src_id,mgmt.src_type);
	break;
      default:
	ROS_WARN_STREAM("cafer_client: received unknown message: type="<<mgmt.type);
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
      ROS_INFO_STREAM("get_watchdog initialized "<<ns<<" time="<<map_watchdog[cd]<<" "<<id<<" my id="<<get_id());
      return map_watchdog[cd];
      
    }

    /** Send a watchdog message telling that the client is still alive... */
    void watchdog_cb(const ros::TimerEvent& event)
    {
      ROS_INFO_STREAM("watchdog_cb my_id="<<get_id());
      cafer_client::Management msg;
      msg.type=WATCHDOG;
      msg.src_node=ros_nh->getNamespace();
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
      ROS_INFO_STREAM("watchdog_cb my_id="<<get_id());
      cafer_client::Management msg;
      msg.type=COMPLETE_NODE_DEATH;
      msg.src_node=ros_nh->getNamespace();
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
