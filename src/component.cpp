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

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <ros/spinner.h>
#include "cafer_core/component.hpp"
#include "component_python.cpp"

#include <tbb/concurrent_hash_map.h>

namespace cafer_core {

  // Hashing for strings
  struct MyHashCompare {
    static size_t hash( const std::string& x ) {
      size_t h = 0;
      for( const char* s = x.c_str(); *s; ++s )
	h = (h*17)^*s;
      return h;
    }
    //! True if strings are equal
    static bool equal( const std::string& x, const std::string& y ) {
      return x.compare(y)==0;
    }
  };

  // set of node groups allocated to this sferes run (to clean it up at the end)
  tbb::concurrent_hash_map<std::string,std::string,MyHashCompare> allocated_node_group;



  boost::shared_ptr<ros::NodeHandle> ros_nh;


  void init(int argc, char **argv, std::string node_name) {
    ros::init(argc, argv, node_name);
    ros_nh.reset(new ros::NodeHandle("~"));
    std::cout<<"Initialising ROS. Node name: "<<node_name<<std::endl;
  }

  void python_init(std::string node_name) {
    int _argc = 0;
    char **_argv = NULL;
    init(_argc, _argv, node_name);
  }

  std::string python_get_node_name(){
    return ros::this_node::getName();
  }

  int Component::python_get_created_nodes_number() {
    return this->created_nodes.size();
  }

  void Component::python_print_created_nodes_id(){
    // std::vector<int> node_id_vector;
    BOOST_FOREACH(cafer_core::CreatedNodes_t::value_type & v, this->created_nodes) {
      BOOST_FOREACH(cafer_core::ClientDescriptor cd, v.second) {
        // node_id_vector.push_back(cd.id);
        ROS_INFO_STREAM("Component: id="<<cd.id<<" ns="<<cd.ns);
      }
    }
    return ;
  }

  bool Component::python_clients_status(std::string type){
    if (type.compare("up") == 0 ){
      bool all_up=true;
      BOOST_FOREACH(cafer_core::CreatedNodes_t::value_type & v, this->created_nodes) {
        BOOST_FOREACH(cafer_core::ClientDescriptor cd, v.second) {
          all_up=all_up && this->is_client_up(cd.ns, cd.id);
        }      
      }
      return all_up;
    }

    else if (type.compare("down") == 0 ){
      bool all_down=true;
      BOOST_FOREACH(cafer_core::CreatedNodes_t::value_type & v, this->created_nodes) {
        BOOST_FOREACH(cafer_core::ClientDescriptor cd, v.second) {
          all_down=all_down && !this->is_client_up(cd.ns, cd.id);
        }
      }      
      return all_down;
    }

    else {
      ROS_ERROR("python_clients_status was called with a wrong value");
      exit(-1);
    }
  }


  // ros::NodeHandle python_nh () {
  //   return *ros_nh;
  // }

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


  bool operator==(ClientDescriptor const &cd1, ClientDescriptor const &cd2) {
    return (cd1.ns==cd2.ns) && (cd1.id==cd2.id);
  }

}