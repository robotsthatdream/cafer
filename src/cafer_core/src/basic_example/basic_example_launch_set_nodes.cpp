
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
#include <ros/package.h>
#include <ros/impl/duration.h>
#include "../component.hpp"
#include "cafer_core/Management.h"


class DummyClient {
  boost::shared_ptr<ros::Publisher> dummy_p; 
  long int n;
public:
  void connect_to_ros(void) {  }
  void disconnect_from_ros(void) {dummy_p.reset();}
  bool is_initialized(void){return true;}
  void update(void) {  }
};

int main(int argc, char **argv){
  
  /** Parameters */
  std::string management_topic, type, ns;  
  int nb_nodes = 2;
  double freq;

  /**  New node */
  cafer_core::init(argc, argv, "basic_example_launch_set_nodes");	
     
  cafer_core::ros_nh->getParam(ros::this_node::getName()+"/management_topic",management_topic);  
  cafer_core::ros_nh->getParam(ros::this_node::getName()+"/type",type); 
  cafer_core::ros_nh->getParam(ros::this_node::getName()+"/frequency",freq);
  ns = cafer_core::ros_nh->getNamespace();
  ROS_INFO_STREAM("Launching "<<nb_nodes<<" new nodes (name="<<ros::this_node::getName()<<")");

  /** Create component, in charge of calling the launch file creating the new nodes */
  cafer_core::Component<DummyClient> cc(management_topic, type);
  cc.wait_for_init();
  sleep(3);
  std::string basic_example_new_node_launch = ros::package::getPath("cafer_core")+"/launch/basic_example_new_node.launch";

  /** Check the number of current nodes */
  std::vector<cafer_core::ClientDescriptor> current_nodes;
  cc.get_connected_client_with_type(type, current_nodes);  

  /** New nodes to be created */
  for(int i=0; i < nb_nodes; i++){
    ROS_INFO_STREAM("Params : " << " " << ns << " " << management_topic << " " << type << " " << nb_nodes << " " << freq);
    cc.call_launch_file(basic_example_new_node_launch,ns + "/basic_node");
  }

  cc.spin();

  return 0;
}
