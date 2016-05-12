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
#include "cafer_core/cafer_core.hpp"
#include "cafer_core/Management.h"

/** This is a basic example in which two nodes are created through the call to a launch_file. The program checks that they have been properly launched and then kills them and check that they have stopped appropriately. */


class DummyClient : public cafer_core::Component {
  using cafer_core::Component::Component; // To inherit Component's constructor

  long int n;
public:
  ~DummyClient(){shutdown();}
  void client_disconnect_from_ros(void) {}
  void client_connect_to_ros(void) {}
  void update(void) {  }
  void init(void) {}

  bool is_initialized(void){return true;}
};

int main(int argc, char **argv){

  /** Parameters */
  std::string management_topic, type, ns;
  int nb_nodes = 2;
  double freq;

  /**  New node */
  cafer_core::init(argc, argv, "basic_example_launch_set_nodes");

  cafer_core::ros_nh->getParam("management_topic",management_topic);
  cafer_core::ros_nh->getParam("type",type);
  cafer_core::ros_nh->getParam("nb_nodes",nb_nodes);
  cafer_core::ros_nh->getParam("frequency",freq);
  ns = cafer_core::ros_nh->getNamespace();
  ROS_INFO_STREAM("Launching "<<nb_nodes<<" new nodes (name="<<ros::this_node::getName()<<")");

  /** Create the component in charge of calling the launch file for creating the new nodes */
  DummyClient cc(management_topic, type);
  cc.wait_for_init();

  /** Finding the path towards the launch_file to call */
  std::string basic_example_new_node_launch = ros::package::getPath("cafer_core")+"/launch/basic_example_new_node.launch";


 
  /** Creation of the new nodes */
  std::vector<std::string> created_namespaces;
  for(int i=0; i < nb_nodes; i++){
    ROS_INFO_STREAM("Params : " << " " << ns << " " << management_topic << " " << type << " " << nb_nodes << " " << freq);
    std::ostringstream oss;
    oss<<ns<<"/basic_node";
    created_namespaces.push_back(cc.call_launch_file(basic_example_new_node_launch,oss.str()));
    cc.sleep();
  }
  
  
  /** Checking that they are up */
  int nb_tries=10,count;
  while (nb_tries>0) {
    count=20;
    while((count>0)&&cc.get_created_nodes().size()!=nb_nodes) {
      cc.spin();
      cc.update();
      cc.sleep();
      count--;
    }
    if (count == 0) {
      ROS_INFO_STREAM("PROBLEM: we haven't received the ack from some nodes. We ask for a new ack.");
      ROS_INFO_STREAM("============= Ack received from "<<cc.get_created_nodes().size()<<" components: ");
      BOOST_FOREACH(cafer_core::CreatedNodes_t::value_type & v, cc.get_created_nodes()) {
	BOOST_FOREACH(cafer_core::ClientDescriptor cd, v.second) {
	  ROS_INFO_STREAM("Component: id="<<cd.id<<" ns="<<cd.ns);
	}
      }
      ROS_INFO_STREAM("=========== End ack received");
      ROS_INFO_STREAM("");
      cc.ask_new_ack();
      cc.spin();
      cc.update();
      cc.sleep();
    }
    else {
      break;
    }
    nb_tries--;
  }

  if (nb_tries==0) {
    ROS_INFO_STREAM("PROBLEM: the nodes haven't been all launched.");
    while(ros::ok()&&(!cc.get_terminate())) {
      cc.spin();
      cc.update();
      cc.sleep();
    }

    exit(1);
  }



  /** At this point at least one node from each call to call_launch_file has answered. As there is only one node in the launch file, it should be fine, but just to be sure (and to show how to check it) we verify that all of those nodes are up. */
  bool all_up=true;
  BOOST_FOREACH(cafer_core::CreatedNodes_t::value_type & v, cc.get_created_nodes()) {
    BOOST_FOREACH(cafer_core::ClientDescriptor cd, v.second) {
      all_up=all_up&&cc.is_client_up(cd.ns, cd.id);
    }
  }
  if (!all_up) {
    ROS_INFO_STREAM("PROBLEM: the nodes aren't up.");
    exit(1);
  }

  /** We kill all created_nodes */
  cc.kill_created_nodes();

  /** We check that they are down */
  count=100*nb_nodes;
  bool all_down=false;
  std::vector<cafer_core::ClientDescriptor>::iterator it;
  while((count>0)&&!all_down) {
    all_down=true;
    BOOST_FOREACH(cafer_core::CreatedNodes_t::value_type & v, cc.get_created_nodes()) {
      BOOST_FOREACH(cafer_core::ClientDescriptor cd, v.second) {
	all_down=all_down&&!cc.is_client_up(cd.ns, cd.id);
      }
    }
    cc.spin();
    cc.update();
    cc.sleep();
    count--;
  }

  if (!all_down) {
    ROS_INFO_STREAM("PROBLEM: the nodes aren't down after a kill.");
    exit(1);
  }
  else {
    ROS_INFO_STREAM("GREAT ! All nodes were created and killed as expected (count="<<count<<")");
  }

  /** The roslaunch may need to be stopped with a ctrl-c as the getid node will go on working after this node exits. */

  return 0;
}
