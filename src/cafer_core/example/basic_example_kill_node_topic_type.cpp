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


class DummyClient : public cafer_core::AbstractClient {
  boost::shared_ptr<ros::Publisher> dummy_p;
  long int n;
public:
  void disconnect_from_ros(void) {dummy_p.reset();}
  void update(void) {  }
};

int main(int argc, char **argv){

  /** Parameters */
  std::string management_topic,type;

  if (argc != 3){
    ROS_INFO_STREAM("Correct use : rosrun  cafer_core  basic_example_kill_node_topic_type  topic  type  ");
    return 0;
  } else {
    management_topic = argv[1];
    type = argv[2];
  }

  /**  New node */
  cafer_core::init(argc, argv, "basic_example_kill_node_topic_type");

  // cafer_core::ros_nh->getParam("/basic_example_ns/basic_example_new_node/management_topic",management_topic);
  // cafer_core::ros_nh->getParam("/basic_example_ns/basic_example_new_node/type",type);
  ROS_INFO_STREAM("Killing nodes from topic " << management_topic << " and type " << type);

  /** Create component, in charge of calling the nodes to kill themself*/
  cafer_core::Component<DummyClient> cc(management_topic, type);
  cc.wait_for_init();
  sleep(3);

  cc.spin();

  /** Check the number of current nodes */
  int nb_nodes_to_kill = cc.how_many_client_from_type(type);
  std::vector<cafer_core::ClientDescriptor> ntk;
  if (nb_nodes_to_kill > 0){
    ROS_INFO_STREAM(nb_nodes_to_kill <<" nodes of type " << type << " to be killed");
    cc.get_connected_client_with_type(type, ntk);
  }

  /** Kill the nodes */
  for (int i=0; i < nb_nodes_to_kill; i++){
    cc.send_local_node_death(ntk[i].ns, ntk[i].id);
    int cpt=10;
    while (cc.is_client_up(ntk[0].ns, ntk[0].id) && (cpt>0)) {
      ROS_INFO_STREAM("The node " << ntk[0].id << " is up.");
      cc.spin();
      cc.sleep();
      sleep(1);
      cpt--;
    }
  }

  /** Check the number of nodes killed */
  ROS_INFO_STREAM(cc.how_many_client_from_type(type) <<" nodes available of type " << type);

  return 0;
}
