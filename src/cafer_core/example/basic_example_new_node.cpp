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
#include "cafer_core/cafer_core.hpp"
#include "cafer_core/Management.h"
#include <std_msgs/Int64.h>
#include <ros/impl/duration.h>


/** Basic example of client. Does nothing fancy, except */
class DummyClient :public cafer_core::Component {
  using cafer_core::Component::Component; // To inherit Component's constructor

  boost::shared_ptr<ros::Publisher> dummy_p;
  long int n;
public:
  ~DummyClient() {shutdown();}
  void client_connect_to_ros(void) {
    ROS_INFO_STREAM("DummyClient type="<<get_type()<<" id="<<get_id()<<" connecting to ROS");
    dummy_p.reset(new ros::Publisher(cafer_core::ros_nh->advertise<std_msgs::Int64>("basic_example_topic",10)));
    _is_init=true;
  }
  void client_disconnect_from_ros(void) {
    ROS_INFO_STREAM("DummyClient type="<<get_type()<<" id="<<get_id()<<" disconnecting from ROS");
    dummy_p.reset();}
  void update(void) {  }
  void init(){connect_to_ros();}
  void publish(int n) {
    if (is_connected_to_ros()) {
      std_msgs::Int64 mi;
      mi.data=n;
      dummy_p->publish(mi);
    }
  }
};

int main(int argc, char **argv){

  cafer_core::init(argc,argv,"basic_example_new_node");

  std::string management_topic="unset",type="basic_example_new_node";
  double freq;
  cafer_core::ros_nh->getParam("management_topic",management_topic);
  cafer_core::ros_nh->getParam("frequency",freq);
  ROS_INFO_STREAM("Management topic for new node: "<<management_topic<< " ; Namespace: "<<cafer_core::ros_nh->getNamespace());

  DummyClient cc(management_topic,type,freq);
  cc.wait_for_init();

  int cpt=0;

  while(ros::ok()&&(!cc.get_terminate())) {
    cc.spin();
    cc.publish(cpt);
    cc.update();
    cc.sleep();
    cpt++;
  }
  cc.shutdown();
  return 0;
}
