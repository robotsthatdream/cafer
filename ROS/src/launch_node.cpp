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

#include <string>
#include <sstream>
#include <std_msgs/String.h>
#include "ros/ros.h"
#include "ros_sferes/LaunchNode.h"
#include "ros_sferes/GetID.h"
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <unistd.h>
/** 
  Service to launch an instance of a ROS node
*/

typedef boost::unordered_map<std::string, int> map_ID_t;
map_ID_t map_ID;

boost::shared_ptr<ros::NodeHandle> n;


bool launch_node(
        ros_sferes::LaunchNode::Request  &req,
        ros_sferes::LaunchNode::Response &res)
{

  ros_sferes::GetID v;
  v.request.name = req.namespace_base;
  static ros::ServiceClient client = n->serviceClient<ros_sferes::GetID>("get_id");
  if (client.call(v))
    {
      std::ostringstream os, osf;
      os<<"/"<<req.namespace_base<<"_"<<v.response.id;
      res.created_namespace=os.str();
      std::string ns="namespace:="+os.str();
      osf<<"frequency:="<<req.frequency;
      std::string cmd="roslaunch "+req.launch_file+" "+ns+" "+osf.str()+"&";
      system(cmd.c_str());
      ROS_INFO("Launch node(s): namespace=%s launch file=%s frequency=%f", res.created_namespace.c_str(), req.launch_file.c_str(), (float)req.frequency );
    }
  else
    {
      ROS_ERROR("Failed to call service get_id");
      return false;
    }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "launch_node_server");

  n.reset(new ros::NodeHandle);
  ros::ServiceServer service = n->advertiseService("launch_node", launch_node);
  ROS_INFO("Ready to launch new nodes on demand.");
  ros::spin();

  return 0;
}
