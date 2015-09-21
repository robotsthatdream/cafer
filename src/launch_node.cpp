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
  static ros::ServiceClient client = n->serviceClient<ros_sferes::GetID>("/ros_sferes/get_id");
  if (client.call(v))
    {
      std::ostringstream os, osf;
      os<<req.namespace_base<<"_"<<v.response.id;
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
  ros::ServiceServer service = n->advertiseService("/ros_sferes/launch_node", launch_node);
  ROS_INFO("Ready to launch new nodes on demand.");
  ros::spin();

  return 0;
}
