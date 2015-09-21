#include <string>
#include <std_msgs/String.h>
#include "ros/ros.h"
#include "ros_sferes/GetID.h"
#include  <boost/unordered_map.hpp>
/** 
  Service to get a unique ID associated to a string.
*/

typedef boost::unordered_map<std::string, int> map_ID_t;
map_ID_t map_ID;

bool get_id(
        ros_sferes::GetID::Request  &req,
        ros_sferes::GetID::Response &res)
{

  if (map_ID.find(req.name) == map_ID.end()) {
    map_ID[req.name]=0;
  }
  else {
    map_ID[req.name]++;
  }
  res.id=map_ID[req.name];
  ROS_INFO("request: name=%s", req.name.c_str());
  ROS_INFO("sending back response: [%ld]", (long int)res.id);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getID_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("/ros_sferes/get_id", get_id);
  ROS_INFO("Ready to provide new ID for SFERES nodes.");
  ros::spin();

  return 0;
}