#include <string>
#include <sstream>
#include <std_msgs/String.h>
#include "ros/ros.h"
#include "cafer/LaunchNode.h"
#include "cafer/CleanNodeGroup.h"
#include "cafer/KillNodeGroup.h"
#include "cafer/ReleaseNode.h"
#include "cafer/GetID.h"
#include <boost/unordered_map.hpp>
#include <boost/shared_ptr.hpp>
#include <unistd.h>
/** 
  Service to get a free node or set of nodes
*/

std::string exec(const char* cmd) {
  FILE* pipe = popen(cmd, "r");
  if (!pipe) return "ERROR";
  char buffer[128];
  std::string result = "";
  while(!feof(pipe)) {
    if(fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }
  pclose(pipe);
  return result;
}

boost::shared_ptr<ros::NodeHandle> n;

class node_group {
public:
  std::string gr_namespace;
  float frequency;
  std::string launch_file;
  bool available;

  node_group(const std::string &ns, float freq, const std::string &lf, bool av):gr_namespace(ns), frequency(freq), launch_file(lf), available(av) {}

};

typedef boost::unordered_map<std::string, std::list<node_group> > node_group_map_t;
node_group_map_t ngm;

bool get_node_group(
        cafer::LaunchNode::Request  &req,
        cafer::LaunchNode::Response &res)
{


  if (ngm.find(req.namespace_base)!=ngm.end()) {
    std::list<node_group> &l=ngm[req.namespace_base];
    for (std::list<node_group>::iterator it=l.begin();it!=l.end();it++) {
      if (it->available) {
	if ((it->launch_file!=req.launch_file)||(it->frequency!=req.frequency)) {
	  ROS_ERROR("Problem: conflict between launch_file and frequency: ");
	  ROS_ERROR("Existing: %s frequency=%f", it->launch_file.c_str(), it->frequency);
	  ROS_ERROR("Requested: %s frequency=%f", req.launch_file.c_str(), req.frequency);
	  continue;
	}

	ROS_INFO("Reserving node group: %s", it->gr_namespace.c_str());
	it->available=false;
	res.created_namespace=it->gr_namespace;
	// WARNING: makes the assumption that the node has been launched with the appropriate frequency. 
	// The frequency can't be updated on the fly.
	return true;
      }
    }
  }

  cafer::LaunchNode v;
  v.request.namespace_base = req.namespace_base;
  v.request.launch_file = req.launch_file;
  v.request.frequency = req.frequency;
  ros::ServiceClient client = n->serviceClient<cafer::LaunchNode>("/cafer/launch_node");
  std::string ns="<Failed>";
  if (client.call(v))
    {
      ns=v.response.created_namespace;
      ROS_INFO("Node group request successful: namespace=%s launch file=%s", ns.c_str(), req.launch_file.c_str());

      if (ngm.find(req.namespace_base)==ngm.end()) {
	std::list<node_group> ll;
	ngm[req.namespace_base]=ll;
      }
      node_group ng(v.response.created_namespace, req.frequency, req.launch_file, false);
      ngm[req.namespace_base].push_back(ng);
      res.created_namespace=v.response.created_namespace;

    }
  else
    {
      ROS_ERROR("Failed to call service launch_node");
      return false;
    }

  return true;
}

bool release_node_group(
        cafer::ReleaseNode::Request  &req,
        cafer::ReleaseNode::Response &res)
{

  if (ngm.find(req.namespace_base)==ngm.end()) {
    ROS_ERROR("No such node exists: %s",req.namespace_base.c_str());
    res.ack=false;
    return false;
  }
  std::list<node_group> &l=ngm[req.namespace_base];
  for (std::list<node_group>::iterator it=l.begin();it!=l.end();it++) {
    if (it->gr_namespace == req.gr_namespace) {
      if (it->available)
	ROS_WARN("Trying to release a node group that is already free.");

      ROS_INFO("Releasing node group: %s", req.gr_namespace.c_str());
      it->available=true;
      res.ack=true;
      return true;
    }
  }

  ROS_WARN("Trying to release a node group that does not exist: %s.", req.gr_namespace.c_str());
  res.ack=false;
  return false;
}

bool clean_node_group(
        cafer::CleanNodeGroup::Request  &req,
        cafer::CleanNodeGroup::Response &res)
{
  if (ngm.find(req.namespace_base)==ngm.end()) {
    ROS_ERROR("No such node exists: %s",req.namespace_base.c_str());
    res.ack=false;
    return false;
  }
  std::list<node_group> &l=ngm[req.namespace_base];
  for (std::list<node_group>::iterator it=l.begin();it!=l.end();) {
    if((!req.availableOnly)|| (it->available)) {
      // kill the nodes in the corresponding namespace
      std::string nodes=exec(std::string("rosnode list /"+it->gr_namespace).c_str());
      std::istringstream iss(nodes);
      std::string node;
      while (std::getline(iss, node)) {
	std::cout<<"Node to kill: "<<node<<std::endl;
	std::string cmd("rosnode kill "+node);
	system(cmd.c_str());
      }
      it=l.erase(it);
    }
    else
      it++;
  }

  res.ack=true;
  return true;

}

bool kill_node_group(
        cafer::KillNodeGroup::Request  &req,
        cafer::KillNodeGroup::Response &res)
{
  if (ngm.find(req.namespace_base)==ngm.end()) {
    ROS_ERROR("No such node exists: %s",req.namespace_base.c_str());
    res.ack=false;
    return false;
  }
  std::list<node_group> &l=ngm[req.namespace_base];
  for (std::list<node_group>::iterator it=l.begin();it!=l.end();) {
    if(!it->gr_namespace.compare(req.gr_namespace)){
      // kill the nodes in the corresponding namespace
      std::string nodes=exec(std::string("rosnode list /"+it->gr_namespace).c_str());
      std::istringstream iss(nodes);
      std::string node;
      while (std::getline(iss, node)) {
	std::cout<<"Node to kill: "<<node<<std::endl;
	std::string cmd("rosnode kill "+node);
	system(cmd.c_str());
      }
      it=l.erase(it);
    }
    else
      it++;
  }

  res.ack=true;
  return true;
  

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_manager");

  n.reset(new ros::NodeHandle);
  ros::ServiceServer service1 = n->advertiseService("get_node_group",get_node_group );
  ros::ServiceServer service2 = n->advertiseService("release_node_group",release_node_group );
  ros::ServiceServer service3 = n->advertiseService("clean_node_group",clean_node_group );
  ros::ServiceServer service4 = n->advertiseService("kill_node_group",kill_node_group );
  ROS_INFO("Ready to manage node groups on demand.");
  ros::spin();

  return 0;
}
