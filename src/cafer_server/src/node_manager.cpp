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
#include "cafer_server/LaunchNode.h"
#include "cafer_server/CleanNodeGroup.h"
#include "cafer_server/KillNodeGroup.h"
#include "cafer_server/ReleaseNode.h"
#include "cafer_server/GetID.h"
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
        cafer_server::LaunchNode::Request  &req,
        cafer_server::LaunchNode::Response &res)
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

  cafer_server::LaunchNode v;
  v.request.namespace_base = req.namespace_base;
  v.request.launch_file = req.launch_file;
  v.request.frequency = req.frequency;
  ros::ServiceClient client = n->serviceClient<cafer_server::LaunchNode>("launch_node");
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
        cafer_server::ReleaseNode::Request  &req,
        cafer_server::ReleaseNode::Response &res)
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
        cafer_server::CleanNodeGroup::Request  &req,
        cafer_server::CleanNodeGroup::Response &res)
{
  ROS_INFO("Clean node group: %s.",req.namespace_base.c_str());
  if (ngm.find(req.namespace_base)==ngm.end()) {
    ROS_ERROR("No such node exists: %s",req.namespace_base.c_str());
    std::cerr<<"No such node exists: "<<req.namespace_base.c_str()<<std::endl;
    res.ack=false;
    return false;
  }
  std::list<node_group> &l=ngm[req.namespace_base];

  for (std::list<node_group>::iterator it=l.begin();it!=l.end();) {
    if((!req.availableOnly)|| (it->available)) {
      // kill the nodes in the corresponding namespace
      std::string nodes=exec(std::string("rosnode list "+it->gr_namespace).c_str());
      std::istringstream iss(nodes);
      std::string node;
      while (std::getline(iss, node)) {
	std::string cmd("rosnode kill "+node);
	system(cmd.c_str());
	ROS_INFO("Killing node: %s", node.c_str());      
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
        cafer_server::KillNodeGroup::Request  &req,
        cafer_server::KillNodeGroup::Response &res)
{
  ROS_INFO("Killing node group: %s in namespace_base=%s.",req.gr_namespace.c_str(),req.namespace_base.c_str());
  if (ngm.find(req.namespace_base)==ngm.end()) {
    ROS_ERROR("No such node exists: %s",req.namespace_base.c_str());
    res.ack=false;
    return false;
  }
  std::list<node_group> &l=ngm[req.namespace_base];
  for (std::list<node_group>::iterator it=l.begin();it!=l.end();) {

    if(!it->gr_namespace.compare(req.gr_namespace)){
      // kill the nodes in the corresponding namespace
      ROS_INFO("Killing node: %s",it->gr_namespace.c_str());
      std::string nodes=exec(std::string("rosnode list "+it->gr_namespace).c_str());
      std::istringstream iss(nodes);
      std::string node;
      while (std::getline(iss, node)) {
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
