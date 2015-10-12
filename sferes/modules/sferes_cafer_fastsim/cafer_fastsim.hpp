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

#ifndef _CAFER_FASTSIM_HPP_
#define _CAFER_FASTSIM_HPP_

#include <unistd.h>
#include <modules/sferes_cafer/sferes_cafer.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <tbb/tbb.h>
#include <ros/callback_queue.h>
#include "fastsim/Teleport.h"


namespace sferes_cafer {


  const std::string namespace_base="sferes_cafer_fastsim";

  class Posture {
  public:
    Posture(void):_x(0),_y(0),_theta(0){}
    Posture(float x, float y, float theta):_x(x),_y(y),_theta(theta){}
    float get_x(void) const {return _x;}
    float get_y(void) const {return _y;}
    float get_theta(void) const {return _theta;}
    float dist_to(Posture &dest) {
      return sqrt((_x-dest._x)*(_x-dest._x)+(_y-dest._y)*(_y-dest._y));
    }
    
    float _x;
    float _y;
    float _theta;
    
  };
  
  class cafer_fastsim {
  public:
    
    std::string _ros_namespace;
    
    // ROS handles
    boost::shared_ptr<ros::Subscriber> laser_s;
    boost::shared_ptr<ros::Subscriber> odom_s;
    boost::shared_ptr<ros::Subscriber> collision_s;
    boost::shared_ptr<ros::Publisher> speed_left_p;
    boost::shared_ptr<ros::Publisher> speed_right_p;
    
    // Sensor data (updated by the callbacks)
    tbb::concurrent_bounded_queue<sensor_msgs::LaserScan> lasers;
    tbb::concurrent_bounded_queue<nav_msgs::Odometry> odom;
    tbb::concurrent_bounded_queue<bool> collision;

    sensor_msgs::LaserScan lasers_current;
    nav_msgs::Odometry odom_current;
    bool collision_current;
    


    ~cafer_fastsim(void) {
      //std::cout<<" Function: "<<__func__<<" Line: "<<__LINE__<<" "<<this<<std::endl;
      disconnect_from_ros();
    }

    void disconnect_from_ros(void) {
      //std::cout<<" Function: "<<__func__<<" Line: "<<__LINE__<<" "<<this<<std::endl;
      laser_s.reset();
      odom_s.reset();
      collision_s.reset();
      speed_left_p.reset();
      speed_right_p.reset();
      sferes_cafer::release_node_group(namespace_base,_ros_namespace);
    }
    
    void init(const char *launch_file) {
      _ros_namespace = sferes_cafer::get_node_group(namespace_base,launch_file);
      std::cerr<<"ROS FASTSIM, namespace="<<_ros_namespace<<std::endl;
      if (_ros_namespace.find("<Failed>")!=std::string::npos) {
	std::cerr<<"ROS initialisation failed."<<std::endl;
	exit(1);
      }
      
      std::string topic_laser=_ros_namespace+"/simu_fastsim/laser_scan";
      laser_s.reset(new ros::Subscriber(ros_nh->subscribe(topic_laser.c_str(),10,&cafer_fastsim::laser_cb,this))); 
      std::string topic_odom=_ros_namespace+"/simu_fastsim/odom";
      odom_s.reset(new ros::Subscriber(ros_nh->subscribe(topic_odom.c_str(),10,&cafer_fastsim::odom_cb,this)));      
      std::string topic_collision=_ros_namespace+"/simu_fastsim/collision";
      collision_s.reset(new ros::Subscriber(ros_nh->subscribe(topic_collision.c_str(),10,&cafer_fastsim::collision_cb,this)));
      
      std::string topic_speed_left=_ros_namespace+"/simu_fastsim/speed_left";
      speed_left_p.reset(new ros::Publisher(ros_nh->advertise<std_msgs::Float32>(topic_speed_left.c_str(),10)));
      std::string topic_speed_right=_ros_namespace+"/simu_fastsim/speed_right";
      speed_right_p.reset(new ros::Publisher(ros_nh->advertise<std_msgs::Float32>(topic_speed_right.c_str(),10)));
      

      ros::Rate loop_rate(sferes_cafer::ros_frequency);

      while(!is_initialized()) {
	//std::cout<<"Waiting for initialization..."<<std::endl;
	ros::spinOnce();
	loop_rate.sleep();
	
      }
      //std::cout<<"Initialization done !"<<std::endl;
      update();
    }
    
    void collision_cb(const std_msgs::Bool &coll) {
      //std::cout<<" Function: "<<__func__<<" Line: "<<__LINE__<<" "<<this<<std::endl;
      collision.push(coll.data);
    }

    void laser_cb(const sensor_msgs::LaserScan &laser_msg){
      //std::cout<<" Function: "<<__func__<<" Line: "<<__LINE__<<" "<<this<<std::endl;
      lasers.push(laser_msg);
    }

    void odom_cb(const nav_msgs::Odometry &odom_msg) {
      //std::cout<<" Function: "<<__func__<<" Line: "<<__LINE__<<" "<<this<<std::endl;
      odom.push(odom_msg);
    }

    bool is_initialized(void) {
      bool init=true;
      int os=odom.size();
      int ls=lasers.size();
      int cs=collision.size();
      init=init && (os>0);
      init=init && (ls>0);
      init=init && (cs>0);
      //std::cout<<" Function: "<<__func__<<" Line: "<<__LINE__<<" odom.size="<<os<<" lasers.size="<<ls<<" coll.size="<<cs<<" "<<this<<std::endl;
      return init;
    }

    void update(void) {
      //std::cout<<" Function: "<<__func__<<" Line: "<<__LINE__<<" "<<this<<std::endl;
      ros::spinOnce();

      lasers.try_pop(lasers_current);
      odom.try_pop(odom_current);
      collision.try_pop(collision_current);
    }

    void publish_speed(float speed_left, float speed_right) {
      //std::cout<<" Function: "<<__func__<<" Line: "<<__LINE__<<std::endl;
      std_msgs::Float32 sl,sr;
      sl.data=speed_left;
      sr.data=speed_right;
      speed_left_p->publish(sl);
      speed_right_p->publish(sr);
    }

    void teleport(float x, float y, float theta) {
      //std::cout<<"Teleporting the robot."<<std::endl;
      fastsim::Teleport v;
      v.request.x=x;
      v.request.y=y;
      v.request.theta=theta;
      std::string teleport=_ros_namespace+"/simu_fastsim/teleport";
      ros::ServiceClient client = ros_nh->serviceClient<fastsim::Teleport>(teleport.c_str());
      if (client.call(v)) {
	if (!v.response.ack) {
	  std::cerr<<"cafer_fastsim: teleport failed ! Position: "<<x<<" "<<y<<" "<<theta<<std::endl;
	}
      }
      //std::cout<<"== END == Teleporting the robot."<<std::endl;
    }

    Posture get_pos(void) {
      Posture pos(odom_current.pose.pose.position.x,odom_current.pose.pose.position.y,tf::getYaw(odom_current.pose.pose.orientation) );
      return pos;
    }

  };


}


#endif
