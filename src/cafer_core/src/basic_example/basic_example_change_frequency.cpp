
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
#include <gtest/gtest.h>
#include <unistd.h>
#include "../component.hpp"
#include "cafer_core/Management.h"
#include <sstream>


int main(int argc, char **argv){

  int freq_value = 10;
  if (argc != 3){
    ROS_INFO_STREAM("Correct use : rosrun  cafer_core  basic_example_change_frequency  management_topic freq_value");
    return 0;
  } else {
    std::stringstream ss (argv[2]);    
    ss >> freq_value;    
  }

  ros::init(argc, argv, "change_freq_node");		
  ros::NodeHandle nh;
  std::string management_topic = argv[1];
  ROS_INFO_STREAM("management_topic" << management_topic);
  ros::Publisher p=nh.advertise<cafer_core::Management>(management_topic,freq_value);

  sleep(1);  

  cafer_core::Management msg;
  msg.type=cafer_core::CHG_FREQ;
  msg.src_node="";
  msg.src_id=-1;
  msg.dest_node="all";
  msg.dest_id=-1;
  msg.data_int=0;
  msg.data_flt=freq_value;
  msg.data_str="";

  ROS_INFO_STREAM("Publishing the message to change frequency");
  p.publish(msg);

  return 0;
}
