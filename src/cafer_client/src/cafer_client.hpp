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

#ifndef _SFERES_CAFER_HPP
#define _SFERES_CAFER_HPP

#include <ros/ros.h>
#include "cafer_server/LaunchNode.h"
#include "cafer_server/ReleaseNode.h"
#include "cafer_server/KillNodeGroup.h"
#include <ros/spinner.h>


namespace cafer_client {

 
  extern boost::shared_ptr<ros::NodeHandle> ros_nh;
  extern float ros_frequency;

  void init(int argc, char **argv, std::string node_name, float frequency);
  std::string get_node_group(std::string namespace_base, std::string launch_file);
  void release_node_group(std::string namespace_base, std::string gr_namespace);
  void kill_node_group(std::string namespace_base, std::string gr_namespace);
  void kill_all_allocated_node_groups(void);


}

#endif
