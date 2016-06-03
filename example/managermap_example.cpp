//| This file is a part of the CAFER framework developped within
//| the DREAM project (http://www.robotsthatdream.eu/).
//| Copyright 2015, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s):
//|   * Stephane Doncieux, stephane.doncieux@isir.upmc.fr
//|   * Léni Le Goff, le_goff@isir.upmc.fr
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
#include "cafer_core/manager_test.h"

int main(int argc, char **argv)
{


    //initiate the cafer core
    cafer_core::init(argc, argv, "managermap_example");

    std::string topic;
    cafer_core::ros_nh->getParam("/manager_ns/managermap_example/topic",topic);

    //create the data manager for the manager_test msg
    cafer_core::ManagerMap<cafer_core::manager_test> manager("example", "example1");

    //listen to a specific topic
    manager.listen_to(topic);


    //listen until manager have 10 messages in his data set
    while (manager.data_size() < 10) {
        ros::spinOnce();
    }

    cafer_core::manager_test msg;

    //and get it with the random access getter
    msg = manager.get();

    ROS_INFO_STREAM( "Message info :\n"
              << "   id : " << msg.header.seq << std::endl
              << "   timestamp : " << msg.header.stamp << std::endl
              << "   description : " << msg.description << std::endl
              << "   content :" << msg.content << std::endl);

    //you can search the first message arrived
    int i = 0;
    while(!manager.search(i,msg))
        i++;

    ROS_INFO_STREAM( "Message info :\n"
              << "   id : " << msg.header.seq << std::endl
              << "   timestamp : " << msg.header.stamp << std::endl
              << "   description : " << msg.description << std::endl
              << "   content :" << msg.content << std::endl);

    //you can remove it
    manager.remove(i);
    if(!manager.search(i,msg))
        ROS_INFO_STREAM("the message with id " << i << " doesn't exist");


    return 0;
}
