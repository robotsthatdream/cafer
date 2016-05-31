//
// Created by phlf on 19/05/16.
//

#ifndef CAFER_CORE_CAFER_TYPES_H
#define CAFER_CORE_CAFER_TYPES_H

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

namespace cafer_core {

    //When ROS will be c++11 compliant (ROS 2), could be changed to std::shared_ptr
    template<typename T>
    using shared_ptr=boost::shared_ptr<T>;

    extern shared_ptr<ros::NodeHandle> ros_nh;

    using NodeHandle=ros::NodeHandle;
    using NodeHandlePtr=shared_ptr<ros::NodeHandle>;
    using NodeHandleConstPtr=const shared_ptr<ros::NodeHandle>;

    using Subscriber=ros::Subscriber;
    using SubscriberPtr=shared_ptr<ros::Subscriber>;
    using SubscriberConstPtr=const shared_ptr<ros::Subscriber>;

    using Publisher=ros::Publisher;
    using PublisherPtr=shared_ptr<ros::Publisher>;
    using PublisherConstPtr=const shared_ptr<ros::Publisher>;

}//cafer_core

#endif //CAFER_CORE_CAFER_TYPES_H
