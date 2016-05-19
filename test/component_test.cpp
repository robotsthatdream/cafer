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
#include <ros/impl/duration.h>

#include <gtest/gtest.h>

#include "cafer_core/component.hpp"
#include "cafer_core/Management.h"

class DummyClient : public cafer_core::Component {
    using cafer_core::Component::Component; // To inherit Component's constructor
public:
    void client_connect_to_ros(void)
    { }

    void client_disconnect_from_ros(void)
    { }

    void init()
    { }

    bool is_initialized(void)
    { return true; }

    void update(void)
    { }
};

// Declare a test
TEST(Component, cafer_core1)
{
    DummyClient cc1("component_test_management", "test1");
    DummyClient cc2("component_test_management", "test2");

    // Check ID generation
    ASSERT_NE(cc1.get_id(), -1);
    ASSERT_NE(cc2.get_id(), -1);
    EXPECT_NE(cc1.get_id(), cc2.get_id());

    // Check namespace value
    EXPECT_EQ(cafer_core::ros_nh->getNamespace(), cc1.get_namespace());

    // Check watchdog...
    ros::Time t1(cc1.get_watchdog(cc2.get_namespace(), cc2.get_id()));
    ros::Time t2(cc2.get_watchdog(cc1.get_namespace(), cc1.get_id()));

    // ... never received at first
    EXPECT_EQ(t1.toSec(), 0);
    EXPECT_EQ(t2.toSec(), 0);

    // wait for the client to be up (i.e. one watchdog message received, at least)
    cc1.wait_for_client(cc2.get_namespace(), cc2.get_id());
    cc2.wait_for_client(cc1.get_namespace(), cc1.get_id());

    // Check that the watchdog message has been taken into account
    t1 = cc1.get_watchdog(cc2.get_namespace(), cc2.get_id());
    t2 = cc2.get_watchdog(cc1.get_namespace(), cc1.get_id());

    EXPECT_NE(t1.toSec(), 0);
    EXPECT_NE(t2.toSec(), 0);

    // Check terminate management

    // False at first
    EXPECT_FALSE(cc1.get_terminate());
    EXPECT_FALSE(cc2.get_terminate());

    // send a terminate message to cc1
    ros::Publisher pub(cafer_core::ros_nh->advertise<cafer_core::Management>("component_test_management", 10));
    cafer_core::Management msg;
    msg.type = cafer_core::LOCAL_CLIENT_DEATH;
    msg.src_node = cafer_core::ros_nh->getNamespace();
    msg.src_id = -1;
    msg.dest_node = cafer_core::ros_nh->getNamespace();
    msg.dest_id = cc1.get_id();
    msg.data_int = 0;
    msg.data_flt = 0;
    msg.data_str = "";
    pub.publish(msg);

    cc1.spin();
    cc1.sleep();
    cc1.spin();
    cc1.sleep();
    // one spin&sleep may not be enough to receive the message...

    // check whether it has been received or not
    EXPECT_TRUE(cc1.get_terminate());
    EXPECT_FALSE(cc2.get_terminate());

    // sends a terminate message to every client
    msg.dest_node = "all";
    msg.dest_id = -1;
    msg.data_int = 0;
    msg.data_flt = 0;
    msg.data_str = "";
    pub.publish(msg);

    cc1.spin();
    cc1.sleep();
    cc1.spin();
    cc1.sleep();
    // one spin&sleep may not be enough to receive the message...

    // checks whether it has been received
    EXPECT_TRUE(cc1.get_terminate());
    EXPECT_TRUE(cc2.get_terminate());


}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    cafer_core::init(0, NULL, "component_test");


    return RUN_ALL_TESTS();
}
