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
#include "cafer_core/component.hpp"
#include "cafer_core/Management.h"

#include <ros/package.h>
#include <ros/impl/duration.h>

class DummyClient : public cafer_core::Component {
    using cafer_core::Component::Component; // To inherit Component's constructor

public:
    void client_disconnect_from_ros(void)
    { }

    void client_connect_to_ros(void)
    { }

    void init(void)
    { }

    bool is_initialized(void)
    { return true; }

    void update(void)
    { }
};

/** local_node_death (i.e. terminate attribute) test */
TEST(Component, component_launch_death_local)
{
    DummyClient cc("my_mgmt_ld", "test_local");


    cc.wait_for_init();


    unsigned int nbdummy = 0;
    int count = 10;
    while ((nbdummy == 0) && (count > 0)) {
        cc.spin();
        cc.sleep();
        nbdummy = cc.how_many_client_from_type("dummy_node");
        count--;
    }

    EXPECT_EQ(0, count);
    EXPECT_EQ(0, nbdummy);


    int nb_launch_death;
    cafer_core::ros_nh->param(ros::this_node::getName() + "/nb_launch_death", nb_launch_death, 10);
    ROS_INFO_STREAM("Launching " << nb_launch_death << " Launch-Die cycles (name=" << ros::this_node::getName() << ")");
    std::string cafer_test_launch =
            ros::package::getPath("cafer_core") + "/test_launch/component_test_launch_death_dummy_node.launch";

    int i = 1;
    while (nb_launch_death) {
        std::ostringstream os;
        os << "test_" << i;
        SCOPED_TRACE(os.str());

        std::cout << "Launching the " << i << "th cycle. " << nb_launch_death << " to go." << std::endl;
        nb_launch_death--;

        cc.call_launch_file(cafer_test_launch, cafer_core::ros_nh->getNamespace() + "/dummy_nodes");

        sleep(5);

        nbdummy = 0;
        count = 100;
        while ((nbdummy == 0) && (count > 0)) {
            cc.spin();
            cc.sleep();
            nbdummy = cc.how_many_client_from_type("dummy_node");
            count--;
        }

        EXPECT_NE(0, count);
        EXPECT_EQ(1, nbdummy);

        std::vector<cafer_core::ClientDescriptor> vcd;
        cc.get_connected_client_with_type("dummy_node", vcd);

        ASSERT_EQ(vcd.size(), 1);

        ASSERT_TRUE(cc.is_client_up(vcd[0].ns, vcd[0].id));

        cc.send_local_node_death(vcd[0].ns, vcd[0].id);

        int cpt = 10;
        while (cc.is_client_up(vcd[0].ns, vcd[0].id) && (cpt > 0)) {
            cc.spin();
            cc.sleep();
            cpt--;
        }

        ASSERT_FALSE(cc.is_client_up(vcd[0].ns, vcd[0].id));

    }

}

/** complete_node_death (i.e. ros::shutdown) test */
TEST(Component, component_launch_death_complete)
{
    DummyClient cc("my_mgmt_ld", "test_global");


    cc.wait_for_init();


    unsigned int nbdummy = 0;
    int count = 10;
    while ((nbdummy == 0) && (count > 0)) {
        cc.spin();
        cc.sleep();
        nbdummy = cc.how_many_client_from_type("dummy_node");
        count--;
    }

    EXPECT_EQ(0, count);
    EXPECT_EQ(0, nbdummy);


    int nb_launch_death;
    cafer_core::ros_nh->param(ros::this_node::getName() + "/nb_launch_death", nb_launch_death, 10);
    ROS_INFO_STREAM("Launching " << nb_launch_death << " Launch-Die cycles (name=" << ros::this_node::getName() << ")");
    std::string cafer_test_launch =
            ros::package::getPath("cafer_core") + "/test_launch/component_test_launch_death_dummy_node.launch";

    int i = 1;
    while (nb_launch_death) {
        std::cout << "Launching the " << i << "th cycle. " << nb_launch_death << " to go." << std::endl;
        nb_launch_death--;

        cc.call_launch_file(cafer_test_launch, cafer_core::ros_nh->getNamespace() + "/dummy_nodes");

        nbdummy = 0;
        count = 100;
        while ((nbdummy == 0) && (count > 0)) {
            cc.spin();
            cc.sleep();
            nbdummy = cc.how_many_client_from_type("dummy_node");
            count--;
        }

        EXPECT_NE(0, count);
        EXPECT_EQ(1, nbdummy);

        std::vector<cafer_core::ClientDescriptor> vcd;
        cc.get_connected_client_with_type("dummy_node", vcd);

        ASSERT_EQ(vcd.size(), 1);

        ASSERT_TRUE(cc.is_client_up(vcd[0].ns, vcd[0].id));

        cc.send_local_node_death(vcd[0].ns, vcd[0].id);

        int cpt = 10;
        while (cc.is_client_up(vcd[0].ns, vcd[0].id) && (cpt > 0)) {
            cc.spin();
            cc.sleep();
            cpt--;
        }

        ASSERT_FALSE(cc.is_client_up(vcd[0].ns, vcd[0].id));

    }

}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);

    cafer_core::init(argc, argv, "component_test");


    return RUN_ALL_TESTS();
}
