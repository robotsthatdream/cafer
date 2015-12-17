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
#include <cafer_core/manager.hpp>
#include <cafer_core/component.hpp>
#include <cafer_core/manager_test.h>

/**
 * @brief TEST test for the basis function of manager : add, random access, search and remove.
 */
TEST(Manager,cafer_core1){

    cafer_core::Manager<cafer_core::manager_test> manager("test","manager test");

    //you can create a message
    cafer_core::manager_test msg1;
    std_msgs::Header header;
    header.seq = 1;
    header.stamp = ros::Time(2.0);
    header.frame_id = "0";

    msg1.header = header;
    msg1.description = "I am a message";
    msg1.tags = {"t","d","e"};
    msg1.content = "this is not a content";

    //and add it to the data manager
    manager.add(msg1);

    cafer_core::manager_test msg2 = manager.get();

    EXPECT_TRUE(msg1.header.seq == msg2.header.seq);
    EXPECT_TRUE(msg1.header.stamp == msg2.header.stamp);
    EXPECT_TRUE(msg1.header.frame_id == msg2.header.frame_id);

    EXPECT_TRUE(msg1.description == msg2.description);
    EXPECT_TRUE(msg1.tags == msg2.tags);
    EXPECT_TRUE(msg1.content == msg2.content);

    cafer_core::manager_test msg3;
    header.seq = 2;
    header.stamp = ros::Time(3.0);
    header.frame_id = "frame";

    msg3.header = header;
    msg3.description = "I am another message";
    msg3.tags = {"tt","ds","es"};
    msg3.content = "content";

    manager.add(msg3);

    cafer_core::manager_test msg4 = manager.search(2);

    EXPECT_TRUE(msg3.header.seq == msg4.header.seq);
    EXPECT_FALSE(msg4.header.seq == msg2.header.seq);
    EXPECT_TRUE(msg3.header.stamp == msg4.header.stamp);
    EXPECT_FALSE(msg2.header.stamp == msg4.header.stamp);
    EXPECT_TRUE(msg3.header.frame_id == msg4.header.frame_id);
    EXPECT_FALSE(msg2.header.frame_id == msg4.header.frame_id);

    EXPECT_TRUE(msg3.description == msg4.description);
    EXPECT_FALSE(msg2.description == msg4.description);
    EXPECT_TRUE(msg3.tags == msg4.tags);
    EXPECT_FALSE(msg2.tags == msg4.tags);
    EXPECT_TRUE(msg3.content == msg4.content);
    EXPECT_FALSE(msg2.content == msg4.content);

    cafer_core::manager_test msg5 = manager.get();

    EXPECT_TRUE(msg3.header.seq == msg5.header.seq || msg1.header.seq == msg5.header.seq);
    EXPECT_TRUE(msg3.header.stamp == msg5.header.stamp || msg1.header.stamp == msg5.header.stamp);
    EXPECT_TRUE(msg3.header.frame_id == msg5.header.frame_id || msg1.header.frame_id == msg5.header.frame_id);

    EXPECT_TRUE(msg3.description == msg5.description || msg1.description == msg5.description);
    EXPECT_TRUE(msg3.tags == msg5.tags || msg1.tags == msg5.tags);
    EXPECT_TRUE(msg3.content == msg5.content || msg1.content == msg5.content);

    size_t msg3_size = manager.remove(2);

    EXPECT_EQ(msg3_size,1);

    msg2 = manager.get();

    EXPECT_TRUE(msg1.header.seq == msg2.header.seq);
    EXPECT_TRUE(msg1.header.stamp == msg2.header.stamp);
    EXPECT_TRUE(msg1.header.frame_id == msg2.header.frame_id);

    EXPECT_TRUE(msg1.description == msg2.description);
    EXPECT_TRUE(msg1.tags == msg2.tags);
    EXPECT_TRUE(msg1.content == msg2.content);

}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);

    cafer_core::init(0,NULL,"manager_test_subcribe");

  return RUN_ALL_TESTS();
}
