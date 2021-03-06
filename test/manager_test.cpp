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

#include <thread>
#include <atomic>
#include <ros/ros.h>
#include <gtest/gtest.h>

#include "cafer_core/manager.hpp"
#include "cafer_core/manager_test.h"

class DummyData : public cafer_core::Data {
    using cafer_core::Data::Data;

public:
    std::map<std::string, std::string> get_serialized_data() const override
    {
        std::stringstream data;
        std::map<std::string, std::string> serialized_data;
        cafer_core::shared_ptr<cafer_core::manager_test> msg;

        msg = _stored_msg.instantiate<cafer_core::manager_test>();
        data << "header: " << std::endl;
        data << "  frame_id: " << msg->header.frame_id << std::endl;
        data << "  seq: " << msg->header.seq << std::endl;
        data << "  stamp: " << msg->header.stamp << std::endl;
        data << "description: " << msg->description << std::endl;
        data << "tags:" << std::endl;
        for (uint32_t i = 0; i < msg->tags.size(); ++i) {
            data << "  tag_" << i << ": " << msg->tags[i] << std::endl;
        }
        data << "content: " << msg->content << std::endl;

        serialized_data["test_record"] = data.str();

        return serialized_data;
    }
};

/**
 * TODO: Rewrite ManagerMap and the corresponding test.
 * @brief TEST test for the basis function of manager : add, random access, search and remove.
 */
//TEST(Manager, ManagerMap)
//{
//
//    cafer_core::ManagerMap<cafer_core::manager_test> manager("test", "manager test");
//
//    //you can create a message
//    cafer_core::manager_test msg1;
//    std_msgs::Header header;
//    header.seq = 1;
//    header.stamp = ros::Time(2.0);
//    header.frame_id = "0";
//
//    msg1.header = header;
//    msg1.description = "I am a message";
//    msg1.tags = {"t", "d", "e"};
//    msg1.content = "this is not a content";
//
//    //and add it to the data manager
//    manager.add(msg1);
//
//    cafer_core::manager_test msg2 = manager.get();
//
//    EXPECT_TRUE(msg1.header.seq == msg2.header.seq);
//    EXPECT_TRUE(msg1.header.stamp == msg2.header.stamp);
//    EXPECT_TRUE(msg1.header.frame_id == msg2.header.frame_id);
//
//    EXPECT_TRUE(msg1.description == msg2.description);
//    EXPECT_TRUE(msg1.tags == msg2.tags);
//    EXPECT_TRUE(msg1.content == msg2.content);
//
//    cafer_core::manager_test msg3;
//    header.seq = 2;
//    header.stamp = ros::Time(3.0);
//    header.frame_id = "frame";
//
//    msg3.header = header;
//    msg3.description = "I am another message";
//    msg3.tags = {"tt", "ds", "es"};
//    msg3.content = "content";
//
//    manager.add(msg3);
//
//    cafer_core::manager_test msg4;
//    bool is_exist = manager.search(2,msg4);
//
//    EXPECT_TRUE(is_exist);
//    EXPECT_TRUE(msg3.header.seq == msg4.header.seq);
//    EXPECT_FALSE(msg4.header.seq == msg2.header.seq);
//    EXPECT_TRUE(msg3.header.stamp == msg4.header.stamp);
//    EXPECT_FALSE(msg2.header.stamp == msg4.header.stamp);
//    EXPECT_TRUE(msg3.header.frame_id == msg4.header.frame_id);
//    EXPECT_FALSE(msg2.header.frame_id == msg4.header.frame_id);
//
//    EXPECT_TRUE(msg3.description == msg4.description);
//    EXPECT_FALSE(msg2.description == msg4.description);
//    EXPECT_TRUE(msg3.tags == msg4.tags);
//    EXPECT_FALSE(msg2.tags == msg4.tags);
//    EXPECT_TRUE(msg3.content == msg4.content);
//    EXPECT_FALSE(msg2.content == msg4.content);
//
//    cafer_core::manager_test msg5 = manager.get();
//
//    EXPECT_TRUE(msg3.header.seq == msg5.header.seq || msg1.header.seq == msg5.header.seq);
//    EXPECT_TRUE(msg3.header.stamp == msg5.header.stamp || msg1.header.stamp == msg5.header.stamp);
//    EXPECT_TRUE(msg3.header.stamp == msg5.header.stamp || msg1.header.stamp == msg5.header.stamp);
//    EXPECT_TRUE(msg3.header.frame_id == msg5.header.frame_id || msg1.header.frame_id == msg5.header.frame_id);
//
//    EXPECT_TRUE(msg3.description == msg5.description || msg1.description == msg5.description);
//    EXPECT_TRUE(msg3.tags == msg5.tags || msg1.tags == msg5.tags);
//    EXPECT_TRUE(msg3.content == msg5.content || msg1.content == msg5.content);
//
//    size_t msg3_size = manager.remove(2);
//
//    EXPECT_EQ(msg3_size, 1);
//
//    msg2 = manager.get();
//
//    EXPECT_TRUE(msg1.header.seq == msg2.header.seq);
//    EXPECT_TRUE(msg1.header.stamp == msg2.header.stamp);
//    EXPECT_TRUE(msg1.header.frame_id == msg2.header.frame_id);
//
//    EXPECT_TRUE(msg1.description == msg2.description);
//    EXPECT_TRUE(msg1.tags == msg2.tags);
//    EXPECT_TRUE(msg1.content == msg2.content);
//
//}


/**
 * @brief TEST test for the basis function of manager : add, get.
 */
TEST(Manager, ManagerQueue)
{
    cafer_core::ManagerQueue<DummyData> manager;

    // Create a message
    topic_tools::ShapeShifter msg;
    topic_tools::ShapeShifter msg1;
    msg1.morph("dummy_checksum","dummy_datatype","dummy_definition","dummy_latching");

    // and add it to the data manager
    manager.add(msg);
    EXPECT_TRUE(manager.data_size()==1);

    manager.add(msg1);
    EXPECT_TRUE(manager.data_size()==2);

    auto retrieved_msg=manager.get();
    EXPECT_TRUE(manager.data_size()==1);

    EXPECT_TRUE(retrieved_msg->get_stored_msg().getDataType().empty());
    EXPECT_TRUE(retrieved_msg->get_stored_msg().getMD5Sum().empty());
    EXPECT_TRUE(retrieved_msg->get_stored_msg().getMessageDefinition().empty());

    retrieved_msg=manager.get();
    EXPECT_TRUE(manager.data_size()==0);

    EXPECT_TRUE(retrieved_msg->get_stored_msg().getDataType()=="dummy_datatype");
    EXPECT_TRUE(retrieved_msg->get_stored_msg().getMD5Sum()=="dummy_checksum");
    EXPECT_TRUE(retrieved_msg->get_stored_msg().getMessageDefinition()=="dummy_definition");
}

/**
 * Test if conccurrent access to the ManagerQueue's DataContainer works.
 * If not, it will segfault. Unfortunately, it is not really possible to that cleaner.
 */
TEST(Manager, ManagerQueue_concurrrency)
{
    cafer_core::ManagerQueue<DummyData> manager;

    // Create a message
    topic_tools::ShapeShifter dummy_msg;

    std::thread consumer_thread([&manager]()
                                {
                                    for (unsigned int i = 0; i < 100000; ++i) {
                                        if (manager.data_size() > 0) {
                                            manager.get();
                                        }
                                    }
                                });

    for (unsigned int i = 0; i < 10000; ++i) {
        manager.add(dummy_msg);
    }

    consumer_thread.join();
    SUCCEED();
}

/**
 * Test if conccurrent access to the ManagerMap's DataContainer works.
 * If not, it will do a dirty segfault.
 */
//TEST(Manager, ManagerMap_concurrrency)
//{
//    cafer_core::ManagerMap<cafer_core::manager_test> manager("test", "manager test");
//    cafer_core::manager_test dummy_msg;
//    std_msgs::Header header;
//    std::atomic<unsigned int> data_race_sync(0);
//
//    header.stamp = ros::Time(2.0);
//    header.frame_id = "0";
//
//    dummy_msg.description = "I am a message";
//    dummy_msg.tags = {"t", "d", "e"};
//    dummy_msg.content = "this is not a content";
//
//    for (unsigned int i = 0; i < 100000; ++i) {
//        data_race_sync.store(false);
//        header.seq = i + 1;
//
//        dummy_msg.header = header;
//
//        manager.add(dummy_msg);
//    }
//
//    std::thread consumer_thread_1([&manager, &dummy_msg, &data_race_sync]()
//                                  {
//                                      while (data_race_sync.load() < 100000) {
//                                          manager.remove(data_race_sync.load());
//                                          ++data_race_sync;
//                                      }
//                                  });
//
//    std::thread consumer_thread_2([&manager, &dummy_msg, &data_race_sync]()
//                                  {
//                                      while (data_race_sync.load() < 100000) {
//                                          manager.remove(data_race_sync.load());
//                                          ++data_race_sync;
//                                      }
//                                  });
//
//    consumer_thread_1.join();
//    consumer_thread_2.join();
//    SUCCEED();
//}

int main(int argc, char **argv)
{

    testing::InitGoogleTest(&argc, argv);

    cafer_core::init(0, NULL, "manager_test_subcribe");

    return RUN_ALL_TESTS();
}
