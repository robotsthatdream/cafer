
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
#include "../src/component.hpp"
#include <cafer_core/Management.h>
#include <tbb/tbb.h>
#include <ros/package.h>

class DummyClient {
public:
  void disconnect_from_ros(void) {}
  bool is_initialized(void){return true;}
  void update(void) {}
};

void test_launch_death(int i,bool local) {
  
  std::ostringstream os1,os2;
  os1<<"test_tbb_"<<local<<"_"<<i;
  os2<<"mgmt_ld_"<<local<<i;
  cafer_core::Component<DummyClient> cc(os2.str(), os1.str());
 
  ROS_INFO_STREAM("Launch-death iteration with tbb: "<<i);

  
  SCOPED_TRACE(os1.str());

  cc.wait_for_init();

  unsigned int nbdummy=0;
  int count=10;
  while((nbdummy==0)&&(count>0)) {
    cc.spin();
    cc.sleep();
    nbdummy=cc.how_many_client_from_type("dummy_node");
    count--;
  }

  EXPECT_EQ(0,count);
  EXPECT_EQ(0,nbdummy);

 
  std::string cafer_test_launch = ros::package::getPath("cafer_core")+"/test_launch/component_test_launch_death_dummy_node.launch";
  
  cc.call_launch_file(cafer_test_launch,cafer_core::ros_nh->getNamespace()+"/dummy_nodes");
  
  nbdummy=0;
  count=100;
  while((nbdummy==0)&&(count>0)) {
    cc.spin();
    cc.sleep();
    nbdummy=cc.how_many_client_from_type("dummy_node");
    count--;
  }
  
  EXPECT_NE(0,count);
  EXPECT_EQ(1,nbdummy);
  
  std::vector<cafer_core::ClientDescriptor> vcd;
  cc.get_connected_client_with_type("dummy_node", vcd);
  
  ASSERT_EQ(vcd.size(),1);
  
  ASSERT_TRUE(cc.is_client_up(vcd[0].ns, vcd[0].id));
  
  if (local)
    cc.send_local_node_death(vcd[0].ns, vcd[0].id);
  else {
    cc.send_complete_node_death(vcd[0].ns, vcd[0].id);
  }

  int cpt=10;
  while (cc.is_client_up(vcd[0].ns, vcd[0].id) && (cpt>0)) {
    cc.spin();
    cc.sleep();
    cpt--;
  }
  
  ASSERT_FALSE(cc.is_client_up(vcd[0].ns, vcd[0].id));
  
}

class ApplyTest {
  bool _local;
public:
  void operator()( const tbb::blocked_range<size_t>& r ) const {
    for( size_t i=r.begin(); i!=r.end(); ++i ) 
      test_launch_death(i,_local);
  }
  ApplyTest(bool local):_local(local) {}
};



TEST(Component, component_launch_local_death_tbb)
{

  int nb_launch_death;
  cafer_core::ros_nh->param(ros::this_node::getName()+"/nb_launch_death",nb_launch_death,10);
  ROS_INFO_STREAM("Launching "<<nb_launch_death<<" Launch-Die cycles (name="<<ros::this_node::getName()<<")");


  tbb::parallel_for(tbb::blocked_range<size_t>(0,nb_launch_death),ApplyTest(true));

}

TEST(Component, component_launch_global_death_tbb)
{

  int nb_launch_death;
  cafer_core::ros_nh->param(ros::this_node::getName()+"/nb_launch_death",nb_launch_death,10);
  ROS_INFO_STREAM("Launching "<<nb_launch_death<<" Launch-Die cycles (name="<<ros::this_node::getName()<<")");


  tbb::parallel_for(tbb::blocked_range<size_t>(0,nb_launch_death),ApplyTest(false));

}


int main(int argc, char **argv){

  tbb::task_scheduler_init init;

  testing::InitGoogleTest(&argc, argv);

  cafer_core::init(0,NULL,"component_test_tbb");


  return RUN_ALL_TESTS();
}
