//| This file is a part of the CAFER framework developped within
//| the DREAM project (http://www.robotsthatdream.eu/).
//|
//| Copyright 2015, GII / Universidad de la Coruna (UDC)
//| Main contributor(s): 
//|   * Rodrigo Salgado, rodrigo.salgado@udc.es
//|   * Pilar Caamano, pilar.caamano@udc.es
//|   * Juan Monroy, juan.monroy@udc.es
//|   * Luis Calvo, luis.calvo@udc.es
//|   * Jose Antonio Becerra, jose.antonio.becerra.permuy@udc.es
//|
//| This file is also part of MDB.
//| 
//| * MDB is free software: you can redistribute it and/or modify it under the
//| * terms of the GNU Affero General Public License as published by the Free
//| * Software Foundation, either version 3 of the License, or (at your option) any
//| * later version.
//| *
//| * MDB is distributed in the hope that it will be useful, but WITHOUT ANY
//| * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
//| * A PARTICULAR PURPOSE. See the GNU Affero General Public License for more
//| * details.
//| *
//| * You should have received a copy of the GNU Affero General Public License
//| * along with MDB. If not, see <http://www.gnu.org/licenses/>.

#include <ros/ros.h>
#include <cafer_core/Management.h>
#include <cafer_core/cafer_core.hpp>
#include <ros/package.h>
#include <std_msgs/String.h>

class cafer_mdb_node_client : public cafer_core::AbstractClient {
    ros::Publisher rosmdb_cafer_pub;
    
    public:
        void disconnect_from_ros(void) {
            finish_rosmdb();
            
        }
        bool is_initialized(void){
			std::string topic = cafer_core::ros_nh->getNamespace()+"/rosmdb_nodes/rosmdb_topic";
            rosmdb_cafer_pub = cafer_core::ros_nh->advertise<std_msgs::String>(topic,10);
            launch_rosmdb();
            return true;
        }
        void update(void) {}
    
        void launch_rosmdb(void) {
        	std::string rosmdb_launch_file = ros::package::getPath("rosmdb")+"/launch/cafer_rosmdb.launch";
            std::string namespace_base = cafer_core::ros_nh->getNamespace();
			std::string::iterator beg = namespace_base.begin()+namespace_base.find_first_not_of('/')-1;
			namespace_base.erase(namespace_base.begin(), beg);
			std::string namespace_constructed = namespace_base+"/rosmdb_nodes";
            std::string ns="ns:="+namespace_constructed;
            std::string cmd="roslaunch "+rosmdb_launch_file+" "+ns+"&";
            system(cmd.c_str());
        }
        void finish_rosmdb (void) {
            std_msgs::String msg;
            msg.data ="stop";
            rosmdb_cafer_pub.publish(msg);
        }
};

int main(int argc, char **argv){

    cafer_core::init(argc,argv,"cafer_mdb_node");
    
    std::string management_topic, type;
    double freq;
    cafer_core::ros_nh->getParam("cafer_mdb_node/management_topic", management_topic);
    cafer_core::ros_nh->getParam("cafer_mdb_node/frequency", freq);
    cafer_core::ros_nh->getParam("cafer_mdb_node/type",type);
    
    cafer_core::Component<cafer_mdb_node_client> cafer_mdb_node_client (management_topic,type,freq);
    cafer_mdb_node_client.wait_for_init();

    while(ros::ok()&&(!cafer_mdb_node_client.get_terminate())) {
        cafer_mdb_node_client.spin();
        cafer_mdb_node_client.update();
        cafer_mdb_node_client.sleep();
    }
    
  return 0;
}
