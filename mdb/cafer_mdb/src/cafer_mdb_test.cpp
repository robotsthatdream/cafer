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

class exp_client : public cafer_core::AbstractClient {
    
    public:
        void disconnect_from_ros(void) {}
        bool is_initialized(void) {return true;}
        void update(void) {}
};

int main(int argc, char **argv){

    cafer_core::init(0,NULL,"cafer_mdb_test");

	std::string management_topic, type;
    double freq;
	int iterations;

    cafer_core::ros_nh->getParam("cafer_mdb_test/management_topic", management_topic);
    cafer_core::ros_nh->getParam("cafer_mdb_test/frequency", freq);
    cafer_core::ros_nh->getParam("cafer_mdb_test/type",type);
	cafer_core::ros_nh->getParam("cafer_mdb_test/iterations",iterations);
	

    cafer_core::Component<exp_client> cec (management_topic, type, freq);

    cec.wait_for_init();
    
    std::string cafer_rosmdb_node_launch = ros::package::getPath("cafer_mdb")+"/launch/cafer_mdb_node.launch";
    std::string namespace_base = cafer_core::ros_nh->getNamespace()+"/node";
    
    cec.call_launch_file(cafer_rosmdb_node_launch, namespace_base);
    
    while(ros::ok()&&(!cec.get_terminate())&&(iterations>0)){
        cec.spin();
        cec.sleep();
        iterations--;
    }
    
    std::vector<cafer_core::ClientDescriptor> vcd;
    cec.get_connected_client_with_type("cafer_mdb_node", vcd);
    if (vcd.size()==1){
        cec.send_local_node_death(vcd[0].ns, vcd[0].id);
    }
  return 0;
}
