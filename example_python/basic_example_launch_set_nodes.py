#!/usr/bin/python

import rospy
from cafer_core.msg import *
from std_msgs.msg import String
import sys
import rospkg

import component

class DummyClient (component.Component) :
	# def __init__(self,*args, **kwargs):
	# 	super(DummyClient, self ).__init__(*args, **kwargs)

	def client_connect_to_ros(self):
		pass

	def client_disconnect_from_ros(self):
		pass

	def update (self):
		pass

	def init (self):
		pass

	def is_initialized (self):
		return True

def set_nodes_function():

	## Parameters
	nb_nodes = 2;

	''' New node '''
	component.python_init('basic_example_launch_set_nodes')

	name_tmp = rospy.search_param('management_topic')
	param_dict = rospy.get_param(name_tmp)['basic_example_launch_set_nodes']
	management_topic = param_dict['management_topic']
	type_value = param_dict['type_value']
	nb_nodes = param_dict['nb_nodes']
	freq = param_dict['frequency']
	ns = component.python_get_node_name()
	# ns = rospy.get_namespace()
	# print("Launching "+str(nb_nodes)+")  new nodes (name="+rospy.get_name()+")")	
	print("Launching "+str(nb_nodes)+" new nodes (name="+ns+")")

	''' Create the component in charge of calling the launch file for creating the new nodes '''
	cc = DummyClient(management_topic, type_value, int(freq), False)
	cc.wait_for_init()	

	''' Finding the path towards the launch_file to call '''
	rospack = rospkg.RosPack()
	basic_example_new_node_launch = rospack.get_path('cafer_core') + '/launch/basic_example_new_node.launch'

	''' Creation of the new nodes '''
	created_namespaces = []
	for i in range(0,nb_nodes):
		print("Params : " + " " + ns + " " + management_topic + " " + type_value + " " + str(nb_nodes) + " " + str(freq))
		created_namespaces.append(cc.call_launch_file(basic_example_new_node_launch, ns+'/basic_node', management_topic))
		cc.sleep()

	''' Checking that they are up '''
	nb_tries=10
	count = -1
	while nb_tries > 0 :
		count=20
		# while (count>0) and len(cc.python_get_created_nodes_id) != nb_nodes :
		print ('cc.python_get_created_nodes_number() : ', cc.python_get_created_nodes_number())
		while (count>0) and cc.python_get_created_nodes_number() != nb_nodes :		
			cc.spin()
			cc.update()
			cc.sleep()
			count = count - 1

		if count == 0 :
			print("PROBLEM: we haven't received the ack from some nodes. We ask for a new ack.")
			print("============= Ack received from "+cc.get_created_nodes().size()+" components: ")
			# node_id_vector = cc.python_get_created_nodes_id()

			# for node_id in node_id_vector:
			# 	print("Component: id="+cd.id)

			cc.python_print_created_nodes_id()

			print("=========== End ack received")
			print("")

			cc.ask_new_ack()
			cc.spin()
			cc.update()
			cc.sleep()

		else :
			break    

		nb_tries = nb_tries -1

	if nb_tries  == 0:
		print("PROBLEM: the nodes haven't been all launched.")
		while not rospy.is_shutdown() and not cc.get_terminate() :
			cc.spin()
			cc.update()
			cc.sleep()
		sys.exit()

	''' At this point at least one node from each call to call_launch_file has answered. 
	As there is only one node in the launch file, it should be fine, but just to be sure 
	(and to show how to check it) we verify that all of those nodes are up. '''
	all_up = cc.python_clients_status("up")

	if not all_up :
		print("PROBLEM: the nodes aren't up.")
		sys.exit()

	''' We kill all created_nodes '''
	cc.kill_created_nodes()

	''' We check that they are down '''
	count = 100*nb_nodes
	all_down = False
	while (count>0) and not all_down :
		all_down = cc.python_clients_status("down")
		cc.spin()
		cc.update()
		cc.sleep()
		count = count -1

	if not all_down :
		print("PROBLEM: the nodes aren't down after a kill.")
		sys.exit()
	else :
		print("GREAT ! All nodes were created and killed as expected (count=" + str(count) + ")")

	''' The roslaunch may need to be stopped with a ctrl-c as the getid node will go on working after this node exits. '''

	print('Done.')

if __name__ == '__main__':
	try:
		set_nodes_function()
	except rospy.ROSInterruptException:
		pass