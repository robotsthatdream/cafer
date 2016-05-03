CAFER CORE EXAMPLE
==================

These are simple examples to show how to use cafer.

New node
--------

This example shows how to define a simple CAFER client that does nothing but send a message containing a counter that is incremented in the node. It is used by other examples.

```
roslaunch cafer_core basic_example_new_node.launch 
```

The namespace, management_topic and frequency can be changed through command line arguments. If they are not specified, they take a default value. See  basic_example_new_node.launch for mode details.

Example of use:
```
roslaunch cafer_core basic_example_new_node.launch ns=my_ns management_topic:=my_mgmt
```

Launch set nodes
----------------

This is a basic example in which several nodes are created through the call to a launch_file. The program checks that they have been properly launched and then kills them and check that they have stopped appropriately. 


```
roslaunch cafer_core basic_example_launch_set_nodes.launch
```

See the launch files for the parameters that can be modified through the command line, e.g. the number of created nodes. Example of use:
```
roslaunch cafer_core basic_example_launch_set_nodes.launch nb_nodes:=10
```



Kill node topic type
--------------------

This example shows how to kill a node using the facilities offered by a CAFER Component.

command :
```
rosrun  cafer_core  basic_example_kill_node_topic_type  
```
usage : two args : a topic and a type 

Example of use:
1. Launch several nodes
```
roslaunch basic_example_new_node.launch ns:=basic0 management_topic:=/basic_mgmt &
roslaunch basic_example_new_node.launch ns:=basic1 management_topic:=/basic_mgmt &
roslaunch basic_example_new_node.launch ns:=basic2 management_topic:=/basic_mgmt &
```
rosnode list must display the 3 different nodes.

2. Kill them using the kill node example (the type of the created nodes is basic_example_new_node, see basic_example_new_node.cpp) 
```
rosrun cafer_core basic_example_kill_node_topic_type /basic_mgmt basic_example_new_node
```
rosnode list should not display the nodes anymore.

Change frequency
----------------
command : rosrun  cafer_core  basic_example_change_frequency  
usage : two args : a management topic and a frequence value

Example of use:
1. Launch several nodes
```
roslaunch basic_example_new_node.launch ns:=basic0 management_topic:=/basic_mgmt
roslaunch basic_example_new_node.launch ns:=basic1 management_topic:=/basic_mgmt
roslaunch basic_example_new_node.launch ns:=basic2 management_topic:=/basic_mgmt
```
2. Change their frequency using the basic_example change frequency:
```
rosrun cafer_core basic_example_change_frequency /basic_mgmt 40
```

ROS info log channel should contain messages saying that the frequency was modified.



Data manager
------------
```
roslaunch cafer_core data_manager_example.launch
```
Data_publisher_example executable is a part of data_manager_example.