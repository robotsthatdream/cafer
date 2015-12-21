CAFER CORE EXAMPLE
==================

How to launch the examples :

Change frequency
----------------
command : rosrun  cafer_core  basic_example_change_frequency  
usage : two args : a management topic and a frequence value


Kill node topic type
--------------------
command : rosrun  cafer_core  basic_example_kill_node_topic_type 
usage : two args : a topic and a type 


Launch set nodes
----------------
```
roslaunch cafer_core basic_example_launch_set_nodes.launch
```

New node
--------
```
roslaunch cafer_core basic_example_new_node.launch ns:=namespace
```


Data manager
------------
```
roslaunch cafer_core data_manager_example.launch
```
Data_publisher_example executable is a part of data_manager_example.