MDB Experiment
==============

Authors
-------
- Rodrigo Salgado, rodrigo.salgado@udc.es
- Pilar Caamano, pilar.caamano@udc.es
- Juan Monroy, juan.monroy@udc.es
- Luis Calvo, luis.calvo@udc.es
- Jose Antonio Becerra, jose.antonio.becerra.permuy@udc.es

Description:
------------

The experiment consists in a simulated scenario divided in two parts by a wall. The robot is placed in the left part of the scenario, where it can sense a blue box and a green button. In the right side of the scenario there is a red ball that it cannot initially sense because of the wall.

The robot is able to move on this environment and to reach the different objects. If it reaches the button, the wall disappears so it can see the ball. If it reaches the ball, it automatically picks it up. Finally, if it reaches the box when it is carrying the ball, it will receive a reward. When this happens, the ball is placed where it was at the beginning and the robot is placed in a random place of the left part.

With this configuration, the robot has to learn a complex and ordered behavior.

For more information about this experiment and the motivational engine behind it, MotivEn, check this paper:

R.Salgado, A.Prieto, P.Caamaño, F.Bellas, R.J.Duro: **MotivEn: Motivational Engine with Sub-goal Identification for Autonomous Robots.** The Leading European Event on Bio-Inspired Computation, Evostar 2016 conference. Submitted.


Installation:
-------------

MDB is programmed in java, so it is essential to install the java version of ros.

- Install gradle:
```
sudo apt-get install gradle 
```

- Create a ROSJava ws:
```
source /opt/ros/jade/setup.bash
mkdir -p ~/ROSJava/ros_ws
cd ~/ROSJava/ros_ws
wstool init src
```

- Download and build the ROSJava packages:
```
cd ~/ROSJava/ros_ws/src
git clone -b indigo https://github.com/ROSJava/ROSJava_build_tools.git
git clone -b indigo https://github.com/ROSJava/ROSJava_core.git 
git clone -b indigo https://github.com/ROSJava/ROSJava_test_msgs.git
git clone -b indigo https://github.com/ROSJava/genjava.git
git clone -b indigo https://github.com/ROSJava/ROSJava_messages.git
cd ..
catkin_make
source devel/setup.bash
```

- Generate message artifacts:
```
roscd genjava
scripts/genjava_message_artifacts
```

- NOTE: If there are any problems up until this point, check the tutorial this one is based on:
http://visionlab.uncc.edu/dokuwiki/rosjava
 
- Move the rosmdb package to the java src folder and build it:
``` 
cd ~/cafer/mdb
mv rosmdb ~/ROSJava/ros_ws/src
roscd rosmdb
chmod +x gradlew
cd ../..
catkin_make
roscd rosmdb
./gradlew deployApp
```

- Move the cafer_mdb package to the cafer src folder:
```
mv ~/cafer/mdb/cafer_mdb ~/cafer/src
```

- Overlay the cafer ws to the ROSJava ws and build the cafer packages:
```
cd ~/cafer/src
catkin_init_ws
cd ..
rm -rf devel build
catkin_make
source devel/setup.bash
```

Running the experiment:
-----------------------
```
export ROS_HOME=~/ROSJava/ros_ws/src/rosmdb 
roslaunch cafer_mdb cafer_mdb_test.launch
```

The experiment will run until cafer_mdb_test notifies cafer_mdb_node to close itself. For practical reasons, the experiment is configured to run for a total of 2000 iterations and there are no result logs saved, as an example of the mdb use through cafer. 

If you want to save the logs:
- Go to ~/ROSJava/ros_ws/src/rosmdb/__door
- Edit the MDB-Config.xml and MDB-Config-sim.xml files and uncomment all the log entries.
- After you run the experiment, the logs will be saved in the rosmdb/__door/_log folder.

If you want to change the number of iterations:
- Go to ~/ROSJava/ros_ws/src/rosmdb/__door
- Edit the MDB-Config.xml and MDB-Config-sim.xml files and change the number between the maxIterations tags (same in both files).
- Go to ~/cafer/src/cafer_mdb/launch
- Edit the cafer_mdb_test.launch file and change the default value of the "iterations" argument to half the value of the maxIterations tag from the MDB-Config.xml and MDB-Config-sim.xml files. 


