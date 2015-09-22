CAFER
=====

This is the source code of the CAFER framework, to be used within the DREAM project (http://www.robotsthatdream.eu/).

This is a set of ROS nodes providing services and publishing/subscribing to different topics.

Content
-------

* ROS: contains the CAFER ROS nodes
* sferes: contains the code to connect the sferes software framework to ROS through CAFER

Authors
-------
- Stephane Doncieux stephane.doncieux@isir.upmc.fr
- Leni Legoff
- Carlos Maestre carlos.maestre@isir.upmc.fr

ROS Usage
---------

Create a link from the ROS dir to your ROS catkin environment. 

WARNING: the ROS package name should be cafer

This should be done with something like this (to adapt to the directories where you have put the source files):
```
cd ~/catkin_ws/src
ln -s ~/git/cafer/ROS cafer
```

then compile it:
```
cd ~/catkin_ws/
catkin_make
```

To launch the CAFER modules:
```
roslaunch cafer cafer.launch
```

SFERES Usage
------------

SFERES is not required to use CAFER. Modules are provided for those that want to use this framework for evolutionary robotics.

You will need to download SFERES first: https://github.com/sferes2

Assuming that sferes is in ~/git/sferes2, you will need to create symlinks pointing to each dir in cafer/sferes/modules in sferes2/modules. If you want to test the provided examples, you will also need to  create symlinks pointing to dirs in cafer/sferes/exp in sferes2/exp.

You should then do something like this:
```
cd ~/git/sferes2/modules
ln -s ~/git/cafer/sferes/modules/sferes_cafer
ln -s ~/git/cafer/sferes/modules/sferes_cafer_fastsim
cd ~/git/sferes2/exp
ln -s ~/git/cafer/sferes/exp/example_cafer_fastsim
```

To compile the experiment:
```
# To take into account cafer modules
echo sferes_cafer >> ~/git/sferes2/modules.conf
echo sferes_cafer_fastsim >> ~/git/sferes2/modules.conf

# To compile the exp
./waf build --exp=example_cafer_fastsim
```
