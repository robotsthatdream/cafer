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

To create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

To compile the package:
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


And, you need some sferes's modules :
```
git clone https://github.com/sferes2/fastsim.git
git clone https://github.com/sferes2/nn2.git
cd ~/git/sferes2/modules
ln -s ~/git/fastsim
ln -s ~/git/nn2
echo fastsim >> modules.conf
echo nn2 >> modules.conf
```
And then install two libraries libfastsim and ros_fastsim :
```
git clone https://github.com/jbmouret/libfastsim.git
cd libfastsim
./waf configure
./waf build
./waf install

cd ..
git clone https://github.com/jbmouret/ros_fastsim.git
cd ros_fastsim
./waf configure
./waf build
./waf install
```

You should then do something like this:
```
cd ~/git/sferes2/modules
ln -s ~/git/cafer/sferes/modules/sferes_cafer
ln -s ~/git/cafer/sferes/modules/sferes_cafer_fastsim
cd ~/git/sferes2/exp
ln -s ~/git/cafer/sferes/exp/example_cafer_fastsim
```

To install sferes2 :
```
cd git
git clone https://github.com/sferes2/sferes2.git
cd sferes2
./waf configure
./waf build
./waf install
```


To compile the experiment:
```
# To take into account cafer modules
echo sferes_cafer >> ~/git/sferes2/modules.conf
echo sferes_cafer_fastsim >> ~/git/sferes2/modules.conf

# To compile the exp
./waf build --exp=example_cafer_fastsim
```
 