CAFER [![Build Status](https://travis-ci.org/robotsthatdream/cafer.svg?branch=master)](https://travis-ci.org/robotsthatdream/cafer)
=====

This is the source code of the CAFER framework, to be used within the DREAM project (http://www.robotsthatdream.eu/). This project aims to enable robots to gain an open-ended understanding of the world over long periods of time, with alternating periods of experience and sleep, based on a cognitive architecture that exploits sleep.

CAFER (Cognitive Architecture Framework based on Evolutionary Robotics) is a robust and light-weight framework aimed at facilitating the implementation of DREAM cognitive architecture. CAFER runs on top of a standard robotics middleware called ROS (Robotics Operating System). CAFER is composed of nodes, topics and services. Regarding the DREAM project, CAFER aims at helping the integration of the different components implemented by the project partners.

The main CAFER features are:
* Workflow management of an experiment
* Storage and reuse of episodic memories
* Information sharing within the different steps of an experiment, and among different experiments
* Available in C++ (core) and Python (wrapper)

Authors
-------
- Stephane Doncieux stephane.doncieux@isir.upmc.fr
- Leni Legoff le_goff@isir.upmc.fr
- Carlos Maestre carlos.maestre@isir.upmc.fr
- Pierre-Henri Le Fur pierre-henri.le-fur@edu.ece.fr
- Ghanim Mukhtar mukhtar@isir.upmc.fr

Content
-------

This package contains all the files required to compile and launch the cpp version of the cafer core ROS modules. It also contains tests and basic examples.

Get started
=====

How to build and configure CAFER
--------------------------------

1- Compilers Compatibility 

First make sure your gcc and g++ are above version 5. You can check for the version simply by:
```
gcc --version
g++ --version
```
Then if they are not above 5 you can fix that as follows:
```
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get install gcc-5 g++-5

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 60 --slave /usr/bin/g++ g++ /usr/bin/g++-5
```
In the following, we assume that your catkin workspace is ~/catkin_ws.

2- If you don't have a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd  ~/catkin_ws/src
catkin_init_workspace
```
And to set up permanently your environment :
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

3- Install CAFER at the right place

Move the CAFER directory to ~/catkin_ws/src (or create a symbolic link pointing to it).
Example, assuming that you have cloned CAFER into ~/git:
```
cd  ~/catkin_ws/src
ln -s ~/git/cafer
```

4- Compile CAFER
As CAFER is a catkin workspace, to build the project you must simply do :
```
cd ~/catkin_ws
catkin_make
sudo -s (to enter in root mode)
catkin_make install -DCMAKE_INSTALL_PREFIX=/usr/local (to install CAFER in your system)
exit (to exit from root mode)
```

If you haven't root access on your computer simply do :
```
catkin_make install
```
It will install in ~/{YOUR_GIT_FOLDER}/cafer/install/

5- Configure the Python wrapper

```
cd  ~/catkin_ws/src/cafer/example_python/
ln -s ~/catkin_ws/devel/lib/component.so
```

6- Test CAFER

CAFER comes with some tests. Some require up to several minutes to run. That is normal.

Then we can run the tests:
```
cd ~/catkin_ws
catkin_make run_tests
```
Or (for a more concise output):
```
cd ~/catkin_ws
catkin_make test
```

7- Testing Connection of Two waves

In this test there is one wave that simply launch one node which publishes, periodically, a "hellow world" message. Then there is the database manager node which its interaction with the first wave is defined as listening to this topic and recording it for 5 second each iteration. 
Then there is a second wave that simply accesses asks the database manager to access those data stored by the first wave and print number of lines in each file stored.

To run the test you need to follow these steps, assuming you have cafer installed as indicated above:

A- Clone the necessary packages:
```
cd ~/catkin_ws/src
git clone https://github.com/ghanimmukhtar/example_publisher_wave_cafer.git 
git clone https://github.com/ghanimmukhtar/simple_publisher.git
git clone https://github.com/ghanimmukhtar/example_sub_pub_db_cafer.git
git clone https://github.com/ghanimmukhtar/example_subscriber_wave_cafer.git
```
B- Compile everything:
```
cd ~/catkin_ws
catkin_make
```
C- Make a directory with the name "ros_hame" which will be the database storage location:
```
mkdir -p ~/ros_home
```
D- Set ROS_HOME environment variable to point to "ros_home":
```
export ROS_HOME=~/ros_home
```
E- Launch cafer core, then wave 1, then wave 2 and finally the database manager:
```
roslaunch cafer_core cafer.launch
roslaunch example_publisher_wave_cafer Supervisor_wave_1.launch
roslaunch examplroslaunch example_sub_pub_db_cafer db_manager.launche_subscriber_wave_cafer subscriber_wave_supervisor.launch
```

Further information 
=====

Available in the [Wiki](https://github.com/robotsthatdream/cafer/wiki)

* [Reference Manual](https://github.com/robotsthatdream/cafer/wiki/Reference-Manual)
* [Example execution (C++)](https://github.com/robotsthatdream/cafer/blob/master/example/README.md)
* [Example explanation (C++)] (https://github.com/robotsthatdream/cafer/wiki/Example-explanation-(CPP))
* [Example execution (Python)](https://github.com/robotsthatdream/cafer/tree/python_wrapper/example_python/README.md)
* [Example explanation (Python)](https://github.com/robotsthatdream/cafer/wiki/Example-explanation-(Python))
* [Data Manager Example (C++)](https://github.com/robotsthatdream/cafer/wiki/Data-Manager-Example)
* [Code Reference](http://robotsthatdream.github.io/namespacecafer__core.html)

IMPORTANT: CAFER is under development and current version should not be considered as stable.
