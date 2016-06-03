CAFER
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

In the following, we assume that your catkin workspace is ~/catkin_ws.

1- If you don't have a catkin workspace:
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

2- Install CAFER at the right place

Move the CAFER directory to ~/catkin_ws/src (or create a symbolic link pointing to it).
Example, assuming that you have cloned CAFER into ~/git:
```
cd  ~/catkin_ws/src
ln -s ~/git/cafer
```

3- Compile CAFER
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

4- Configure the Python wrapper

```
cd  ~/catkin_ws/src/cafer/example_python/
ln -s ~/catkin_ws/devel/lib/component.so
```

5- Test CAFER

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
