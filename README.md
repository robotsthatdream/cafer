CAFER
=====

This is the source code of the CAFER framework, to be used within the DREAM project (http://www.robotsthatdream.eu/).

This is a set of ROS nodes providing services and publishing/subscribing to different topics.

Content
-------

This package contains all the files required to compile and launch the cpp version of the cafer core ROS modules. It also contains tests and basic examples.

Authors
-------
- Stephane Doncieux stephane.doncieux@isir.upmc.fr
- Leni Legoff le_goff@isir.upmc.fr
- Carlos Maestre carlos.maestre@isir.upmc.fr
- Pierre-Henri Le Fur pierre-henri.le-fur@edu.ece.fr
- Ghanim Mukhtar mukhtar@isir.upmc.fr

How to build and configure CAFER
--------------------------------

In the following, we assume that your catkin workspace is ~/catkin_ws.

1. If you don't have a catkin workspace:
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

2. Install CAFER at the right place

Move the CAFER directory to ~/catkin_ws/src (or create a symbolic link pointing to it).
Example, assuming that you have cloned CAFER into ~/git:
```
cd  ~/catkin_ws/src
ln -s ~/git/cafer
```

3. Compile CAFER
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

4. Test CAFER

CAFER comes with some tests. Some require up to several minutes to run. That is normal.

To run them, first we need to run ROS and CAFER:
```
roscore

roslaunch cafer_core cafer.launch
```
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

5. Basic examples

See example/README.md
