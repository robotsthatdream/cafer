CAFER
=====

This is the source code of the CAFER framework, to be used within the DREAM project (http://www.robotsthatdream.eu/).

This is a set of ROS nodes providing services and publishing/subscribing to different topics.

Content
-------

* ROS: contains the CAFER ROS nodes (go to http://wiki.ros.org/indigo/Installation/Ubuntu for installation instuction)
* sferes: contains the code to connect the sferes software framework to ROS through CAFER

Authors
-------
- Stephane Doncieux stephane.doncieux@isir.upmc.fr
- Leni Legoff le_goff@isir.upmc.fr
- Carlos Maestre carlos.maestre@isir.upmc.fr

How to build and configure CAFER
--------------------------------

Before building the project :
```
cd ~/{YOUR_GIT_FOLDER}/cafer/src/
catkin_init_workspace
```

CAFER is a catkin workspace. To build the project you must simply do (in the cafer directory) :
```
catkin_make
sudo -s (to enter in root mode)
catkin_make install -DCMAKE_INSTALL_PREFIX=/usr/local (to install cafer in your system)
exit (to exit from root mode)
```

if you haven't root access on your computer simply do :
```
catkin_make install 
```
It will install in ~/{YOUR_GIT_FOLDER}/cafer/install/

And to set up permanently your environment :
```
echo "source ~/{YOUR_GIT_FOLDER}/cafer/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc
```

Organisation of CAFER
---------------------

There are two ROS packages cafer_server and cafer_client.
cafer_server define the services of CAFER and
cafer_client is a library which provide implementations of ROS clients to help the use the cafer's services.
For future contributions please follow this structure.
