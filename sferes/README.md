SFERES module for CAFER
=======================

Modules for connecting SFERES2 to CAFER.

SFERES is not required to use CAFER. Modules are provided for those that want to use this framework for evolutionary robotics.

You will need to download SFERES first: https://github.com/sferes2

Assuming that sferes is in ~/git/sferes2, you will need to create symlinks pointing to each dir in cafer/sferes/modules in sferes2/modules. If you want to test the provided examples, you will also need to  create symlinks pointing to dirs in cafer/sferes/exp in sferes2/exp.

Go in your git folder and do the folowing instructions.

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
cd ~/git/cafer/src/
ln -s ~/git/ros_fastsim/
cd ..
catkin_make
```

You should then do something like this:
```
cd ~/git/sferes2/modules
ln -s ~/git/cafer/sferes/modules/ros_binding
ln -s ~/git/cafer/sferes/modules/sferes_cafer_fastsim
echo ros_binding >> modules.conf
echo sferes_cafer_fastsim >> modules.conf
cd ~/git/sferes2/exp
ln -s ~/git/cafer/sferes/exp/example_cafer_fastsim
```

To install sferes2 :
```
git clone https://github.com/sferes2/sferes2.git
cd sferes2
./waf configure
./waf build
sudo ./waf install
```


To compile the experiment:
```
./waf build --exp=example_cafer_fastsim
```
 