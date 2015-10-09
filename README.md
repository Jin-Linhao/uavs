# UAVs for Internet of Things Lab
	Edited by Jeffsan Chen Wang

##To use python, you need to install scipy, numpy etc.
	sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose

	sudo apt-get install python-numpy python-scipy python-matplotlib mayavi2 ipython python-setuptools python-simpy python-pyparsing

##To use python pymc, drawing stream and generate *.gif animation files, you need install pymc and its dependecies
	sudo apt-get install gfortran
	sudo apt-get install mencoder
	sudo apt-get install imagemagick
	git clone https://github.com/pymc-devs/pymc.git
	cd pymc
	sudo python setup.py config_fc --fcompiler gfortran build
	sudo python setup.py install

##To use kalman filter in python, you need to install pykalman
 	sudo easy_install numpy scipy Sphinx numpydoc nose pykalman
 	you can find more information in : http://pykalman.github.io

##To use package about vision, you need to install opencv2.4.9, later we will migrate to opencv3.0

When you install opencv2.4.9, Please remember to ENABLE NONFREE MODULE and DISABLE CUDA MODULE.

you can set opencv2.4.9 compile settings using: 

	cmake -D WITH_CUDA=OFF -D CMAKE_BUILD_TYPE=RELEASE -D WITH_OPENCL=OFF ..

when you compile and it says can not load nonfree module, please run:

	sudo ldconfig -v
	
##(7/Oct/2015) Some algrithm is faster in OpenCV 3.0. You can find our new OpenCV 3.0 relatated Repository: https://github.com/JeffsanC/drones.git


## To use gazebo and rviz for quadcopter simulation, you need to install hector-quadrotor
	wget -O /tmp/gazebo6_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo6_install.sh; sudo sh /tmp/gazebo6_install.sh
	
	sudo apt-get install ros-hydro-hector-quadrotor-demo
	
If you have problems when you install gazebo, you can run:
	
	sudo apt-get remove .*gazebo.* && sudo apt-get update && sudo apt-get install gazebo6

## For svo slam or lsd package(two 3D vision slam method), they need their own dependencies, pleas refer to each readme file in respective package



