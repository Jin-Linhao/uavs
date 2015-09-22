#  Mono-SLAM for Internet of Things Lab
	Edited by Wang Chen:

##To use python, you need to install scipy, numpy etc.
sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas python-sympy python-nose

sudo apt-get install python-numpy python-scipy python-matplotlib mayavi2 ipython python-setuptools python-simpy python-pyparsing

##To use kalman filter in python, you need to install pykalman
 	sudo easy_install numpy scipy Sphinx numpydoc nose pykalman
 	you can find more information in : http://pykalman.github.io

##To use package about vision, you need to install opencv2.4.9, later we will migrate to opencv3.0

When you install opencv2.4.9, Please remember to ENABLE NONFREE MODULE and DISABLE CUDA MODULE.

you can set opencv2.4.9 compile settings using: 

	cmake -D WITH_CUDA=OFF -D CMAKE_BUILD_TYPE=RELEASE -D WITH_OPENCL=OFF ..

when you compile and it says can not load nonfree module, please run:

	sudo ldconfig -v

## To use gazebo and rviz for quadcopter simulation, you need to install hector-quadrotor
	wget -O /tmp/gazebo6_install.sh http://osrf-distributions.s3.amazonaws.com/gazebo/gazebo6_install.sh; sudo sh /tmp/gazebo6_install.sh
	sudo apt-get install ros-hydro-hector-quadrotor-demo

## For svo slam or lsd package(two 3D vision slam method), they need their own dependencies, pleas refer to each readme file in respective package



