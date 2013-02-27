rm -r devel build install
cd src
rm CMakeLists.txt
catkin_init_workspace
cd ..
catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
source devel/setup.bash
