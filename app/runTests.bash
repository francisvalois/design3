catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
source devel/setup.bash
cd build
make tests
cd ..
./devel/lib/kinocto/kinoctoTest
