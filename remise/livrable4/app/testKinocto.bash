source devel/setup.bash
rm devel/lib/kinocto/kinoctoTest
cd build
make tests
cd ..
./devel/lib/kinocto/kinoctoTest
