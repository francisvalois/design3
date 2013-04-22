source devel/setup.bash
rm devel/lib/kinocto/kinoctoLongTest
cd build
make tests
cd ..
./devel/lib/kinocto/kinoctoLongTest
