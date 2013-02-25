rm -r bin
rm -r Debug
mkdir bin
cd bin
cmake ./../
make
cp SudokuReader ../SudokuReader
./../SudokuReader
