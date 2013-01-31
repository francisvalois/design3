rm -r bin
mkdir bin
cd bin
cmake ./../
make
./SudokuReader ./../sudoku.png
