#include <gtest/gtest.h>

#include "SudocubeExtractor.h"

using namespace cv;
using namespace std;

namespace {
class SudocubeExtractorIT: public ::testing::Test {
public:

protected:
    SudocubeExtractor * sudocubeExtractor;

    virtual void SetUp() {
        sudocubeExtractor = new SudocubeExtractor();
    }

    virtual void TearDown() {
        //delete sudocubeExtractor;
        //sudocubeExtractor = NULL;
    }
};

/*TEST_F(SudocubeExtractorIT, simpleTest) {
 char filename[255];

 for (int i = 1; i <= 42; i++) {
 sprintf(filename, "%s/%d.png", "img/testSudocubes", i);
 Mat src = imread(filename);

 if (!src.data) {
 cout << "SudocubeExtractorIT could not load img sudocube test" << endl;
 }

 sudocubeExtractor->extractSudocube(src);
 }

 ASSERT_TRUE(true);
 }*/

Sudokube getSudokube1() {
    Sudokube sudokube;
    sudokube.setCaseValue(1, 1, 4, 8);
    sudokube.setCaseValue(1, 1, 3, 2);
    sudokube.setCaseValue(1, 1, 2, 6);
    sudokube.setCaseValue(1, 4, 1, 4);
    sudokube.setCaseValue(2, 1, 2, 5);
    sudokube.setCaseValue(2, 2, 3, 2);
    sudokube.setCaseValue(2, 3, 2, 8);
    sudokube.setCaseValue(3, 1, 1, 1);
    sudokube.setCaseValue(3, 2, 4, 4);
    sudokube.setCaseValue(3, 4, 3, 3);
    sudokube.setRedCase(1, 3, 3);
    return sudokube;
}

TEST_F(SudocubeExtractorIT, testSudoku1_1) {
    char filename[255];
    sprintf(filename, "%s/%d.png", "img/testSudocubes", 1);
    Mat img = imread(filename);
    if (!img.data) {
        cout << "SudocubeExtractorIT could not load img sudocube test" << endl;
    }
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

}

