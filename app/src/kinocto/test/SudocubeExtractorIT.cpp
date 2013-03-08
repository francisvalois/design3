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
        delete sudocubeExtractor;
        sudocubeExtractor = NULL;
    }
};

Mat loadSudocubeNo(int no) {
    char filename[255];
    sprintf(filename, "%s/%d.jpeg", "img/testSudocubesJpg", no);
    Mat img = imread(filename);
    if (!img.data) {
        cout << "SudocubeExtractorIT could not load img sudocube test" << endl;
    }

    return img;
}

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

Sudokube getSudokube2() {
    Sudokube sudokube;
    sudokube.setCaseValue(1, 1, 1, 8);
    sudokube.setCaseValue(1, 2, 2, 1);
    sudokube.setCaseValue(1, 3, 2, 4);
    sudokube.setCaseValue(1, 3, 3, 7);
    sudokube.setCaseValue(1, 4, 4, 3);
    sudokube.setCaseValue(3, 1, 1, 8);
    sudokube.setCaseValue(3, 2, 2, 1);
    sudokube.setCaseValue(3, 4, 1, 2);
    sudokube.setCaseValue(2, 1, 4, 5);
    sudokube.setCaseValue(2, 2, 2, 6);
    sudokube.setCaseValue(2, 3, 2, 4);

    sudokube.setRedCase(3, 4, 2);

    return sudokube;
}

Sudokube getSudokube3() {
    Sudokube sudokube;
    sudokube.setCaseValue(1, 1, 4, 8);
    sudokube.setCaseValue(1, 1, 3, 2);
    sudokube.setCaseValue(1, 1, 2, 6);
    sudokube.setCaseValue(1, 4, 1, 4);
    sudokube.setCaseValue(3, 2, 4, 4);
    sudokube.setCaseValue(3, 1, 1, 1);
    sudokube.setCaseValue(3, 4, 3, 3);
    sudokube.setCaseValue(2, 3, 2, 8);
    sudokube.setCaseValue(2, 1, 2, 5);
    sudokube.setCaseValue(2, 2, 3, 2);

    sudokube.setRedCase(2, 3, 3);

    return sudokube;
}


Sudokube getSudokube4() {
    Sudokube sudokube;
    sudokube.setCaseValue(1, 2, 4, 2);
    sudokube.setCaseValue(1, 1, 3, 7);
    sudokube.setCaseValue(1, 2, 1, 6);
    sudokube.setCaseValue(1, 3, 2, 5);
    sudokube.setCaseValue(1, 4, 1, 7);
    sudokube.setCaseValue(2, 1, 2, 4);
    sudokube.setCaseValue(2, 1, 4, 6);
    sudokube.setCaseValue(2, 2, 3, 7);
    sudokube.setCaseValue(2, 3, 1, 2);
    sudokube.setCaseValue(2, 4, 2, 1);
    sudokube.setCaseValue(3, 1, 1, 3);
    sudokube.setCaseValue(3, 2, 3, 1);
    sudokube.setCaseValue(3, 4, 1, 5);
    sudokube.setCaseValue(3, 4, 4, 8);

    sudokube.setRedCase(1, 4, 4);

    return sudokube;
}


TEST_F(SudocubeExtractorIT, testSudoku1_1) {
    Mat img = loadSudocubeNo(1);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_2) {
    Mat img = loadSudocubeNo(2);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_3) {
    Mat img = loadSudocubeNo(3);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_4) {
    Mat img = loadSudocubeNo(4);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_5) {
    Mat img = loadSudocubeNo(5);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_6) {
    Mat img = loadSudocubeNo(6);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_7) {
    Mat img = loadSudocubeNo(7);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_9) {
    Mat img = loadSudocubeNo(9);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_10) {
    Mat img = loadSudocubeNo(10);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_11) {
    Mat img = loadSudocubeNo(11);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_12) {
    Mat img = loadSudocubeNo(12);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_13) {
    Mat img = loadSudocubeNo(13);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_14) {
    Mat img = loadSudocubeNo(14);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_15) {
    Mat img = loadSudocubeNo(15);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube1();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}



TEST_F(SudocubeExtractorIT, testSudoku1_16) {
    Mat img = loadSudocubeNo(16);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube2();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_17) {
    Mat img = loadSudocubeNo(17);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube2();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_18) {
    Mat img = loadSudocubeNo(18);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube2();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_21) {
    Mat img = loadSudocubeNo(21);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube2();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_22) {
    Mat img = loadSudocubeNo(22);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube2();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_24) {
    Mat img = loadSudocubeNo(24);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube2();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_25) {
    Mat img = loadSudocubeNo(25);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube3();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_26) {
    Mat img = loadSudocubeNo(26);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube3();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_28) {
    Mat img = loadSudocubeNo(28);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube3();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_29) {
    Mat img = loadSudocubeNo(29);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube3();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_30) {
    Mat img = loadSudocubeNo(30);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube3();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_31) {
    Mat img = loadSudocubeNo(31);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube3();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_32) {
    Mat img = loadSudocubeNo(32);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube3();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_33) {
    Mat img = loadSudocubeNo(33);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube3();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_34) {
    Mat img = loadSudocubeNo(34);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube3();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_35) {
    Mat img = loadSudocubeNo(35);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube4();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_36) {
    Mat img = loadSudocubeNo(36);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube4();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_37) {
    Mat img = loadSudocubeNo(37);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube4();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_38) {
    Mat img = loadSudocubeNo(38);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube4();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_39) {
    Mat img = loadSudocubeNo(39);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube4();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_40) {
    Mat img = loadSudocubeNo(40);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube4();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

TEST_F(SudocubeExtractorIT, testSudoku1_42) {
    Mat img = loadSudocubeNo(42);
    Sudokube sudokubeExtracted = sudocubeExtractor->extractSudocube(img);
    Sudokube sudokube = getSudokube4();

    ASSERT_TRUE(sudokube.equals(sudokubeExtracted));
}

}

