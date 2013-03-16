#include <gtest/gtest.h>

#include "SudocubeExtractor.h"

using namespace cv;
using namespace std;

namespace {

class SudocubeExtractorIT: public ::testing::Test {
protected:
    SudocubeExtractor sudocubeExtractor;
};

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

Mat loadSudocubeNo(int no) {
    char filename[255];
    sprintf(filename, "%s/%d.jpeg", "img/testSudocubesJpg", no);
    Mat img = imread(filename);
    if (!img.data) {
        cout << "SudocubeExtractorIT could not load img sudocube test" << endl;
    }

    return img;
}

bool isSudocubeCorrectlyExtracted(int sudocubeNo, Sudokube correctSudocube, SudocubeExtractor & sudocubeExtractor) {
    bool correctlyExtracted = false;
    Mat img = loadSudocubeNo(sudocubeNo);
    Sudokube * sudokubeExtracted = sudocubeExtractor.extractSudocube(img);
    correctlyExtracted = correctSudocube.equals(*sudokubeExtracted);
    delete sudokubeExtracted;

    return correctlyExtracted;
}

TEST_F(SudocubeExtractorIT, testSudoku1_1) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(1, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_2) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(2, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_3) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(3, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_4) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(4, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_5) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(5, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_6) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(6, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_7) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(7, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_9) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(9, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_10) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(10, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_11) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(11, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_12) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(12, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_13) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(13, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_14) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(14, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_15) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(15, getSudokube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_16) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(16, getSudokube2(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_17) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(17, getSudokube2(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_18) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(18, getSudokube2(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_21) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(21, getSudokube2(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_22) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(22, getSudokube2(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_24) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(24, getSudokube2(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_25) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(25, getSudokube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_26) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(26, getSudokube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_28) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(28, getSudokube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_29) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(29, getSudokube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_30) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(30, getSudokube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_31) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(31, getSudokube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_32) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(32, getSudokube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_33) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(33, getSudokube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_34) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(34, getSudokube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_35) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(35, getSudokube4(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_36) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(36, getSudokube4(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_37) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(37, getSudokube4(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_38) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(38, getSudokube4(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_39) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(39, getSudokube4(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_40) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(40, getSudokube4(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_42) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(42, getSudokube4(), sudocubeExtractor));
}

}

