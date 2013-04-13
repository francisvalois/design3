#include <gtest/gtest.h>

#include "vision/SudocubeExtractor.h"

using namespace cv;
using namespace std;

namespace {

class SudocubeExtractorIT: public ::testing::Test {
protected:
    SudocubeExtractor sudocubeExtractor;
};

Sudocube getSudocube1() {
    Sudocube sudokube;
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

Sudocube getSudocube2() {
    Sudocube sudokube;
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

Sudocube getSudocube3() {
    Sudocube sudokube;
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

Sudocube getSudocube4() {
    Sudocube sudokube;
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

bool isSudocubeCorrectlyExtracted(int sudocubeNo, Sudocube correctSudocube, SudocubeExtractor & sudocubeExtractor) {
    Mat img = loadSudocubeNo(sudocubeNo);
    Sudocube * sudokubeExtracted = sudocubeExtractor.extractSudocube(img);
    bool correctlyExtracted = correctSudocube.equals(*sudokubeExtracted);

    return correctlyExtracted;
}

TEST_F(SudocubeExtractorIT, testSudoku1_1) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(1, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_2) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(2, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_3) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(3, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_4) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(4, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_5) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(5, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_6) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(6, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_7) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(7, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_9) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(9, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_10) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(10, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_11) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(11, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_12) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(12, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_13) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(13, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_14) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(14, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_15) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(15, getSudocube1(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_16) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(16, getSudocube2(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_17) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(17, getSudocube2(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_18) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(18, getSudocube2(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_21) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(21, getSudocube2(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_22) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(22, getSudocube2(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_24) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(24, getSudocube2(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_25) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(25, getSudocube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_26) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(26, getSudocube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_28) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(28, getSudocube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_29) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(29, getSudocube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_30) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(30, getSudocube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_31) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(31, getSudocube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_32) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(32, getSudocube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_33) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(33, getSudocube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_34) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(34, getSudocube3(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_35) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(35, getSudocube4(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_36) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(36, getSudocube4(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_37) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(37, getSudocube4(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_38) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(38, getSudocube4(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_39) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(39, getSudocube4(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_40) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(40, getSudocube4(), sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_42) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(42, getSudocube4(), sudocubeExtractor));
}

}

