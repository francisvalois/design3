#include <gtest/gtest.h>

#include "vision/SudocubeExtractor.h"

using namespace cv;
using namespace std;

namespace {

class SudocubeExtractorIT: public ::testing::Test {
protected:
    static SudocubeExtractor * sudocubeExtractor;

    static void SetUpTestCase() {
        sudocubeExtractor = new SudocubeExtractor();
    }

    static void TearDownTestCase() {
        delete sudocubeExtractor;
    }

};

SudocubeExtractor * SudocubeExtractorIT::sudocubeExtractor = NULL;

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

Sudocube getSudocube5() {
    Sudocube sudokube;
    sudokube.setCaseValue(1, 1, 1, 1);
    sudokube.setCaseValue(1, 1, 4, 2);
    sudokube.setCaseValue(1, 4, 2, 6);

    sudokube.setCaseValue(2, 1, 2, 6);
    sudokube.setCaseValue(2, 1, 4, 2);
    sudokube.setCaseValue(2, 2, 2, 3);
    sudokube.setCaseValue(2, 4, 1, 4);
    sudokube.setCaseValue(2, 4, 3, 5);

    sudokube.setCaseValue(3, 2, 2, 7);
    sudokube.setCaseValue(3, 2, 3, 8);
    sudokube.setCaseValue(3, 4, 2, 5);

    sudokube.setRedCase(3, 4, 4);

    return sudokube;
}

Sudocube getSudocube6() {
    Sudocube sudokube;
    sudokube.setCaseValue(1, 1, 1, 6);
    sudokube.setCaseValue(1, 2, 2, 3);
    sudokube.setCaseValue(1, 3, 3, 8);
    sudokube.setCaseValue(1, 4, 4, 7);

    sudokube.setCaseValue(2, 1, 1, 7);
    sudokube.setCaseValue(2, 2, 2, 2);
    sudokube.setCaseValue(2, 3, 3, 5);
    sudokube.setCaseValue(2, 4, 4, 4);

    sudokube.setCaseValue(3, 1, 1, 4);
    sudokube.setCaseValue(3, 2, 2, 5);
    sudokube.setCaseValue(3, 3, 3, 4);
    sudokube.setCaseValue(3, 4, 4, 8);

    sudokube.setRedCase(2, 3, 2);

    return sudokube;
}

Sudocube getSudocube7() {
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

    sudokube.setRedCase(1, 3, 3);

    return sudokube;
}

Mat loadSudocubeNo(int no) {
    char filename[255];
    //sprintf(filename, "%s/%d.jpeg", "img/testSudocubesJpg", no);
    sprintf(filename, "%s/%d.png", "img/testSudocubes", no);

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

    //cout << sudokubeExtracted->print() << endl;
    //int * redCase = sudokubeExtracted->getRedCase();
    //cout << "Red case:"<<  redCase[0] << "." << redCase[1] << "." << redCase[2] << endl;

    delete sudokubeExtracted;

    return correctlyExtracted;
}

TEST_F(SudocubeExtractorIT, testSudoku1_1) {
    /*cout << getSudocube3().print() << endl;
    int * redCase = getSudocube3().getRedCase();
    cout << "Red case:"<<  redCase[0] << "." << redCase[1] << "." << redCase[2] << endl;*/
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(1, getSudocube3(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_2) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(2, getSudocube3(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_3) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(3, getSudocube3(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_4) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(4, getSudocube3(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_5) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(5, getSudocube3(), *sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_6) {
    /*cout << getSudocube4().print() << endl;
    int * redCase = getSudocube4().getRedCase();
    cout << "Red case:"<<  redCase[0] << "." << redCase[1] << "." << redCase[2] << endl;*/
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(6, getSudocube4(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_7) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(7, getSudocube4(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_8) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(8, getSudocube4(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_9) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(9, getSudocube4(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_10) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(10, getSudocube4(), *sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_11) {
    /*cout << getSudocube5().print() << endl;
    int * redCase = getSudocube5().getRedCase();
    cout << "Red case:"<<  redCase[0] << "." << redCase[1] << "." << redCase[2] << endl;*/
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(11, getSudocube5(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_12) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(12, getSudocube5(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_13) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(13, getSudocube5(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_14) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(14, getSudocube5(), *sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_15) {
    /*cout << getSudocube2().print() << endl;
    int * redCase = getSudocube2().getRedCase();
    cout << "Red case:"<<  redCase[0] << "." << redCase[1] << "." << redCase[2] << endl;*/
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(15, getSudocube2(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_16) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(16, getSudocube2(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_17) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(17, getSudocube2(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_18) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(18, getSudocube2(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_19) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(19, getSudocube2(), *sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_20) {
    /*cout << getSudocube7().print() << endl;
    int * redCase = getSudocube7().getRedCase();
    cout << "Red case:"<<  redCase[0] << "." << redCase[1] << "." << redCase[2] << endl;*/
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(20, getSudocube7(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_21) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(21, getSudocube7(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_22) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(22, getSudocube7(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_23) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(23, getSudocube7(), *sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_24) {
    /*cout << getSudocube4().print() << endl;
    int * redCase = getSudocube4().getRedCase();
    cout << "Red case:"<<  redCase[0] << "." << redCase[1] << "." << redCase[2] << endl;*/
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(24, getSudocube4(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_25) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(25, getSudocube4(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_26) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(26, getSudocube4(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_27) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(27, getSudocube4(), *sudocubeExtractor));
}

TEST_F(SudocubeExtractorIT, testSudoku1_28) {
    /*cout << getSudocube6().print() << endl;
    int * redCase = getSudocube6().getRedCase();
    cout << "Red case:"<<  redCase[0] << "." << redCase[1] << "." << redCase[2] << endl;*/
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(28, getSudocube6(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_29) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(29, getSudocube6(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_30) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(30, getSudocube6(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_31) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(31, getSudocube6(), *sudocubeExtractor));
}
TEST_F(SudocubeExtractorIT, testSudoku1_32) {
    ASSERT_TRUE(isSudocubeCorrectlyExtracted(32, getSudocube6(), *sudocubeExtractor));
}

/*TEST_F(SudocubeExtractorIT, testSortSudocube) {
 vector<Sudocube *> sudocubes;
 vector<Sudocube *> pool;
 vector<int> poolNumber;
 for (int i = 1; i <= 32; i++) {
 Mat img = loadSudocubeNo(i);
 Sudocube * sudocube = sudocubeExtractor->extractSudocube(img);
 sudocubes.push_back(sudocube);
 cout << sudocube->print();
 }

 for (int i = 0; i < sudocubes.size(); i++) {
 bool isAlreadyIn = false;
 for (int j = 0; j < pool.size(); j++) {
 if (sudocubes[i]->equals(*pool[j])) {
 isAlreadyIn = true;
 poolNumber[j] = poolNumber[j] + 1;
 }
 }

 if (isAlreadyIn == false) {
 pool.push_back(sudocubes[i]);
 poolNumber.push_back(1);
 }
 }

 int biggest = 0;
 for (int i = 0; i < poolNumber.size(); i++) {
 if (poolNumber[i] > poolNumber[biggest]) {
 biggest = i;
 }
 }

 cout << "Ze best sudocube" << endl << pool[biggest]->print();

 Sudocube correctSudocube = getSudocube1();
 ASSERT_TRUE(pool[biggest]->equals(correctSudocube));

 pool.clear();
 for (int i = 0; i < sudocubes.size(); i++) {
 Sudocube * sudocube = sudocubes[i];
 sudocubes[i] = NULL;
 delete sudocube;
 }
 }*/

}

