//**************************************************************************************
//FOR NOW, ALL THESE TESTS REQUIRE VISUAL VERIFICATION TO BE PASSED
//TO DO SO. UNCOMMENT LINES WITH pathPlanning.printTable(); AND VERIFY MANUALLY
//**************************************************************************************

#include <gtest/gtest.h>

#include "pathPlanning/PathPlanning.h"
#include "Workspace.h"

using namespace std;

namespace {


  class PathPlanningTest : public ::testing::Test {
  protected:
    PathPlanning pathPlanning;
    Position obs1;
    Position obs2;
    Position start;
    Workspace workspace;

	float STARTING_ANGLE;

    virtual void SetUp() {
    	start.set(36,85);

		STARTING_ANGLE = 0.0f;
    }
  };

  TEST_F(PathPlanningTest, IntegrationTest1) {
	  	//CONFIGURATION POSSIBILITY 1
//	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//	  xx0000000000xxxxxx0000000000000000000xx
//	  xx0000000000xxxxxx0000000000000000000xx
//	  xx0000000000xxxxxx0000000000000000000xx
//	  xx00000000000000000000000000000000000xx
//	  xx00000000000000000000000000000000000xx
//	  xx000000000000000xxxxxx00000000000000xx
//	  xx000000000000000xxxxxx00000000000000xx
//	  xx000000000000000xxxxxx00000000000000xx
//	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

		obs1.set(90,85);
		obs2.set(120,25);

		pathPlanning.setObstacles(obs1,obs2);

		for(int i = 1; i <=8; i++) {
			pathPlanning.getPath(start, workspace.getSudocubePos(i));
//			pathPlanning.printTable();
		}
  }

  TEST_F(PathPlanningTest, IntegrationTest2) {
	  	//CONFIGURATION POSSIBILITY 2
//	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//	  xx0000000000xxxxxxxxxxx00000000000000xx
//	  xx0000000000xxxxxxxxxxx00000000000000xx
//	  xx0000000000xxxxxxxxxxx00000000000000xx
//	  xx00000000000000000000000000000000000xx
//	  xx00000000000000000000000000000000000xx
//	  xx00000000000000000000000000000000000xx
//	  xx00000000000000000000000000000000000xx
//	  xx00000000000000000000000000000000000xx
//	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    	obs1.set(90,85);
    	obs2.set(110,85);

		pathPlanning.setObstacles(obs1,obs2);

		for(int i = 1; i <=8; i++) {
			pathPlanning.getPath(start, workspace.getSudocubePos(i));
//			pathPlanning.printTable();
		}
  }

  TEST_F(PathPlanningTest, IntegrationTest3) {
	  	//CONFIGURATION POSSIBILITY 3
//	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//	  xx00000000000000000000000000000000000xx
//	  xx00000000000000000000000000000000000xx
//	  xx0000000000xxxxxxxxxxx00000000000000xx
//	  xx0000000000xxxxxxxxxxx00000000000000xx
//	  xx0000000000xxxxxxxxxxx00000000000000xx
//	  xx00000000000000000000000000000000000xx
//	  xx00000000000000000000000000000000000xx
//	  xx00000000000000000000000000000000000xx
//	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

	    obs1.set(90,65);
    	obs2.set(110,65);

		pathPlanning.setObstacles(obs1,obs2);

		for(int i = 1; i <=8; i++) {
			pathPlanning.getPath(start, workspace.getSudocubePos(i));
//			pathPlanning.printTable();
		}
  }

  TEST_F(PathPlanningTest, IntegrationTest4) {
//	  	CONFIGURATION POSSIBILITY 4
//	  	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//	  	  xx00000000000000000000000000000000000xx
//	  	  xx00000000000000000000000000000000000xx
//	  	  xx0000000000xxxxx00000xxxxx0000000000xx
//	  	  xx0000000000xxxxx00000xxxxx0000000000xx
//	  	  xx0000000000xxxxx00000xxxxx0000000000xx
//	  	  xx00000000000000000000000000000000000xx
//	  	  xx00000000000000000000000000000000000xx
//	  	  xx00000000000000000000000000000000000xx
//	  	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    	obs1.set(90,55);
    	obs2.set(150,55);

		pathPlanning.setObstacles(obs1,obs2);

		for(int i = 1; i <=8; i++) {
			pathPlanning.getPath(start, workspace.getSudocubePos(i));
//			pathPlanning.printTable();
		}
  }

  TEST_F(PathPlanningTest, IntegrationTest5) {
	  	//CONFIGURATION POSSIBILITY 5
//	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
//	  xx00000000000000000000000000000000000xx
//	  xx0000000000xxxxxx0000000000000000000xx
//	  xx0000000000xxxxxx0000000000000000000xx
//	  xx0000000000xxxxxx0000000000000000000xx
//	  xx000000000000000000xxxxxx00000000000xx
//	  xx000000000000000000xxxxxx00000000000xx
//	  xx000000000000000000xxxxxx00000000000xx
//	  xx00000000000000000000000000000000000xx
//	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    	obs1.set(90,65);
    	obs2.set(150,45);

		pathPlanning.setObstacles(obs1,obs2);

		for(int i = 1; i <=8; i++) {
			pathPlanning.getPath(start, workspace.getSudocubePos(i));
//			pathPlanning.printTable();
		}
  }

  TEST_F(PathPlanningTest, IntegrationTest6) {
  	  	//CONFIGURATION POSSIBILITY 5
  //	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //	  xx00000000000000000000000000000000000xx
  //	  xx0000000000xxxxxx0000000000000000000xx
  //	  xx0000000000xxxxxx0000000000000000000xx
  //	  xx0000000000xxxxxx0000000000000000000xx
  //	  xx000000000000000000xxxxxx00000000000xx
  //	  xx000000000000000000xxxxxx00000000000xx
  //	  xx000000000000000000xxxxxx00000000000xx
  //	  xx00000000000000000000000000000000000xx
  //	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    	obs1.set(90,65);
    	obs2.set(150,45);
    	start.set(36,55);

  		pathPlanning.setObstacles(obs1,obs2);

		for(int i = 1; i <=8; i++) {
			pathPlanning.getPath(start, workspace.getSudocubePos(i));
//			pathPlanning.printTable();
		}
    }

  TEST_F(PathPlanningTest, IntegrationTest7) {
  	  	//CONFIGURATION POSSIBILITY 5
  //	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //	  xx00000000000000000000000000000000000xx
  //	  xx0000000000xxxxxx0000000000000000000xx
  //	  xx0000000000xxxxxx0000000000000000000xx
  //	  xx0000000000xxxxxx0000000000000000000xx
  //	  xx000000000000000000xxxxxx00000000000xx
  //	  xx000000000000000000xxxxxx00000000000xx
  //	  xx000000000000000000xxxxxx00000000000xx
  //	  xx00000000000000000000000000000000000xx
  //	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

		obs1.set(90,65);
		obs2.set(150,45);
		start.set(40,25);

  		pathPlanning.setObstacles(obs1,obs2);

		for(int i = 1; i <=8; i++) {
			pathPlanning.getPath(start, workspace.getSudocubePos(i));
//			pathPlanning.printTable();
		}
    }

  TEST_F(PathPlanningTest, IntegrationTest8) {
  	  	//CONFIGURATION POSSIBILITY 5
  //	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
  //	  xx00000000000000000000000000000000000xx
  //	  xx0000000000xxxxxx00xxxxxx00000000000xx
  //	  xx0000000000xxxxxx00xxxxxx00000000000xx
  //	  xx0000000000xxxxxx00xxxxxx00000000000xx
  //	  xx00000000000000000000000000000000000xx
  //	  xx00000000000000000000000000000000000xx
  //	  xx00000000000000000000000000000000000xx
  //	  xx00000000000000000000000000000000000xx
  //	  xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

		obs1.set(130,35);
		obs2.set(150,35);
		start.set(60,15);

  		pathPlanning.setObstacles(obs1,obs2);

		for(int i = 1; i <=8; i++) {
			pathPlanning.getPath(start, workspace.getSudocubePos(i));
//			pathPlanning.printTable();
		}
//		vector<Position> pos = pathPlanning.getPath(start, workspace.getSudocubePos(7));
//		cout << pos.size() << endl;
//		for(unsigned int i = 0; i <pos.size(); i++) {
//			cout << pos[i].x << "," << pos[i].y << endl;
//		}
//			pathPlanning.printTable();
    }

  TEST_F(PathPlanningTest, IntegrationTestAntenna) {
	  	obs1.set(110,75);
		obs2.set(180,30);
		start.set(19,57);

		pathPlanning.setObstacles(obs1,obs2);

		pathPlanning.getPath(start, workspace.getSquareCenter());
//		pathPlanning.printTable();
  }

  TEST_F(PathPlanningTest, SegFaultTest) {
		obs1.set(90,65);
		obs2.set(150,45);

		pathPlanning.setObstacles(obs1,obs2);

		obs1.set(91,66);
		obs2.set(151,46);

		pathPlanning.setObstacles(obs1,obs2);
		//TO ASSERT NO SEG FAULT
  }

  TEST_F(PathPlanningTest, doesNotReturnAPathWhenDestinationIsWithinObstacle) {
	  	obs1.set(110,75);
		obs2.set(176,66);
		start.set(19,57);

		pathPlanning.setObstacles(obs1,obs2);

		ASSERT_EQ(0, pathPlanning.getPath(start, workspace.getSudocubePos(1)).size());
//		pathPlanning.printTable();
  }
}
