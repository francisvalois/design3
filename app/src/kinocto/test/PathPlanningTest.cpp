//**************************************************************************************
//FOR NOW, ALL THESE TESTS REQUIRE VISUAL VERIFICATION TO BE PASSED
//TO DO SO. UNCOMMENT LINES WITH pathPlanning.printTable(); AND VERIFY MANUALLY
//**************************************************************************************

#include <gtest/gtest.h>

#include "PathPlanning.h"
#include "Workspace.h"

namespace {


  class PathPlanningTest : public ::testing::Test {
  protected:
    PathPlanning pathPlanning;
    Position obs1, obs2, start;
    Workspace workspace;

	float STARTING_ANGLE;

    virtual void SetUp() {
		start.x = 36;
		start.y = 85;

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

	  	obs1.x = 90;
		obs1.y = 85;
		obs2.x = 120;
		obs2.y = 25;

		pathPlanning.setObstacles(obs1,obs2);

		pathPlanning.getPath(start, workspace.getSudocubePos(1));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(2));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(3));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(4));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(5));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(6));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(7));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(8));
//		pathPlanning.printTable();
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

	  	obs1.x = 90;
		obs1.y = 85;
		obs2.x = 110;
		obs2.y = 85;

		pathPlanning.setObstacles(obs1,obs2);

		pathPlanning.getPath(start, workspace.getSudocubePos(1));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(2));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(3));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(4));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(5));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(6));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(7));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(8));
//		pathPlanning.printTable();
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

	  	obs1.x = 90;
		obs1.y = 65;
		obs2.x = 110;
		obs2.y = 65;

		pathPlanning.setObstacles(obs1,obs2);

		pathPlanning.getPath(start, workspace.getSudocubePos(1));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(2));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(3));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(4));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(5));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(6));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(7));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(8));
//		pathPlanning.printTable();
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

	  	obs1.x = 90;
		obs1.y = 55;
		obs2.x = 150;
		obs2.y = 55;

		pathPlanning.setObstacles(obs1,obs2);

		pathPlanning.getPath(start, workspace.getSudocubePos(1));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(2));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(3));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(4));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(5));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(6));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(7));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(8));
//		pathPlanning.printTable();
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

	  	obs1.x = 90;
		obs1.y = 65;
		obs2.x = 150;
		obs2.y = 45;

		pathPlanning.setObstacles(obs1,obs2);

		pathPlanning.getPath(start, workspace.getSudocubePos(1));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(2));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(3));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(4));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(5));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(6));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(7));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(8));
//		pathPlanning.printTable();
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

  	  	obs1.x = 90;
  		obs1.y = 65;
  		obs2.x = 150;
  		obs2.y = 45;

		start.x = 36;
		start.y = 55;

  		pathPlanning.setObstacles(obs1,obs2);

		pathPlanning.getPath(start, workspace.getSudocubePos(1));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(2));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(3));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(4));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(5));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(6));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(7));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(8));
//		pathPlanning.printTable();
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

  	  	obs1.x = 90;
  		obs1.y = 65;
  		obs2.x = 150;
  		obs2.y = 45;

		start.x = 40;
		start.y = 25;

  		pathPlanning.setObstacles(obs1,obs2);

		pathPlanning.getPath(start, workspace.getSudocubePos(1));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(2));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(3));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(4));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(5));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(6));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(7));
//		pathPlanning.printTable();

		pathPlanning.getPath(start, workspace.getSudocubePos(8));
//		pathPlanning.printTable();
    }

  TEST_F(PathPlanningTest, IntegrationTest8) {
	  	obs1.x = 90;
		obs1.y = 65;
		obs2.x = 150;
		obs2.y = 45;

		start.x = 40;
		start.y = 25;

		pathPlanning.setObstacles(obs1,obs2);

		pathPlanning.getPath(start, workspace.getSudocubePos(1));
//		pathPlanning.printTable();
}

  TEST_F(PathPlanningTest, IntegrationTest9) {
	  	obs1.x = 90;
		obs1.y = 65;
		obs2.x = 150;
		obs2.y = 45;

		start.x = 40;
		start.y = 25;

		pathPlanning.setObstacles(obs1,obs2);

		pathPlanning.getPath(start, workspace.getSudocubePos(1));
//		pathPlanning.printTable();
  }

  TEST_F(PathPlanningTest, IntegrationTest9) {
	  	obs1.x = 90;
		obs1.y = 65;
		obs2.x = 150;
		obs2.y = 45;

		pathPlanning.setObstacles(obs1,obs2);

	  	obs1.x = 91;
		obs1.y = 66;
		obs2.x = 151;
		obs2.y = 46;

		pathPlanning.setObstacles(obs1,obs2);

		EXPECT_NO_THROW(pathPlanning.setObstacles(obs1,obs2));
  }

}
