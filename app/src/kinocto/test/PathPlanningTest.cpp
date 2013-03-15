//**************************************************************************************
//FOR NOW, ALL THESE TESTS REQUIRE VISUAL VERIFICATION TO BE PASSED
//TO DO SO. UNCOMMENT LINES WITH pathPlanning.printTable(); AND VERIFY MANUALLY
//**************************************************************************************

#include <gtest/gtest.h>

#include "PathPlanning.h"

namespace {


  class PathPlanningTest : public ::testing::Test {
  protected:
    PathPlanning pathPlanning;
    Position obs1, obs2, start;

	Position SUDOKUBE1;
	Position SUDOKUBE2;
	Position SUDOKUBE3;
	Position SUDOKUBE4;
	Position SUDOKUBE5;
	Position SUDOKUBE6;
	Position SUDOKUBE7;
	Position SUDOKUBE8;
	float SUDOKUBE1_ANGLE;
	float SUDOKUBE2_ANGLE;
	float SUDOKUBE3_ANGLE;
	float SUDOKUBE4_ANGLE;
	float SUDOKUBE5_ANGLE;
	float SUDOKUBE6_ANGLE;
	float SUDOKUBE7_ANGLE;
	float SUDOKUBE8_ANGLE;

	float STARTING_ANGLE;

    virtual void SetUp() {
    	SUDOKUBE1.x = 176;
    	SUDOKUBE1.y = 78;
    	SUDOKUBE2.x = 209;
    	SUDOKUBE2.y = 78;
    	SUDOKUBE3.x = 197;
    	SUDOKUBE3.y = 96;
    	SUDOKUBE4.x = 197;
    	SUDOKUBE4.y = 70;
    	SUDOKUBE5.x = 197;
    	SUDOKUBE5.y = 44;
    	SUDOKUBE6.x = 197;
    	SUDOKUBE6.y = 18;
    	SUDOKUBE7.x = 209;
    	SUDOKUBE7.y = 36;
    	SUDOKUBE8.x = 176;
    	SUDOKUBE8.y = 36;

		start.x = 36;
		start.y = 85;

		SUDOKUBE1_ANGLE = -90.0f;
		SUDOKUBE2_ANGLE = -90.0f;
		SUDOKUBE3_ANGLE = 0.0f;
		SUDOKUBE4_ANGLE = 0.0f;
		SUDOKUBE5_ANGLE = 0.0f;
		SUDOKUBE6_ANGLE = 0.0f;
		SUDOKUBE7_ANGLE = 90.0f;
		SUDOKUBE8_ANGLE = 90.0f;

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

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE1, SUDOKUBE1_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE2, SUDOKUBE2_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE3, SUDOKUBE3_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE4, SUDOKUBE4_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE5, SUDOKUBE5_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE6, SUDOKUBE6_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE7, SUDOKUBE7_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE8, SUDOKUBE8_ANGLE);
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

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE1, SUDOKUBE1_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE2, SUDOKUBE2_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE3, SUDOKUBE3_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE4, SUDOKUBE4_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE5, SUDOKUBE5_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE6, SUDOKUBE6_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE7, SUDOKUBE7_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE8, SUDOKUBE8_ANGLE);
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

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE1, SUDOKUBE1_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE2, SUDOKUBE2_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE3, SUDOKUBE3_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE4, SUDOKUBE4_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE5, SUDOKUBE5_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE6, SUDOKUBE6_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE7, SUDOKUBE7_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE8, SUDOKUBE8_ANGLE);
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

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE1, SUDOKUBE1_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE2, SUDOKUBE2_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE3, SUDOKUBE3_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE4, SUDOKUBE4_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE5, SUDOKUBE5_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE6, SUDOKUBE6_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE7, SUDOKUBE7_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE8, SUDOKUBE8_ANGLE);
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

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE1, SUDOKUBE1_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE2, SUDOKUBE2_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE3, SUDOKUBE3_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE4, SUDOKUBE4_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE5, SUDOKUBE5_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE6, SUDOKUBE6_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE7, SUDOKUBE7_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE8, SUDOKUBE8_ANGLE);
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

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE1, SUDOKUBE1_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE2, SUDOKUBE2_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE3, SUDOKUBE3_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE4, SUDOKUBE4_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE5, SUDOKUBE5_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE6, SUDOKUBE6_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE7, SUDOKUBE7_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE8, SUDOKUBE8_ANGLE);
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

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE1, SUDOKUBE1_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE2, SUDOKUBE2_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE3, SUDOKUBE3_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE4, SUDOKUBE4_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE5, SUDOKUBE5_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE6, SUDOKUBE6_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE7, SUDOKUBE7_ANGLE);
//		pathPlanning.printTable();

		pathPlanning.getPath(start, STARTING_ANGLE, SUDOKUBE8, SUDOKUBE8_ANGLE);
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

		pathPlanning.getPath(SUDOKUBE1, SUDOKUBE1_ANGLE, start, STARTING_ANGLE);

		pathPlanning.printTable();
  }

  TEST_F(PathPlanningTest, IntegrationTest9) {
	  	obs1.x = 90;
		obs1.y = 65;
		obs2.x = 150;
		obs2.y = 45;

		start.x = 40;
		start.y = 25;

		pathPlanning.setObstacles(obs1,obs2);

		pathPlanning.getPath(start, 19.0f, SUDOKUBE1, SUDOKUBE1_ANGLE);

		pathPlanning.printTable();
  }
}
