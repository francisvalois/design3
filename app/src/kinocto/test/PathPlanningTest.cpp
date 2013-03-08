#include <gtest/gtest.h>

#include "PathPlanning.h"

namespace {
  class PathPlanningTest : public ::testing::Test {
  protected:
    PathPlanning pathPlanning;
  };


  TEST_F(PathPlanningTest, print) {

	  pathPlanning.setObstacles(90,65,150,95);
	  pathPlanning.printTable();
  }
}
