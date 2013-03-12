#include <iostream>
#include "PathPlanning.h"

using namespace std;


int main(int argc, char **argv) {

	PathPlanning pathPlanning;

	position obs1;
	obs1.x = 90;
	obs1.y = 65;
	position obs2;
	obs2.x = 150;
	obs2.y = 95;
	position start;
	start.x = 26;
	start.y = 75;
	position end;
	end.x = 200;
	end.y = 90;
	pathPlanning.setObstacles(obs1,obs2);
	pathPlanning.getPath(start, end);
	pathPlanning.printTable();

    return 0;
}
