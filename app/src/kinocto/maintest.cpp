#include <iostream>
#include "PathPlanning2.h"

using namespace std;


int main(int argc, char **argv) {

	PathPlanning2 pathPlanning;

	Position obs1;
	obs1.x = 90;
	obs1.y = 65;
	Position obs2;
	obs2.x = 150;
	obs2.y = 45;
	Position start;
	start.x = 26;
	start.y = 75;
	Position end;
	end.x = 200;
	end.y = 90;
	pathPlanning.setObstacles(obs1,obs2);
	pathPlanning.getPath(start, end);
	pathPlanning.printTable();

    return 0;
}
