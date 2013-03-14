#include <iostream>
#include "PathPlanning2.h"
#include <time.h>

using namespace std;

PathPlanning2 pathPlanning;
Position obs1, obs2, start, destination;

Position SUDOKUBE1;
Position SUDOKUBE2;
Position SUDOKUBE3;
Position SUDOKUBE4;
Position SUDOKUBE5;
Position SUDOKUBE6;
Position SUDOKUBE7;
Position SUDOKUBE8;



int main(int argc, char **argv) {

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

  	obs1.x = 90;
	obs1.y = 85;
	obs2.x = 120;
	obs2.y = 25;
	start.x = 36;
	start.y = 85;
	pathPlanning.setObstacles(obs1,obs2);

	pathPlanning.getPath(start, SUDOKUBE1);
	pathPlanning.printTable();

	pathPlanning.getPath(start, SUDOKUBE2);
	pathPlanning.printTable();

	pathPlanning.getPath(start, SUDOKUBE3);
	pathPlanning.printTable();

	pathPlanning.getPath(start, SUDOKUBE4);
	pathPlanning.printTable();

	pathPlanning.getPath(start, SUDOKUBE5);
	pathPlanning.printTable();

	pathPlanning.getPath(start, SUDOKUBE6);
	pathPlanning.printTable();

	pathPlanning.getPath(start, SUDOKUBE7);
	pathPlanning.printTable();

	pathPlanning.getPath(start, SUDOKUBE8);
	pathPlanning.printTable();

    return 0;
}
