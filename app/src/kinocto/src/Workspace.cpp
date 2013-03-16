#include "Workspace.h"

Workspace::Workspace() {
    robot.x = 57;
    robot.y = 13;
    robot.angle = 0;

    obstacles[0].x = 176;
    obstacles[0].y = 78;
    obstacles[0].angle = 0;

    obstacles[1].x = 209;
    obstacles[1].y = 78;
    obstacles[1].angle = 0;

    obstacles[2].x = 197;
    obstacles[2].y = 96;
    obstacles[2].angle = 90;

    obstacles[3].x = 197;
    obstacles[3].y = 70;
    obstacles[3].angle = 90;

    obstacles[4].x = 197;
    obstacles[4].y =44;
    obstacles[4].angle = 90;

    obstacles[5].x = 197;
    obstacles[5].y = 18;
    obstacles[5].angle = 90;

    obstacles[6].x = 209;
    obstacles[6].y = 36;
    obstacles[6].angle = 180;

    obstacles[7].x = 176;
    obstacles[7].y = 36;
    obstacles[7].angle = 180;

}

Workspace::~Workspace() {
}

