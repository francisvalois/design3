#ifndef WORKSPACE_H_
#define WORKSPACE_H_

#include "Pos.h"

class Workspace {

public:
    Workspace();
    virtual ~Workspace();

private:
    Pos antenna;
    Pos robot;
    Pos obstacles[2];
    Pos sudocubes[8];
};

#endif
