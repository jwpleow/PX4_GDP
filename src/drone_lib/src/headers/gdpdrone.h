#ifndef GDPdrone_H
#define GDPdrone_H

#include "data.h"
#include "commands.h"
#include "functions.h"

class GDPdrone
{
public:
    commands Commands = commands(20);
    data Data = {20.0};
    functions Functions;
    GDPdrone();
};

#endif /* GDPdrone_H */
