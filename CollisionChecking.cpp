///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Thomas Herring,
// Date: FILL ME OUT!!
//////////////////////////////////////

#include "CollisionChecking.h"

bool isValidPoint(double x, double y, const std::vector<Rectangle>& obstacles)
{
    bool isValid = true;
    for (Rectangle ob : obstacles){
        double max_x = ob.x + ob.width;
        double max_y = ob.y + ob.height;
        if ((x <= max_x) && (x >= ob.x)){
            if ((y <= max_y) && (y >= ob.y)){
                isValid = false;
            }
        }
    }
    return isValid;
}

bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle>& obstacles,double hbound, double lbound)
{
    //Fillout 
}

