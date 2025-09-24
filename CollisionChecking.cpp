///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Thomas Herring,
// Date: FILL ME OUT!!
//////////////////////////////////////
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "CollisionChecking.h"
#include <cmath>

typedef Eigen::Transform<double, 2, Eigen::AffineCompact> SE2;

bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    bool isValid = true;
    for (Rectangle ob : obstacles)
    {
        double max_x = ob.x + ob.width;
        double max_y = ob.y + ob.height;
        if ((x <= max_x) && (x >= ob.x))
        {
            if ((y <= max_y) && (y >= ob.y))
            {
                isValid = false;
            }
        }
    }
    return isValid;
}

bool checkCollision(std::vector<LineSegment> &square, std::vector<LineSegment> &obstacle)
{
    for (LineSegment side : square)
    {
        // against each side of the current obstacle
        for (LineSegment ob_side : obstacle)
        {
            // find intersection point
            Eigen::Vector2d intersection_point = side.getLine().intersection(ob_side.getLine());
            //std::cout << "Intersection Point:\n";
            //std::cout << intersection_point << std::endl;
            // check if intersection point is within both line segments x & y bounds
            if ((intersection_point(0, 0) >= ob_side.getMinX()) && (intersection_point(0, 0) <= ob_side.getMaxX()))
            {
                if ((intersection_point(1, 0) >= ob_side.getMinY()) && (intersection_point(1, 0) <= ob_side.getMaxY()))
                {
                    if ((intersection_point(0, 0) >= side.getMinX()) && (intersection_point(0, 0) <= side.getMaxX()))
                    {
                        if ((intersection_point(1, 0) >= side.getMinY()) && (intersection_point(1, 0) <= side.getMaxY()))
                        {
                            // if intersection point is on both line segments, line segments collide
                            return true;
                        }
                    }
                }
            }
        }
    }
    // if no line segments collide, objects are not in collision
    return false;
}

std::vector<LineSegment> vectorsToList(Eigen::Vector2d v1, Eigen::Vector2d v2, Eigen::Vector2d v3, Eigen::Vector2d v4)
{
    LineSegment s1 = LineSegment(v1, v2);
    LineSegment s2 = LineSegment(v2, v3);
    LineSegment s3 = LineSegment(v3, v4);
    LineSegment s4 = LineSegment(v4, v1);
    // create list of sides for later use
    std::vector<LineSegment> sides;
    sides.push_back(s1);
    sides.push_back(s2);
    sides.push_back(s3);
    sides.push_back(s4);
    return sides;
}

bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles, double hbound, double lbound)
{
    double halfSide = sideLength / 2.0;
    // Assume (x,y) is center of square & refrence frame, find positions of vertices
    Eigen::Vector2d v1(x + sideLength/2 * std::cos(theta) - sideLength/2 * std::sin(theta), y + sideLength/2 * std::sin(theta) + sideLength/2 * std::cos(theta));
    Eigen::Vector2d v2(x - sideLength/2 * std::cos(theta) - sideLength/2 * std::sin(theta), y - sideLength/2 * std::sin(theta) + sideLength/2 * std::cos(theta));
    Eigen::Vector2d v3(x - sideLength/2 * std::cos(theta) + sideLength/2 * std::sin(theta), y - sideLength/2 * std::sin(theta) - sideLength/2 * std::cos(theta));
    Eigen::Vector2d v4(x + sideLength/2 * std::cos(theta) + sideLength/2 * std::sin(theta), y + sideLength/2 * std::sin(theta) - sideLength/2 * std::cos(theta));
    // convert to list of line segments
    std::vector<LineSegment> square = vectorsToList(v1, v2, v3, v4);
    //check if square is out of bounds
    for (LineSegment side : square){
        if(side.getMaxX() > lbound || side.getMaxY() > hbound || side.getMinX() < 0 || side.getMinY() < 0) {return false;}
    }
    // if centerpoint is within an obstacle, don't bother checking side intersections, square is invalid
    if (!(isValidPoint(x, y, obstacles))){return false;}
        // check all obstacles
        for (Rectangle ob : obstacles){
            // define points for a given obstacle
            double max_x = ob.x + ob.width;
            double max_y = ob.y + ob.height;
            Eigen::Vector2d p1(ob.x, ob.y);
            Eigen::Vector2d p2(max_x, ob.y);
            Eigen::Vector2d p3(max_x, max_y);
            Eigen::Vector2d p4(ob.x, max_y);
            std::vector<LineSegment> obstacle = vectorsToList(p1, p2, p3, p4);
            if (checkCollision(square, obstacle))
            {
                std::cout << "Returning False\n";
                return false;
            }}
        return true;
    }
