#include "LineSegment.h"

LineSegment::LineSegment(const Vector2d& pointA, const Vector2d& pointB)
    : startPoint(pointA), endPoint(pointB)
{
    // Direction vector from A to B
    // Initialize the hyperplane using the normal and one of the points
    line = Hyperplane2d::Through(pointA, pointB);
    if (startPoint(0,0) >= endPoint(0,0)){
        maxX = startPoint(0,0);
        minX = endPoint(0,0);
    }else{
        maxX = endPoint(0,0);
        minX = startPoint(0,0);
    }

    if (startPoint(1,0) >= endPoint(1,0)){
        maxX = startPoint(1,0);
        minX = endPoint(1,0);
    }else{
        maxX = endPoint(1,0);
        minX = startPoint(1,0);
    }
}

LineSegment::Vector2d& LineSegment::getStartPoint() {
    return startPoint;
}

LineSegment::Vector2d& LineSegment::getEndPoint() {
    return endPoint;
}

LineSegment::Hyperplane2d& LineSegment::getLine() {
    return line;
}

double LineSegment::getMaxX(){
    return maxX;
}
double LineSegment::getMinX(){
    return minX;
}
double LineSegment::getMaxY(){
    return maxY;
}
double LineSegment::getMinY(){
    return minY;
}
