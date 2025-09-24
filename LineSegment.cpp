#include "LineSegment.h"

LineSegment::LineSegment(const Vector2d& pointA, const Vector2d& pointB)
    : startPoint(pointA), endPoint(pointB)
{
    line = Hyperplane2d::Through(startPoint, endPoint);

    maxX = std::max(startPoint(0),endPoint(0));
    maxY = std::max(startPoint(1),endPoint(1));
    minX = std::min(startPoint(0),endPoint(0));
    minY = std::min(startPoint(1),endPoint(1));
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
