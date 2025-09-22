#ifndef LINE_SEGMENT_H_
#define LINE_SEGMENT_H_
#include <Eigen/Dense>
#include <Eigen/Geometry>

class LineSegment {
public:
    using Vector2d = Eigen::Vector2d;
    using Hyperplane2d = Eigen::Hyperplane<double, 2>;

    // Constructor
    LineSegment(const Vector2d& pointA, const Vector2d& pointB);

    // Getters
    Vector2d& getStartPoint();
    Vector2d& getEndPoint();
    Hyperplane2d& getLine();
    double getMaxX();
    double getMaxY();
    double getMinX();
    double getMinY();


private:
    Vector2d startPoint;
    Vector2d endPoint;
    Hyperplane2d line;
    double maxX;
    double minX;
    double maxY;
    double minY;
};

#endif