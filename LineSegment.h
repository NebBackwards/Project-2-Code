#include <Eigen/Dense>

class LineSegment {
public:
    using Vector2d = Eigen::Vector2d;
    using Hyperplane2d = Eigen::Hyperplane<double, 2>;

    // Constructor
    LineSegment(const Vector2d& pointA, const Vector2d& pointB);

    // Getters
    const Vector2d& getStartPoint() const;
    const Vector2d& getEndPoint() const;
    const Hyperplane2d& getLine() const;
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