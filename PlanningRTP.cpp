#include "CollisionChecking.h"
#include "RTP.h"
#include <fstream>
#include <sstream>

void planPoint(const std::vector<Rectangle> & obstacles )
{
    std::cout << obstacles[0].x;
}

void planBox(const std::vector<Rectangle> &  obstacles)
{
    // TODO: Plan for a square_box that rotates and translates in the plane.
}

void makeEnvironment(std::vector<Rectangle>& obstacles, const std::string& filename)
{
    std::ifstream env_file(filename);
    if (!env_file)
        return;

    std::string rect_str;
    while (std::getline(env_file, rect_str))
    {
        std::istringstream iss(rect_str);
        double x, y, width, height;
        if (iss >> x >> y >> width >> height)
            obstacles.push_back(Rectangle(x, y, width, height));
    }
}

void makeEnvironment1(std::vector<Rectangle> &  obstacles )
{
    return makeEnvironment(obstacles, "obstacles1.txt");
}

void makeEnvironment2(std::vector<Rectangle> &  obstacles )
{
    return makeEnvironment(obstacles, "obstacles2.txt");
}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) TODO" << std::endl;
        std::cout << " (2) TODO" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}