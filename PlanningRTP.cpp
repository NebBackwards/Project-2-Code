#include "CollisionChecking.h"
#include "RTP.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <fstream>
#include <sstream>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
std::vector<Rectangle> CurrentObstacles;

bool isPointValid(const ob::State *state){
        // cast the abstract state type to the type we expect
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    return(isValidPoint(se2state->getX(),se2state->getY(), CurrentObstacles));

}
bool isSquareValid(const ob::State *state){
        // cast the abstract state type to the type we expect
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    return(isValidSquare(se2state->getX(),se2state->getY(), se2state->getYaw(), 1.0, CurrentObstacles, 5.0, 5.0));

}


void planPoint(const std::vector<Rectangle> & obstacles )
{
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);

    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker([](const ob::State *state){return isPointValid(state);});
    // create a random start state
    ob::ScopedState<> start(space);
    start.random();

    // create a random goal state
    ob::ScopedState<> goal(space);
    goal.random();

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ss.setPlanner(std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation()));
    ss.setup();
    ss.print();

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        //ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}         

void planBox(const std::vector<Rectangle> &  obstacles)
{
    // TODO: Plan for a square_box that rotates and translates in the plane.
    {
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(5);

    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker([](const ob::State *state){return isSquareValid(state);});
    // create a random start state
    ob::ScopedState<> start(space);
    start.random();

    // create a random goal state
    ob::ScopedState<> goal(space);
    goal.random();

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ss.setPlanner(std::make_shared<ompl::geometric::RTP>(ss.getSpaceInformation()));
    ss.setup();
    ss.print();

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        //ss.simplifySolution();
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
} 
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
            CurrentObstacles.clear();
            CurrentObstacles = obstacles;
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
        std::cout << " (1) Environment 1" << std::endl;
        std::cout << " (2) Environment 2" << std::endl;

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