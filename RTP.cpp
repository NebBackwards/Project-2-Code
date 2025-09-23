#include "RTP.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include <limits>

ompl::geometric::RTP::RTP(const base::SpaceInformationPtr& si)
    : base::Planner(si, "RTP")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>(
        "goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:0.05:1."
    );
}

void ompl::geometric::RTP::setup()
{
    Planner::setup();
}

ompl::base::PlannerStatus ompl::geometric::RTP::solve(
    const ompl::base::PlannerTerminationCondition &ptc
) 
{
    checkValidity();

    // Set up the state sampler
    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    // Set up initial states
    base::Goal* goal = pdef_->getGoal().get();
    auto* goal_region = dynamic_cast<base::GoalSampleableRegion*>(goal);
    const base::State* start_state = pis_.nextStart();
    if (start_state == nullptr)
        return base::PlannerStatus::INVALID_START;
    base::State* new_state = si_->allocState();

    bool solved = false;
    double best_distance = std::numeric_limits<double>::infinity();

    // Create the root at the start location
    Node* root_node = new Node(si_);
    si_->copyState(root_node->state, start_state);
    node_list_.push_back(root_node);

    while (!ptc)
    {
        // Step 1: Select a random configuration, q_a, from the tree
        Node* new_parent = node_list_[rng_.uniformInt(0,node_list_.size() - 1)];

        // Step 2: Sample a random configuration, q_b, from the configuration space
        if (rng_.uniform01() > goal_bias_)
            sampler_->sampleUniform(new_state);
        else
            goal_region->sampleGoal(new_state);

        // Step 3: Check whether the straight line path is valid, and add to Tree
        if (si_->checkMotion(new_parent->state, new_state))
        {
            // Create and populate the new Node
            Node* new_node = new Node(si_);
            si_->copyState(new_node->state, new_state);
            new_node->parent = new_parent;
            node_list_.push_back(new_node);

            // Check if the solution is found
            double distance = 0.0;
            if (goal->isSatisfied(new_node->state, &distance))
            {
                best_node_ = new_node;
                solved = true;
                break;
            }
            if (distance < best_distance)
            {
                best_distance = distance;
                best_node_ = new_node;
            }
        }
    }

    // If not solved, we only have an approximate solution
    bool approximate = !solved;
    if (best_node_ != nullptr)
    {
        std::vector<Node*> solution;
        Node* traverser = best_node_;
        while (traverser != nullptr)
        {
            solution.push_back(traverser);
            traverser = traverser->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        for (auto* node : solution)
            path->append(node->state);
        pdef_->addSolutionPath(path, approximate, best_distance, getName());
        solved = true;
    }
    // Not even an approximate solution was found
    else
        approximate = false;

    si_->freeState(new_state);

    return { solved, approximate };

}

void ompl::geometric::RTP::getPlannerData(base::PlannerData& data) const 
{
    Planner::getPlannerData(data);

    if (best_node_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(best_node_->state));
    
    for (auto* node : node_list_)
    {
        if (node->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(node->state));
        else
            data.addEdge(
                base::PlannerDataVertex(node->parent->state), 
                base::PlannerDataVertex(node->state)
            );
    }
}

void ompl::geometric::RTP::clear() 
{
    base::Planner::clear();
    sampler_.reset();
    for (auto &node : node_list_)
    {
        if (node->state != nullptr)
            si_->freeState(node->state);
        delete node;
    }
}