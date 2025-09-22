#ifndef RANDOM_TREE_H
#define RANDOM_TREE_H

#include "ompl/geometric/planners/PlannerIncludes.h"
#include <vector>

namespace ompl
{
    namespace geometric
    {
        class RTP : public base::Planner
        {
        public:
            RTP(const base::SpaceInformationPtr& si);
            void setup();
            base::PlannerStatus solve(const base::PlannerTerminationCondition& ptc) override;
            void getPlannerData(base::PlannerData& data) const override;
            void clear() override;

            void setGoalBias(const double goal_bias) { goal_bias_ = goal_bias; }
            double getGoalBias() const { return goal_bias_;  }

        protected:
            class Node
            {
                Node() = default;
                Node(const base::StateInformationPtr& si) 
                    : state(si->allocState())
                {}

                ~Node() = default;

                base::State* state{nullptr};
                Node* parent{nullptr};
            };

            base::StateSamplerPtr sampler_;
            std::vector<Node*> node_list_;
            RNG rng_;
            double goal_bias_{0.05};
            Node* best_node_{nullptr};
        };

    }  // namespace geometric
}  // namespace ompl

#endif // RANDOM_TREE_H