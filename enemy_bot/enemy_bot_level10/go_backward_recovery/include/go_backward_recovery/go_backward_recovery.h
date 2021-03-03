#ifndef GO_BACKWARD_RECOVERY_GO_BACKWARD_RECOVERY_H
#define GO_BACKWARD_RECOVERY_GO_BACKWARD_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace go_backward_recovery{

    class GoBackwardRecovery : public nav_core::RecoveryBehavior{
        public:
            GoBackwardRecovery();
            ~GoBackwardRecovery();
            void initialize(std::string name, tf2_ros::Buffer*,
                            costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);
            void runBehavior();

        private:
            costmap_2d::Costmap2DROS* local_costmap_;
            bool initialized_;
            base_local_planner::CostmapModel* world_model_;
    };
};  // namespace go_backward_recovery

#endif  // GO_BACKWARD_RECOVERY_GO_BACKWARD_RECOVERY_H