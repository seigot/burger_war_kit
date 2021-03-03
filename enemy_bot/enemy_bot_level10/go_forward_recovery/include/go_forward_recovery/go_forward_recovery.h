#ifndef GO_FORWARD_RECOVERY_GO_FORWARD_RECOVERY_H
#define GO_FORWARD_RECOVERY_GO_FORWARD_RECOVERY_H

#include <nav_core/recovery_behavior.h>
#include <base_local_planner/costmap_model.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace go_forward_recovery{

    class GoForwardRecovery : public nav_core::RecoveryBehavior{
        public:
            GoForwardRecovery();
            ~GoForwardRecovery();
            void initialize(std::string name, tf2_ros::Buffer*,
                            costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);
            void runBehavior();

        private:
            costmap_2d::Costmap2DROS* local_costmap_;
            bool initialized_;
            base_local_planner::CostmapModel* world_model_;
    };
};  // namespace go_forward_recovery

#endif  // GO_FORWARD_RECOVERY_GO_FORWARD_RECOVERY_H
