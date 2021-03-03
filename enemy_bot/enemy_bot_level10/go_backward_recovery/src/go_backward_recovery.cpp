#include <go_backward_recovery/go_backward_recovery.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(go_backward_recovery::GoBackwardRecovery, nav_core::RecoveryBehavior)

namespace go_backward_recovery
{
    GoBackwardRecovery::GoBackwardRecovery(): local_costmap_(NULL), initialized_(false), world_model_(NULL){
    }

    GoBackwardRecovery::~GoBackwardRecovery(){
    }

    void GoBackwardRecovery::initialize(std::string name, tf2_ros::Buffer*,
                                        costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap){
        if(!initialized_){
            initialized_ = true;
        }else{
            ROS_ERROR("You should not call initialize twice on this object, doing nothing");
        }
    }

    void GoBackwardRecovery::runBehavior(){
        if(!initialized_){
            ROS_ERROR("This object must be initialized before runBehavior is called");
            return;
        }

        ros::Rate r(10);
        ros::NodeHandle nh;
        ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        geometry_msgs::Twist cmd_vel;
        int loopCounter = 0;
        int stop_count_thrshld = 10;
        int return_count_thrshld = 15;

        cmd_vel.linear.x = -0.22;//-0.1;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;

        ROS_WARN("Go backward recovery behavior started.");

        while(ros::ok()){
            if(loopCounter == stop_count_thrshld){
                cmd_vel.linear.x = 0.0;
            }else if(loopCounter >= return_count_thrshld){
                break;
            }
            vel_pub.publish(cmd_vel);
            loopCounter++;
            r.sleep();
        }

        return;
    }
};  // namespace go_backward_recovery
