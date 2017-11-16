#ifndef GRIPPER_ACTIONS_H_
#define GRIPPER_ACTIONS_H_

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>
#include <robotiq_85_msgs/GripperCmd.h>
#include <robotiq_85_msgs/GripperStat.h>

class GripperActions
{
public:
    /**
    * \brief Constructor
    */
    GripperActions();

private:
    /**
    * \brief Callback for the gripperCommand action sever, support for standard ROS gripper control messages
    * @param goal action goal
    */
    void executeGripperCommand(const control_msgs::GripperCommandGoalConstPtr &goal);

    /**
    * \brief Callback for joint state updates
    * @param msg joint state message
    */
    void gripperStatusCallback(const robotiq_85_msgs::GripperStat msg);

    //constants
    static const float defaultGripperSpeed = .013;
    static const float defaultGripperForce = 100;
    static const float gripperClosedPosition = 0;
    static const float gripperOpenPosition = 0.085;
    static const float gripperMinSpeed = .013;
    static const float gripperMinForce = 5;

    ros::NodeHandle node, privateNode;

    // Messages
    ros::Publisher gripperCmdPublisher;
    ros::Subscriber gripperStatusSubscriber;

    // Actionlib
    actionlib::SimpleActionServer<control_msgs::GripperCommandAction> asGripperCommand;

    //gripper status
    robotiq_85_msgs::GripperStat gripperStatus;
};

#endif
