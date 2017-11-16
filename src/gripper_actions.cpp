#include <robotiq_85_gripper_actions/gripper_actions.h>

using namespace std;

GripperActions::GripperActions() : privateNode("~"),
    asGripperCommand(privateNode, "gripper_command", boost::bind(&GripperActions::executeGripperCommand, this, _1), false)
{
    // Read in parameters
    string gripperCommandTopic("/gripper/cmd");
    string gripperStatusTopic("/gripper/stat");
    privateNode.getParam("gripper_command_topic", gripperCommandTopic);
    privateNode.getParam("gripper_status_topic", gripperStatusTopic);

    // Messages
    gripperCmdPublisher = node.advertise<robotiq_85_msgs::GripperCmd>(gripperCommandTopic, 1);

    gripperStatusSubscriber = node.subscribe(gripperStatusTopic, 1, &GripperActions::gripperStatusCallback, this);

    // Action servers
    asGripperCommand.start();
}

void GripperActions::gripperStatusCallback(const robotiq_85_msgs::GripperStat msg)
{
    gripperStatus = msg;
}

void GripperActions::executeGripperCommand(const control_msgs::GripperCommandGoalConstPtr &goal)
{
    //TODO: cancel goal on the gripperManipulation action server
    ROS_INFO("Got goal: %f", goal->command.position);

    control_msgs::GripperCommandResult result;

    //send command to gripper
    robotiq_85_msgs::GripperCmd cmd;
    cmd.emergency_release = false;
    cmd.stop = false;
    cmd.position = goal->command.position;
    cmd.speed = defaultGripperSpeed;
    cmd.force = goal->command.max_effort;
    gripperCmdPublisher.publish(cmd);

    //wait until it's finished
    ros::Rate loopRate(30);
    bool positionReached = false;
    bool gripperStopped = false;
    ros::Duration(0.25).sleep(); //give gripper time to start moving
    while (!(positionReached || gripperStopped))
    {
        gripperStopped = !gripperStatus.is_moving;
        positionReached = fabs(gripperStatus.position - cmd.position) < .001;

        loopRate.sleep();
    }

    //stop gripper
    cmd.stop = true;
    gripperCmdPublisher.publish(cmd);

    //publish result
    result.position = gripperStatus.position;
    result.reached_goal = positionReached;
    asGripperCommand.setSucceeded(result);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_actions");

    GripperActions ga;
    ROS_INFO("Started.");

    ros::spin();
}
