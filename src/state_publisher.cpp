#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    double right_shoulder_pitch = 0, right_shoulder_roll = 0, right_elbow_roll = -0.5;
    double step_size_RSP = 0.02, step_size_RSR = 0.02, step_size_RER = 0.02;

    ros::Duration(5, 0).sleep();
    int state = 0;

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(8);
        joint_state.position.resize(8);
        joint_state.name[0] ="HeadYaw";
        joint_state.position[0] = 0;
        joint_state.name[1] ="HeadPitch";
        joint_state.position[1] = 0;
        joint_state.name[2] ="RightShoulderPitch";
        joint_state.position[2] = right_shoulder_pitch;
        joint_state.name[3] = "RightShoulderRoll";
        joint_state.position[3] = right_shoulder_roll;
        joint_state.name[4] = "RightElbowRoll";
        joint_state.position[4] = right_elbow_roll;
        joint_state.name[5] = "LeftShoulderPitch";
        joint_state.position[5] = 1.14;
        joint_state.name[6] = "LeftShoulderRoll";
        joint_state.position[6] = -0.95;
        joint_state.name[7] = "LeftElbowRoll";
        joint_state.position[7] = -0.87;


        // update transform
        // (moving in a circle with radius=2)
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = 0;
        odom_trans.transform.translation.y = 0;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);

        //send the joint state and transform
        joint_pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        // create hand-waving movement
        if(state == 0)
        {
            right_elbow_roll -= step_size_RER;

            if(right_elbow_roll < -1.25)
            {
                state = 1;
            }
        }
        else if(state == 1)
        {
            right_shoulder_pitch += step_size_RSP;

            if(right_shoulder_pitch > 1.58)
            {
                state = 2;
            }
        }
        else if(state == 2)
        {
            right_elbow_roll += step_size_RER;
            if(right_elbow_roll < -1.57 || right_elbow_roll > -1) step_size_RER *= -1;
        }

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
