#include <cartesian_interface/ros/RosClient.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace XBot::Cartesian;

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "pose_inspector");
    ros::NodeHandle nh;

    // Create publishers for boolean and float messages
    ros::Publisher approach_publisher = nh.advertise<std_msgs::Bool>("/approach_authorization", 1);
    ros::Publisher grasp_publisher = nh.advertise<std_msgs::Bool>("/grasp_authorization", 1);
    ros::Publisher pub = nh.advertise<std_msgs::Float32>("/your_topic_name", 10);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    RosClient cli;
    auto ee = cli.getTask("panda_link8");
    auto end_effector = std::dynamic_pointer_cast<CartesianTask>(ee);

    ros::Rate rate(2);  // Set the publishing rate (e.g., 2 Hz)
    const double sleep_dt = 0.1;

    bool choice_switch = true;

    while (ros::ok())
    {
      try
      {
        if (choice_switch)
        {
        choice_switch =false;

        // Look up the transform from source_frame to target_frame
        geometry_msgs::TransformStamped approach_transform;
        approach_transform = tf_buffer.lookupTransform("approach_frame", "ci/world", ros::Time::now());

        Eigen::Affine3d Tdes_approach;
        Eigen::Vector3d Translation;
        Translation << -approach_transform.transform.translation.x, -approach_transform.transform.translation.y, -approach_transform.transform.translation.z;
        Eigen::Quaterniond Quaternion;
        Quaternion.coeffs() << approach_transform.transform.rotation.x, approach_transform.transform.rotation.y, approach_transform.transform.rotation.z, approach_transform.transform.rotation.w;
        Tdes_approach.translation() = Translation;
        Tdes_approach.linear() = Quaternion.toRotationMatrix();

        cli.update(0, 0);
        double ex_time = 3.0;
        end_effector->setPoseTarget(Tdes_approach, ex_time);

        // wait until motion started
        while(end_effector->getTaskState() == State::Online)
        {
            cli.update(0, 0);
            ros::Duration(sleep_dt).sleep();
        }
        // wait until motion completed
        while(end_effector->getTaskState() == State::Reaching)
        {
            cli.update(0, 0);
            ros::Duration(sleep_dt).sleep();
        }

        Eigen::Affine3d Tcurrent;
        end_effector->getPoseReference(Tcurrent);

        Eigen::Vector3d translationDifference = Tcurrent.translation() - Tdes_approach.translation();
        double linear_error = translationDifference.norm();

        // Step 1: Compute the difference in rotations
        Eigen::Matrix3d rotationDifference = Tcurrent.rotation() * Tdes_approach.rotation().transpose();

        // Step 2: Convert the rotation matrix to an axis-angle representation
        Eigen::AngleAxisd axisAngle(rotationDifference);
        double angular_error = axisAngle.angle();

        std_msgs::Bool msg;
        bool output = (linear_error + angular_error < 1e-4);
        msg.data = output;

        // Publish the boolean message
        approach_publisher.publish(msg);
        }
        if (!choice_switch)
        {
        choice_switch =true;

        // Look up the transform from source_frame to target_frame
        geometry_msgs::TransformStamped grasp_transform;
        grasp_transform = tf_buffer.lookupTransform("final_grasp_frame", "ci/world", ros::Time::now());

        Eigen::Affine3d Tdes_grasp;
        Eigen::Vector3d Translation;
        Translation << -grasp_transform.transform.translation.x, -grasp_transform.transform.translation.y, -grasp_transform.transform.translation.z;
        Eigen::Quaterniond Quaternion;
        Quaternion.coeffs() << grasp_transform.transform.rotation.x, grasp_transform.transform.rotation.y, grasp_transform.transform.rotation.z, grasp_transform.transform.rotation.w;
        Tdes_grasp.translation() = Translation;
        Tdes_grasp.linear() = Quaternion.toRotationMatrix();

        cli.update(0, 0);
        double ex_time = 3.0;
        end_effector->setPoseTarget(Tdes_grasp, ex_time);

        // wait until motion started
        while(end_effector->getTaskState() == State::Online)
        {
            cli.update(0, 0);
            ros::Duration(sleep_dt).sleep();
        }
        // wait until motion completed
        while(end_effector->getTaskState() == State::Reaching)
        {
            cli.update(0, 0);
            ros::Duration(sleep_dt).sleep();
        }

        Eigen::Affine3d Tcurrent;
        end_effector->getCurrentPose(Tcurrent);

        Eigen::Vector3d translationDifference = Tcurrent.translation() - Tdes_grasp.translation();
        double linear_error = translationDifference.norm();

        // Step 1: Compute the difference in rotations
        Eigen::Matrix3d rotationDifference = Tcurrent.rotation() * Tdes_grasp.rotation().transpose();

        // Step 2: Convert the rotation matrix to an axis-angle representation
        Eigen::AngleAxisd axisAngle(rotationDifference);
        double angular_error = axisAngle.angle();

        std_msgs::Bool msg;
        bool output = (linear_error + angular_error < 1e-4);
        msg.data = output;

        // Publish the boolean message
        approach_publisher.publish(msg);
        }
      }
      catch (tf2::TransformException& ex)
      {
          ROS_WARN("%s", ex.what());
      }

      rate.sleep();
    }

    return 0;
}
