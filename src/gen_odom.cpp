#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <turtlesim/Pose.h>




class GenOdom
{
    public: 

    tf::TransformBroadcaster odom_broadcaster;
    ros::Publisher odom_pub;
    ros::Subscriber pose_sub; 

    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;



    GenOdom(ros::NodeHandle nh){
        odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
        pose_sub = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 20, &GenOdom::pose_callback,this);
    }

    void pose_callback(const turtlesim::Pose::ConstPtr &);
    void publish_odom();


};

void GenOdom::publish_odom()
{
     odom_trans.header.stamp = ros::Time::now();
     odom_pub.publish(odom);

     odom_broadcaster.sendTransform(odom_trans);

}

void GenOdom::pose_callback(const turtlesim::Pose::ConstPtr& msg)
{
    //update the odometry

    ros::Time current_time = ros::Time::now();


    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(msg->theta);

    //first, we'll publish the transform over tf
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = msg->x;
    odom_trans.transform.translation.y = msg->y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

   
    //next, we'll publish the odometry message over ROS
    odom.header.stamp = current_time;
    odom.header.frame_id = "map";

    //set the position
    odom.pose.pose.position.x = msg->x;
    odom.pose.pose.position.y = msg->y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = msg->linear_velocity * cos(msg->theta);
    odom.twist.twist.linear.y = msg->linear_velocity * sin(msg->theta);
    odom.twist.twist.angular.z = msg->angular_velocity;

  


}

int main(int argc, char** argv){

  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;

  GenOdom go(n);

  
  ros::Rate r(20);

  while(n.ok()){
     ros::spinOnce();               // check for incoming messages
     
     //publish the message
     go.publish_odom();
   
     r.sleep();
  }
}