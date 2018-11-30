#include<stdio.h>
#include<unistd.h>
#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<nav_msgs/Odometry.h>
#include<boost/thread/mutex.hpp>
#include<tf/tf.h>
#include<jw/projectMsg.h>

#define toRadian(degree)	((degree) * (M_PI / 180.))
#define toDegree(radian)	((radian) * (180. / M_PI))

boost::mutex mutex;
nav_msgs::Odometry g_odom;
float pre_dAngleTurned = 0;
ros::Publisher pub, mypub;

void odomMsgCallback(const nav_msgs::Odometry &msg)
{
    mutex.lock(); {
        g_odom = msg;
    } mutex.unlock();
}

tf::Transform getCurrentTransformation(void)
{
    tf::Transform transformation;

    nav_msgs::Odometry odom;
    
    mutex.lock(); {
        odom = g_odom;
    } mutex.unlock();

    transformation.setOrigin(tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));

    transformation.setRotation(tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w));

    return transformation;
}

tf::Transform getInitialTransformation(void)
{
    tf::Transform transformation;

    ros::Rate loopRate(1000.0);

    while(ros::ok()) {
        ros::spinOnce();

        transformation = getCurrentTransformation();

        if(transformation.getOrigin().getX() != 0. || transformation.getOrigin().getY() != 0. && transformation.getOrigin().getZ() != 0.) {
            break;
        } else {
            loopRate.sleep();
        }
    }

    return transformation;
}

double getHeadingAngle(tf::Transform temp)
{
    tf::Quaternion rotationQuat = temp.getRotation();
    double tempvalue = atan2((2 * rotationQuat[2] * rotationQuat[3]) , (1-(2 * (rotationQuat[2] * rotationQuat[2]) ) ));
    
    return tempvalue;
}

bool doRotation(tf::Transform &initialTransformation, double dRotation, double dRotationSpeed)
{
	jw::projectMsg sendmsg;
	sendmsg.Done = 1;
    geometry_msgs::Twist baseCmd;
    baseCmd.linear.x = 0.0;
    baseCmd.linear.y = 0.0;
    
    if(dRotation < 0.) 
        baseCmd.angular.z = -dRotationSpeed;
    else 
        baseCmd.angular.z = dRotationSpeed;
    
    bool bDone = false;
    ros::Rate loopRate(1000.0);

    while(ros::ok() && !bDone) 
    {
        ros::spinOnce();
      
        tf::Transform currentTransformation = getCurrentTransformation();
        tf::Transform relativeTransformation = initialTransformation.inverse() * currentTransformation ;

        double dAngleTurned = getHeadingAngle(relativeTransformation);
        
        if( fabs(dAngleTurned) > fabs(dRotation) || (abs(pre_dAngleTurned - dRotation) <  abs(dAngleTurned - dRotation)) || (dRotation == 0)) 
    	{
            bDone = true;
            pre_dAngleTurned = dAngleTurned;
            break;
        } 
        else
        {
	        pre_dAngleTurned = dAngleTurned;
        
            pub.publish(baseCmd);

            loopRate.sleep();
        }
    }
    
    
    baseCmd.linear.x = 0.05;
    baseCmd.angular.z = 0.0;
    pub.publish(baseCmd);

    mypub.publish(sendmsg);

    return bDone;
}

void projectMsgCallback(const jw::projectMsg &msg)
{
	geometry_msgs::Twist baseCmd;
    baseCmd.linear.x = 0.0;
    baseCmd.linear.y = 0.0;

	tf::Transform initialTransformation = getCurrentTransformation();

	// 방향지시는 없고 멈추거나 출발
	if(msg.Move == 0 || msg.Move == 1 && msg.Direction == 0)
	{
		if(msg.Move == 0)
			printf("Stop");
		else
			printf("Go");
		baseCmd.linear.x = 0.05 * msg.Move;
		pub.publish(baseCmd);
	}
	// 방향지시만 있음
	else if(msg.Move == -1 && msg.Direction == -1 || msg.Direction == 1)
	{
		pub.publish(baseCmd);     
		
		if(msg.Direction == -1)
			printf("Right");
		else
			printf("Left");

    	double dRotation = 90 * msg.Direction;
    	
		float _dRatation = fmod(dRotation, 360);
        
    	if(fabs(_dRatation) > 180)
    	{
	   	 	if(dRotation > 0) 
	        	dRotation = -(360-_dRatation);
        	else 
            	dRotation = (360+_dRatation); 
    	}
    	else
        	dRotation = _dRatation;
       
    	doRotation(initialTransformation, toRadian(dRotation), 0.75);
	}
}

int main(int ac, char *av[])
{
    ros::init(ac, av, "projectMove");

    ros::NodeHandle nhp, nhs, nh1, nh2;
    // odom msg
	ros::Subscriber sub = nhs.subscribe("/odom", 100, &odomMsgCallback);
    pub = nhp.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    // my msg	
	ros::Subscriber mysub = nh1.subscribe("project_msg", 100, &projectMsgCallback);
	mypub = nh2.advertise<jw::projectMsg>("project_msg_done", 100);

	ros::spin();
       
    return 0;
}