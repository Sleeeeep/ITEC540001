#include<ros/ros.h>
#include<sensor_msgs/CompressedImage.h>
#include<image_transport/image_transport.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<jw/projectMsg.h>
#include<sensor_msgs/LaserScan.h>
#include<boost/thread/mutex.hpp>

using namespace cv;
using namespace std;

boost::mutex mutex;
bool Turning = 0;
Mat Leftsrc, Rightsrc, Stopsrc, Gosrc;
vector< vector< Point> > leftcontours, rightcontours, stopcontours, gocontours;
ros::Publisher mypub;

template<typename T> inline bool isnan(T value)
{
	return value != value;
}

void scanMsgCallback(const sensor_msgs::LaserScan &msg)
{
	if(Turning)
		return ;
	bool obstacle = false;
	
	int nRangeSize = (int)msg.ranges.size();
	double leftSum = 0, frontSum = 0, rightSum = 0;
    int leftcount = 0, frontcount = 0, rightcount = 0;
	
	for(int i=0; i<nRangeSize; i++)
	{
		if(i>0 && i<=20)
		{    
			double dRange = msg.ranges[i];
			
			if(isnan(dRange) || dRange > msg.range_max)
				continue;
			else
			{   
				if(i <= 10)
				{
					frontcount++;
					frontSum += dRange;
				}
				leftcount++;
			    leftSum += dRange;
			}
		}
		else if(i>=340 && i<360)
		{    
			double dRange = msg.ranges[i];
			
			if(isnan(dRange) || dRange > msg.range_max)
				continue;
			else
			{   
				if(i >= 350)
				{
					frontcount++;
					frontSum += dRange;
				}
				rightcount++;
			    rightSum += dRange;
			}
		}
		else
			continue;
			
	}
    if(leftcount != 0)
	{
//		printf("%lf  ", leftSum/leftcount);
		if(leftSum/leftcount < 0.35)
			obstacle = true;
	}
	if(frontcount != 0)
	{
//		printf("%lf  ", frontSum/frontcount);
		if(frontSum/frontcount < 0.35)
			obstacle = true;
	}
	if(rightcount != 0)
	{
//		printf("%lf\n", rightSum/rightcount);
		if(rightSum/rightcount < 0.35)
			obstacle = true;
	}

	if(obstacle)
	{
		printf("Detect obstacle!!! ");
		
		mutex.lock(); {
			Turning = true;
		} mutex.unlock();

		jw::projectMsg obstaclemsg;
		obstaclemsg.Move = -1;
		obstaclemsg.Done = 0;
		switch(rand()%2)
		{
			case 0:
				printf("(Turn Right)\n");
				obstaclemsg.Direction = -1;
				break;
			case 1:
				printf("(Turn Left)\n");
				obstaclemsg.Direction = 1;
				break;
		}
		mypub.publish(obstaclemsg);
	}
}

void projectMsgCallback(const jw::projectMsg &msg)
{
	printf("Accept Done Message for Turn\n");
	Turning = false;
}

void grayInverse(Mat &image, Mat &output)
{
	if(image.channels() != 1)
		return ;

	output = image.clone();

	for(int y = 0; y < image.size().height; y++)
		for(int x = 0; x < image.size().width; x++)
			output.at<uchar>(y, x) = 255 - output.at<uchar>(y, x);
}

int matchingPattern(vector< Point> item)
{
	double temp=matchShapes(leftcontours[0], item, CV_CONTOURS_MATCH_I1, 0);
	temp = min(temp, matchShapes(rightcontours[0], item, CV_CONTOURS_MATCH_I1, 0));
	temp = min(temp, matchShapes(stopcontours[0], item, CV_CONTOURS_MATCH_I1, 0));
	temp = min(temp, matchShapes(gocontours[0], item, CV_CONTOURS_MATCH_I1, 0));

	if(temp > 0.5)
		return 0;

	if(temp == matchShapes(leftcontours[0], item, CV_CONTOURS_MATCH_I1, 0))
		return 1;
	if(temp == matchShapes(rightcontours[0], item, CV_CONTOURS_MATCH_I1, 0))
		return 2;
	if(temp == matchShapes(stopcontours[0], item, CV_CONTOURS_MATCH_I1, 0))
		return 3;
	if(temp == matchShapes(gocontours[0], item, CV_CONTOURS_MATCH_I1, 0))
		return 4;
}

void findPattern(Mat &image)
{
	jw::projectMsg msg;
	msg.Direction = 0;
	msg.Move = -1;
	msg.Done = 0;

	GaussianBlur(image, image, Size(5,5), 1,1);

	Mat Gray;
	// 흑백화
	cvtColor(image, Gray, CV_BGR2GRAY);

	Mat Binary(image.size(), CV_8U, Scalar(255));
	// 이진화
	threshold(Gray, Binary, 60, 255, THRESH_BINARY);
	// 흑백 전환
	grayInverse(Binary, Binary);

	vector< vector< Point> > contours;
	// contour 찾기
	findContours(Binary, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	
	drawContours(Gray, contours, -1, Scalar(255, 0, 0), 2);
	vector< Point> approx;
	
	for(size_t i=0; i<contours.size() && !Turning; i++)
	{
		approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
		
		if(fabs(contourArea(Mat(approx)) < 10000) || fabs(contourArea(Mat(approx)) > 50000))
			continue;
		
		int	temp = matchingPattern(contours[i]);	
		switch(temp)
		{
			case 1:
				drawContours(image, contours, i, Scalar(0,0,255), 2);
				printf("Turn Left\n");
				msg.Direction = 1;
			
				mutex.lock(); {
					Turning = true;
				} mutex.unlock();
				
				break;
			case 2:
				drawContours(image, contours, i, Scalar(0,0,255), 2);
				printf("Turn Right\n");
				msg.Direction = -1;
			
				mutex.lock(); {
					Turning = true;
				} mutex.unlock();
				
				break;
			case 3:
				drawContours(image, contours, i, Scalar(0,0,255), 2);
				printf("Stop\n");
				msg.Move = 0;
				break;
			case 4:
				drawContours(image, contours, i, Scalar(0,0,255), 2);
				printf("Go\n");
				msg.Move = 1;
				break;
		}
			mypub.publish(msg);
			break;
	}

}

void poseMessageReceivedRGB(const sensor_msgs::ImageConstPtr& msg) {
	
	Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
	
	findPattern(img);

	imshow("img", img);
	
	char input = waitKey(30);
	if(input == 27)
	{
		jw::projectMsg endmsg;
		endmsg.Direction = 0;
		endmsg.Move = 0;
		mypub.publish(endmsg);
		printf("ESC\n");
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "projectCamera");
	ros::NodeHandle nh1, nh2, nhs, nhp;
	image_transport::ImageTransport it(nh1);

	ros::Subscriber sub = nh2.subscribe("/scan", 10, &scanMsgCallback);

	ros::Subscriber mysub = nhs.subscribe("project_msg_done", 100, &projectMsgCallback);
	mypub = nhp.advertise<jw::projectMsg>("project_msg", 100);

	Leftsrc = imread("Left.png");
	Rightsrc = imread("Right.png");
	Stopsrc = imread("Stop.png");
	Gosrc = imread("Go.png");

	if(Leftsrc.empty() || Rightsrc.empty() || Stopsrc.empty() || Gosrc.empty())
	{
		fprintf(stderr, "cannot find png src file\n");
		exit(1);
	}
	
	srand(time(NULL));

	Mat Gray;
	
	GaussianBlur(Leftsrc, Leftsrc, Size(5,5), 1,1);
	cvtColor(Leftsrc, Gray, CV_BGR2GRAY);
	Mat leftBinary(Leftsrc.size(), CV_8U, Scalar(255));
	threshold(Gray, leftBinary, 60, 255, THRESH_BINARY);
	grayInverse(leftBinary, leftBinary);
	findContours(leftBinary, leftcontours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	
	GaussianBlur(Rightsrc, Rightsrc, Size(5,5), 1,1);
	cvtColor(Rightsrc, Gray, CV_BGR2GRAY);
	Mat rightBinary(Rightsrc.size(), CV_8U, Scalar(255));
	threshold(Gray, rightBinary, 60, 255, THRESH_BINARY);
	grayInverse(rightBinary, rightBinary);
	findContours(rightBinary, rightcontours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	
	GaussianBlur(Stopsrc, Stopsrc, Size(5,5), 1,1);
	cvtColor(Stopsrc, Gray, CV_BGR2GRAY);
	Mat stopBinary(Stopsrc.size(), CV_8U, Scalar(255));
	threshold(Gray, stopBinary, 60, 255, THRESH_BINARY);
	grayInverse(stopBinary, stopBinary);
	findContours(stopBinary, stopcontours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	
	GaussianBlur(Gosrc, Gosrc, Size(5,5), 1,1);
	cvtColor(Gosrc, Gray, CV_BGR2GRAY);
	Mat goBinary(Gosrc.size(), CV_8U, Scalar(255));
	threshold(Gray, goBinary, 60, 255, THRESH_BINARY);
	grayInverse(goBinary, goBinary);
	findContours(goBinary, gocontours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

	image_transport::Subscriber subRGB = it.subscribe("/raspicam_node/image", 1, &poseMessageReceivedRGB, ros::VoidPtr(), image_transport::TransportHints("compressed"));

	ros::spin();

	return 0;
}
