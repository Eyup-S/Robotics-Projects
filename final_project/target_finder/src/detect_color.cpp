#include <iostream>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>
#include <tf/tf.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/core.hpp>


using namespace std;
using namespace cv;

cv::Mat imgOriginal;
std::string camera_topic = "";

void imageCb(const sensor_msgs::ImageConstPtr &msg) {

    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);


    }
    catch (cv_bridge::Exception &e) {
        //imgOriginal.release();
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat &im = cv_ptr->image;
    imgOriginal = im;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "detect_color");
    ros::NodeHandle parameterServer("~");
    ros::NodeHandle n;
    ros::Rate loop_rate(5);

    //get params from server
    parameterServer.getParam("camera_topic",camera_topic);
    ros::Subscriber rgb_image_sub_ = n.subscribe(camera_topic, 3, imageCb);

    while (ros::ok()) {

        ros::Time begin = ros::Time::now();

        namedWindow("Control", WINDOW_AUTOSIZE); //create a window called "Control"

        int iLowH = 0;
        int iHighH = 179;
        int iLowS = 0;
        int iHighS = 255;
        int iLowV = 0;
        int iHighV = 255;

        //Create trackbars in "Control" window
        createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
        createTrackbar("HighH", "Control", &iHighH, 179);

        createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        createTrackbar("HighS", "Control", &iHighS, 255);

        createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        createTrackbar("HighV", "Control", &iHighV, 255);

        while (true) {

            loop_rate.sleep();

            ros::spinOnce();


            if (!imgOriginal.empty()) //if success, loop
            {
                Mat imgHSV, imgThresholded;
                cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
                inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV),
                        imgThresholded); //Threshold the image

                //morphological opening (remove small objects from the foreground)
                erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
                dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

                //morphological closing (fill small holes in the foreground)
                dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
                erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

                imshow("Thresholded Image", imgThresholded); //show the thresholded image
                imshow("Original", imgOriginal); //show the original image

                if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
                {
                    cout << "esc key is pressed by user" << endl;
                    break;
                }


            }


        }

        loop_rate.sleep();

        ros::spinOnce();

    }

    return 0;
}
