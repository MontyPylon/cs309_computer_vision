#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp> 
#include <config.h>
#include <string>

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OUT_WINDOW = "Output window";

int iLowH = 0;
int iHighH = 179;
int iLowS = 0;
int iHighS = 255;
int iLowV = 0;
int iHighV = 255;


class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter()
            : it_(nh_) {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/nav_kinect/rgb/image_color", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        /**
        //cv::namedWindow(OPENCV_WINDOW);
        cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

        //Create trackbars in "Control" window
        cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 255)
        cvCreateTrackbar("HighH", "Control", &iHighH, 179);

        cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        cvCreateTrackbar("HighS", "Control", &iHighS, 255);

        cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        cvCreateTrackbar("HighV", "Control", &iHighV, 255);
         **/
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // DISCLAIMER: This code was borrowed from: http://opencv-srf.blogspot.com/2010/09/object-detection-using-color-seperation.html
        using namespace cv;
        //declare output image
        cv::Mat original_image = cv_ptr->image;
        cv::Mat outImg;
        cv::Mat imgHSV;

        cvtColor(original_image, imgHSV, cv::COLOR_BGR2HSV); // Conver to HSV

        //cv::Mat imgThresholded;
        cv::Mat autoThresh;
        cv::Mat autoThreshContour;

        //inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
        inRange(imgHSV, cv::Scalar(MIN_HUE, MIN_SAT, MIN_VAL), cv::Scalar(MAX_HUE, MAX_SAT, MAX_VAL), autoThresh);

        cv::erode(autoThresh, autoThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
        cv::dilate( autoThresh, autoThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::dilate( autoThresh, autoThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::erode(autoThresh, autoThresh, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );

        
        inRange(imgHSV, cv::Scalar(MIN_HUE, MIN_SAT, MIN_VAL), cv::Scalar(MAX_HUE, MAX_SAT, MAX_VAL), autoThreshContour);

        cv::erode(autoThreshContour, autoThreshContour, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );
        cv::dilate( autoThreshContour, autoThreshContour, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::dilate( autoThreshContour, autoThreshContour, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(4, 4)) );
        cv::erode(autoThreshContour, autoThreshContour, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)) );



        
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        
        Mat temp = autoThreshContour.clone();
        /// Detect edges using canny
        //Canny( src_gray, canny_output, thresh, thresh*2, 3 );
        /// Find contours
        findContours( temp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        vector<Moments> mu(contours.size());
        for (int i = 0; i < contours.size(); i++) { 
            mu[i] = moments(contours[i], false); 
        }

        ///  Get the mass centers:
        vector<Point2f> mc(contours.size());
        double maxArea = -1;
        double numMax = -1;
        double minArea = 20;
        for (int i = 0; i < contours.size(); i++) { 
            mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00); 
            double area = mu[i].m00;
            //std::cout << "Area of moment " << i << ": " << mu[i].m00 << std::endl;
            
            int a = 10;
            std::stringstream ss;
            ss << area;
            string areaStr = ss.str();
            
            //cv::putText(original_image, areaStr, mc[i], 1, 1, CV_RGB(0, 255, 255));
            
            if((area > maxArea) && (area > minArea)) {
                maxArea = area;
                numMax = i;
            }
        }

        if(numMax != -1) {
            circle(original_image, mc[numMax], 5, CV_RGB(255, 255, 0), -1, 8, 0);
        }
        
        
            /**
        
             **/
        /**
        for( int i = 0; i< contours.size(); i++ )
        {
            circle( original_image, mc[i], 15, CV_RGB(0, 0, 255), -1, 8, 0 );
            //cv::circle(original_image, mc[i], 10, CV_RGB(255, 0, 0));
        }
         **/
        
        
        
        
        
        cv::imshow("Auto-Thresholded Image", autoThresh);
        cv::imshow("Auto-Thresholded Image", autoThreshContour);
        //cv::imshow("Thresholded Image", imgThresholded);
        cv::imshow("Original", original_image); //show the original image


        /**
        // Example 1: Draw an example circle on the video stream
        outImg = cv_ptr->image.clone();
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::circle(outImg, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
        
        //show input
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);

        //show output
        cv::imshow(OUT_WINDOW, outImg);
         **/

        //pause for 3 ms
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "image_converter");
    ImageConverter ic;
    
    
    
    ros::spin();
    return 0;
}
