
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/Twist.h>
#include <string>


using namespace cv;
bool findflag=false;
float px=0,py=0;
int cnts=0;
int cntmiss=0;
int cntf=0;
int turn =1;
int cntlost=0;
class TemplateMatchingNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh;
    ros::Publisher pub_vel;
    geometry_msgs::Twist cmd_msg;

    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;

    const char* template_img_path_ = "/home/yfukui/Pictures/person.png";
    Mat template_img_;
    int match_method_ = CV_TM_SQDIFF_NORMED;

public:
    TemplateMatchingNode()
        : it_(nh_)
    {
        sub_ = it_.subscribe("/rrbot/camera/image_raw", 1, &TemplateMatchingNode::camera_callback, this);
        template_img_ = cv::imread(template_img_path_);
        //add
        pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
    }
    geometry_msgs::Twist create_vel_msg(const double vel, const double omega)
    {
        // http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Twist.html
        geometry_msgs::Twist cmd_msg;
        cmd_msg.linear.x = vel;
        cmd_msg.angular.z = omega;
        return cmd_msg;
    }
    ~TemplateMatchingNode()
    {
    }


    void camera_callback(const sensor_msgs::ImageConstPtr& img_msg)
    {
        Mat img,tmp;
        try
        {
            img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
            waitKey(30);
        }
        catch (cv_bridge::Exception & e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg->encoding.c_str());
        }

        // https://docs.opencv.org/3.4/de/da9/tutorial_template_matching.html
        Mat result;
        tmp = cv::imread(template_img_path_);
        matchTemplate(img, tmp, result, match_method_);
        double val;
        Point pt;
        minMaxLoc(result, &val, nullptr, &pt, nullptr);
        std::cout << "val:" <<val<< std::endl;
        std::cout<<"pt:"<<pt<<std::endl;
        if(val<0.20){
            cntf+=1;
            if(cntf>=5)findflag=true;
        }else if (findflag==false){
            cnts+=1;
        }
        Point pt2= pt + cv::Point(tmp.cols, tmp.rows); 
        int vecx=pt.x-px;
        int vecy=pt.y-py;
        if(val<0.20 && pt.x<200){
            turn =1;
        }else if(val<0.20 && pt.x>380){
            turn =-1;
        }
        std::cout<<"vecx:"<<vecx<<std::endl;
        std::cout<<"vecy:"<<vecy<<std::endl;
        if (val < 0.20 && findflag)
        {
            cntmiss=0;
            cv::rectangle(img,
                pt,                                   
                pt + cv::Point(tmp.cols, tmp.rows),   
                cv::Scalar(0, 0, 255),              //boxcolor  
                2,                                    
                cv::LINE_8                            
            );
            if (pt.x < 200)
            {
                cmd_msg = create_vel_msg(0.5, 0.5);
                pub_vel.publish(cmd_msg);
                ROS_INFO("left_top point %d : rotate_left", pt.x);
            }
            else if(pt.x>=330)
            {
                cmd_msg = create_vel_msg(0.5, -0.5);
                pub_vel.publish(cmd_msg);
                ROS_INFO("left_top point %d : rotate_right", pt.x);
            }
            else{
                cmd_msg = create_vel_msg(0.7, 0.0);
                pub_vel.publish(cmd_msg);
                ROS_INFO("left_top point %d : go straight", pt.x);
            }
            px=pt.x;
            py=pt.y;
        }else{
            cntmiss+=1;
            if (cntmiss<=3 && findflag){
                cmd_msg = create_vel_msg(0.7, 0.0);
                pub_vel.publish(cmd_msg);
                ROS_INFO("left_top point %d : go straight but lost", pt.x);
            }else{
                findflag=false;
                if(cntf<=2){
                    if(cnts<10){
                        cmd_msg=create_vel_msg(0,0);
                        pub_vel.publish(cmd_msg);
                        ROS_INFO("I'm wating!");
                    }else{
                        cmd_msg=create_vel_msg(0,0.0);
                        pub_vel.publish(cmd_msg);
                        ROS_INFO("I'm waiting");
                    }
                }
                else if(turn==1){
                    cmd_msg=create_vel_msg(0,0.6);
                    pub_vel.publish(cmd_msg);
                    ROS_INFO("missing target and rotate_left");
                }else if(turn==-1){
                    cmd_msg=create_vel_msg(0,-0.6);
                    pub_vel.publish(cmd_msg);
                    ROS_INFO("missing target and rotate_right");
                }
            }
        }
        if(val<0.20){
            std::string st="left_top="+std::to_string(pt.x);
            putText(img, st, Point(25,75),FONT_HERSHEY_PLAIN, 2,Scalar(0,100,30), 2, LINE_AA);
        }
        std::string st="val="+std::to_string(pt.x);
        putText(img, st, Point(25,75),FONT_HERSHEY_PLAIN, 2,Scalar(0,100,30), 2, LINE_AA);
        imshow("subscribed img", img);

        //add
        
        waitKey(30);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "matching_node");
    TemplateMatchingNode node = TemplateMatchingNode();
    ros::spin();

    return 0;
}