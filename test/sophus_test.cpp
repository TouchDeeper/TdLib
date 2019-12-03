//
// Created by wang on 19-11-5.
//
#include <sophus/se3.h>
#include <iostream>
#include <TdLibrary/motion_transformation.h>
class CandidateFromTraining {

public:
    CandidateFromTraining(){
        stop_subscribe_pose_ = false;
        cl_T_ = Sophus::SE3(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,0));
        cl_T_c_ = cl_T_;
        std::cout<<"cl_T_ = \n"<<cl_T_.matrix()<<std::endl;
        std::cout<<"cl_T_c_ = \n"<<cl_T_c_.matrix()<<std::endl;

        Eigen::Vector3d fixed_euler_angle(-1.5708,0,-1.5708);
//        Eigen::Matrix3d cl_R = (Eigen::AngleAxisd(fixed_euler_angle[2],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(fixed_euler_angle[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(fixed_euler_angle[0],Eigen::Vector3d::UnitX())).matrix();
//        Eigen::Matrix3d cl_R = (Eigen::AngleAxisd(fixed_euler_angle[0],Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(fixed_euler_angle[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(fixed_euler_angle[2],Eigen::Vector3d::UnitX())).matrix();
//        Eigen::Vector3d translate(0,0,0);
        cl_T_ = td::FixedPreTranslateThenEulertoSE3(fixed_euler_angle, Eigen::Vector3d(0,0,0));
        std::cout<<"cl_T_ = \n"<<cl_T_.matrix()<<std::endl;
//        Sophus::SE3 temp(fixed_euler_angle,translate);
        cl_T_c_ = cl_T_;
        std::cout<<"cl_T_c_ = \n"<<cl_T_c_.matrix()<<std::endl;
        Eigen::Matrix3d b_R_banana = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()).matrix();
        b_T_banana_ = Sophus::SE3(b_R_banana,Eigen::Vector3d(0,0,0));
    }
    void SimGetc_T_banana() {
//        ros::NodeHandle n;
//        ros::Subscriber sub = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, boost::bind(&CandidateFromTraining::CallbackGetRelativePose, this,_1,"kinect","banana"));
//        stop_subscribe_pose_ = false;
//        while(!stop_subscribe_pose_ && ros::ok())
//        {
//             循环等待回调函数
//            ros::spinOnce();
//        loop_rate_sub.sleep();
//        }
//        sub.shutdown();
//        Eigen::Vector3d fixed_euler_angle(-1.5708,0,-1.5708);
//        cl_T_ = td::FixedPreTranslateThenEulertoSE3(fixed_euler_angle, Eigen::Vector3d(0,0,0));
//        cl_T_c_ = cl_T_;

        std::cout<<"cl_T_c_ = \n"<<cl_T_c_.matrix()<<std::endl;
        std::cout<<"cl_T_ = \n"<<cl_T_.matrix()<<std::endl;

        c_T_banana_ = cl_T_c_.inverse() * c_T_banana_;
//        Eigen::Matrix3d b_R_banana = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX()).matrix();
//        b_T_banana_ = Sophus::SE3(b_R_banana,Eigen::Vector3d(0,0,0));
        std::cout<<"b_T_banana_ = \n"<<b_T_banana_.matrix()<<std::endl;
        c_T_banana_ = c_T_banana_ * b_T_banana_;
        std::cout<<"c_T_banana_ = \n"<<c_T_banana_.matrix()<<std::endl;
//    ROS_INFO("subscribe pose stop");
    }

private:
//    bool BowlInfoRequest();
//    void CallbackGetRelativePose(const gazebo_msgs::ModelStatesConstPtr& pvModelStates, std::string model_name_1, std::string model_name_2);

private:
    bool stop_subscribe_pose_;
    Sophus::SE3 c_T_bowl_;
    Sophus::SE3 c_T_banana_;
    Sophus::SE3 w_T_bowl_;
    Sophus::SE3 cl_T_;//cl_T_c = cl_T_ * I
    Sophus::SE3 cl_T_c_;//cl_T_c = cl_T_ * I
    Sophus::SE3 b_T_banana_;//The coordinate system of the banana in gazebo is transformed by rotating the raw coordinate -pi/2 alone x
};

int main(){
    CandidateFromTraining candidate_from_training;
    candidate_from_training.SimGetc_T_banana();
}