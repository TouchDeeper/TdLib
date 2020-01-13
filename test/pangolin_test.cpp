//
// Created by wang on 19-8-14.
//
#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>
#include "TdLibrary/slam_tool.h"
#include "TdLibrary/draw_tool.h"
using namespace Eigen;

using namespace std;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
//void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>);
int main(){
    string compare_file = "../compare.txt";
    VecVector3d t_es,t_gs,q_es,q_gs;//原坐标与去质心坐标容器

    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_Tgis;
    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses_Teis;
    Eigen::Matrix<double,3,3> W;
    /// implement pose reading code
    // start your code here (5~10 lines)
    ifstream compare(compare_file);
    if(!compare)
    {
        cout<<"unable to read compare.txt"<<endl;
        exit(1);
    }
    //取数据并表示成SE(3)形式
    while(!compare.eof())
    {
        double data[16] = {0};
        for(auto&i:data)
            compare>>i;
        Eigen::Quaterniond R_e(data[7],data[4],data[5],data[6]);
        Eigen::Quaterniond R_g(data[15],data[12],data[13],data[14]);
        Eigen::Vector3d t_e(data[1],data[2],data[3]);
        Eigen::Vector3d t_g(data[9],data[10],data[11]);
        t_es.push_back(t_e);
        t_gs.push_back(t_g);
        Sophus::SE3d Tgi(R_g,t_g);
        poses_Tgis.push_back(Tgi);
        Sophus::SE3d Tei(R_e,t_e);
        poses_Teis.push_back(Tei);
    }
    compare.close();
    assert(t_es.size() == t_gs.size());
    Eigen::Matrix3d Reg;
    Eigen::Vector3d t;
    td::IcpTrajectoryAlign(t_es, t_gs, Reg, t);
    //通过R,t构造Teg
    Sophus::SE3d Teg(Reg, t);

    //将poses_Teis转换到poses_Tgis的坐标系
    for(auto &t_:t_gs)
        t_ = Teg*t_;


    // draw trajectory in pangolin
    //将两条轨迹拼接成一个poses_Te以作对比
    VecVector3d t_all;
    t_all.insert(t_all.end(),t_gs.begin(),t_gs.end());
    t_all.insert(t_all.end(),t_es.begin(),t_es.end());

    td::DrawTrajectory(t_all);
    std::cout<<"Hello word"<<std::endl;
    Eigen::Vector3d vec(1,1,1);
    std::cout<<vec.norm()<<std::endl;
    std::cout<<vec<<std::endl;
}
///*******************************************************************************************/
//void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses) {
//    if (poses.empty()) {
//        cerr << "Trajectory is empty!" << endl;
//        return;
//    }
//
//    // create pangolin window and plot the trajectory
//    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
//    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_BLEND);
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//
//    pangolin::OpenGlRenderState s_cam(
//            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
//            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
//    );
//
//    pangolin::View &d_cam = pangolin::CreateDisplay()
//            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
//            .SetHandler(new pangolin::Handler3D(s_cam));
//
//
//    while (pangolin::ShouldQuit() == false) {
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//
//        d_cam.Activate(s_cam);
//        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
//
//        glLineWidth(2);
//        for (size_t i = 0; i < poses.size() - 1; i++) {
//            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
//            glBegin(GL_LINES);
//            auto p1 = poses[i], p2 = poses[i + 1];
//            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
//            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
//            glEnd();
//        }
//        pangolin::FinishFrame();
//        usleep(5000);   // sleep 5 ms
//    }
//
//}