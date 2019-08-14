//
// Created by wang on 19-8-14.
//
#include <iostream>
#include <pangolin/pangolin.h>
#include "slam_typedef.h"
#include "draw_tool.h"
namespace td{
    /**
     * draw trajectory in pangolin
     * @param trajectory
     */
    void DrawTrajectory(const VecVector3d &trajectory) {
        if (trajectory.empty()) {
            std::cerr << "Trajectory is empty!" << std::endl;
            return;
        }

        // create pangolin window and plot the trajectory
        pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
                pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
        );

        pangolin::View &d_cam = pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(s_cam));


        while (pangolin::ShouldQuit() == false) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

            glLineWidth(2);
            for (size_t i = 0; i < trajectory.size() - 1; i++) {
                glColor3f(1 - (float) i / trajectory.size(), 0.0f, (float) i / trajectory.size());
                glBegin(GL_LINES);
                auto p1 = trajectory[i], p2 = trajectory[i + 1];
                glVertex3d(p1[0], p1[1], p1[2]);
                glVertex3d(p2[0], p2[1], p2[2]);
                glEnd();
            }
            pangolin::FinishFrame();
            usleep(5000);   // sleep 5 ms
        }

    }
}//namespace td
