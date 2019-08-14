//
// Created by wang on 19-8-14.
//

#ifndef TDLIB_DRAW_TOOL_H
#define TDLIB_DRAW_TOOL_H

#include "slam_typedef.h"
namespace td{
    /**
     * draw trajectory in pangolin
     * @param trajectory
     */
    void DrawTrajectory(const VecVector3d &trajectory);
}
#endif //TDLIB_DRAW_TOOL_H
