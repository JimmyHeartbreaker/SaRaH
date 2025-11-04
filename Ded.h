#pragma once

#include "lidar_lib.h" 
#include "pid_controller.h"
enum DedState
{
    STARTUP,
    SCAN,
    EST_TRANS,
    EST_ROT
};


void RefUpdate();
void SetPos(const Point2D& p);
void RotateCompleted(float rotation);
void DataIn(LidarScanNormalMeasureRaw* nodes, unsigned short nodeCount,bool  move_x, bool move_y, bool rotate);