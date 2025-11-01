#pragma once

#include "lidar_lib.h" 
enum DedState
{
    STARTUP,
    SCAN,
    EST_TRANS,
    ROTATING,
    EST_ROT
};




void RotateCompleted(float rotation);
void DataIn(LidarScanNormalMeasureRaw* nodes, unsigned short nodeCount,bool  move_x, bool move_y, bool rotate);