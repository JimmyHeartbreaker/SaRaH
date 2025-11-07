#ifndef DED_H
#define DED_H

#include "lidar_lib.h" 
#include "pid_controller.h"
enum DedState
{
    STARTUP,
    SCAN,
    EST_TRANS,
    EST_ROT
};



ArcNode* GetRefNodes();
void GetNewNodes(ArcNode* dst );
void RefUpdate();
void SetPos(const Point2D& p);
Point2D GetPos();

float GetYaw();
void RotateCompleted(float rotation);
void DataIn(LidarScanNormalMeasureRaw* nodes, unsigned short nodeCount,bool  move_x, bool move_y, bool rotate);

#endif