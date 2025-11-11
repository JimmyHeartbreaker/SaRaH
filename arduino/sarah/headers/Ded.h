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



void Reset();
ArcNode* GetRefNodes();
void GetNewNodes(ArcNode* dst );
void RefUpdate();
void SetPos(const Point2D& p);
Point2D GetPos();

float GetYaw();
void RotateCompleted(float rotation);
void DataIn(LidarScanNormalMeasureRaw* nodes, unsigned short nodeCount);
bool TryMakeRefScan();

Transform EstimateTranslation(float x, float y, float yaw,float& confidence);
Transform EstimateRotation(float x, float y, float yaw,float& confidence);
#endif