
#ifndef ANGLE_TREE_H // include guard
#define ANGLE_TREE_H

#include "arduino_ext.h"
#include "robot_math.h"
#define N_POINTS  1440
#define RESOLUTION (M_2PI/N_POINTS) //rad


typedef struct ArcNode
{
    float angle;
    float dist; 
    Point2D point;
} __attribute__((packed)) ArcNode;




typedef struct Transform
{
    Point2D Point;
    float Angle;
    float Dist;
    bool IsEmpty;
} __attribute__((packed)) Transform;

typedef struct Wall
{
    Transform Start;
    Transform End;
} __attribute__((packed)) Wall;

int FindClosestNode(float angle, float dist, ArcNode* flatNodes,int searchSIze);

void FindBestFlatSurface(ArcNode* nodes, unsigned short nodeCount,Wall* walls);
void Clear(ArcNode* stack, unsigned short nodeCount);
void PrintNodes(ArcNode* nodes);
void Copy(ArcNode* src, ArcNode* dst);
void CalcPoints(ArcNode* nodes);

void InterpolateMissingNodes(ArcNode* nodes);
int CountZeros(ArcNode* nodes);
#endif