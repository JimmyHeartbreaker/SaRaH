
#pragma once

struct AngleNode
{
    float angle;
    float dist;
    AngleNode* left;
    AngleNode* right;    
    unsigned short index;
}; 

struct Point
{
    double X;
    double Y;
};
struct Transform
{
    Point Point;
    double Angle;
    double Dist;
    bool IsEmpty;
};

struct Wall
{
    Transform Start;
    Transform End;
};

Transform FindNode(double angle, double dist, AngleNode* node, AngleNode* flatNodes, int nodeCount);
int InsertAngle(AngleNode* tree, double angle, double dist);

int Flatten(AngleNode* node, AngleNode* stack);
Wall FindFlatSurface(AngleNode* nodes, int nodeCount);