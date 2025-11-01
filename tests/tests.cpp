#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include "..\pid_controller.h"
#define DEG2RAD(x) ((x) * M_PI / 180.0f)

std::vector<ArcNode> generateNShapeCustom(
    unsigned short n_POINTS,
    float topWallY,
    float topWallXStart,
    float topWallXEnd,
    float verticalWallX,
    float verticalWallYStart,
    float verticalWallYEnd,
    float bottomWallY,
    float bottomWallXStart,
    float bottomWallXEnd,
    float sensorRotationDeg = 0.0f,
    Point2D sensorTranslation = {0.0f, 0.0f})
{
    std::vector<ArcNode> nodes(n_POINTS);
    float sensorRot = DEG2RAD(sensorRotationDeg);

    for (int i = 0; i < n_POINTS; i++)
    {
        float theta = DEG2RAD((360.0f * i) / n_POINTS) + sensorRot;
        Point2D d = {sinf(theta), cosf(theta)}; // ray direction

        float t_min = 1e6f;
        Point2D p = {0.0f, 0.0f};

        // --- Top horizontal wall ---
        if (fabs(d.Y) > 1e-6f)
        {
            float t = (topWallY - sensorTranslation.Y) / d.Y;
            float x_hit = d.X * t - sensorTranslation.X;
            if (t > 0 && x_hit >= topWallXStart && x_hit <= topWallXEnd && t < t_min)
            {
                t_min = t;
                p = {x_hit, topWallY - sensorTranslation.Y};
            }
        }

        // --- Vertical wall ---
        if (fabs(d.X) > 1e-6f)
        {
            float t = (verticalWallX - sensorTranslation.X) / d.X;
            float y_hit = d.Y * t - sensorTranslation.Y;
            if (t > 0 && y_hit >= verticalWallYStart && y_hit <= verticalWallYEnd && t < t_min)
            {
                t_min = t;
                p = {verticalWallX - sensorTranslation.X, y_hit};
            }
        }

        // --- Bottom horizontal wall ---
        if (fabs(d.Y) > 1e-6f)
        {
            float t = (bottomWallY - sensorTranslation.Y) / d.Y;
            float x_hit = d.X * t - sensorTranslation.X;
            if (t > 0 && x_hit >= bottomWallXStart && x_hit <= bottomWallXEnd && t < t_min)
            {
                t_min = t;
                p = {x_hit, bottomWallY - sensorTranslation.Y};
            }
        }

        nodes[i].angle = fmodf(theta + 2 * M_PI, 2 * M_PI);
        nodes[i].dist = sqrtf(p.X * p.X + p.Y * p.Y);
        nodes[i].point = p;
    }

    return nodes;
}

// Generate synthetic LIDAR frame
std::vector<ArcNode> generateSlopedWallScan(
    unsigned short n_POINTS,
    float wallAngleDeg,
    float wallDistance,
    float sensorRotationDeg = 0.0f,
    Point2D sensorTranslation = {0.0f, 0.0f})
{
    std::vector<ArcNode> nodes(n_POINTS);
    float wallAngle = DEG2RAD(wallAngleDeg);
    float sensorRot = DEG2RAD(sensorRotationDeg);

    // wall normal (points toward the sensor)
    float ny = cosf(wallAngle);
    float nx = sinf(wallAngle);

    for (int i = 0; i < N_POINTS; i++)
    {
        float theta = DEG2RAD((360.0f * i) / N_POINTS) + sensorRot;
        float dy = cosf(theta);
        float dx = sinf(theta);

        // Solve for ray–plane intersection:
        // (p = t*d), wall: n·p = wallDistance
        float denom = nx * dx + ny * dy;
        float t = 0;
        if (fabs(denom) > 1e-6f)
        {
            t = wallDistance / denom;
        }
        else
        {
            t = 100.0f; // no hit (parallel)
        }

        Point2D p = {dx * t - sensorTranslation.X, dy * t - sensorTranslation.Y};

        nodes[i].angle = fmodf(theta + 2 * M_PI, 2 * M_PI);
        nodes[i].dist = sqrtf(p.X * p.X + p.Y * p.Y);
        nodes[i].point = p;
    }

    return nodes;
}
void exportPointsToCSV(const std::vector<ArcNode>& nodes, const std::string& filename)
{
    std::ofstream file(filename);
    file << "X,Y\n";
    for (const auto& n : nodes)
    {
        file << n.point.X << "," << n.point.Y << "\n";
    }
    file.close();
}
int main()
{
    const unsigned short n_POINTS = 720;
    float wallAngleDeg = 30.0f;   // wall tilted 30° from forward
    float wallDistance = 2.0f;    // wall ~2m in front of sensor

    // Frame 1: stationary
    float top = 100;
    float bottom = 50;
    float left = -100;
    float mid = 0;
    float right = 100;
    auto old_nodes = generateNShapeCustom(n_POINTS,top,left,mid,mid,top,bottom,bottom,mid,right,0,{0,0});

    exportPointsToCSV(old_nodes, "lidar_points.csv");
    // Frame 2: optionally rotated by 0° (no real motion)
    auto new_nodes = generateNShapeCustom(n_POINTS,top,left,mid,mid,top,bottom,bottom,mid,right,0.0,{0,0});

    // Print subset for debugging
    for (int i = 0; i < n_POINTS; i += 30)
    {
        std::cout << i << " angle=" << old_nodes[i].angle * 180 / M_PI
                  << " dist=" << old_nodes[i].dist << std::endl;
    }
	float std;
	float angleDiff=0;
	ArcNode new_nodesa[1440];
	ArcNode new_nodesb[1440];
	std::copy(new_nodes.begin(), new_nodes.end(), new_nodesa); 
	std::copy(old_nodes.begin(), old_nodes.end(), new_nodesb); 
		Point2D diff = {0,0};
        Point2D heading = {0,1};
	while(true)
	{

		Point2D newdiff = sum_difference(new_nodesa,new_nodesb,-0.1,0.1,diff,-angleDiff/10,std,angleDiff,heading);
        
		diff.X +=newdiff.X/10;
        diff.Y +=newdiff.Y/10;
		//printf("%.2f",diff);
	}
    return 0;
}