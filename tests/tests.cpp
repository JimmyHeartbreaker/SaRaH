#include <cmath>
#include <vector>
#include <iostream>
#include "..\pid_controller.h"
#define DEG2RAD(x) ((x) * M_PI / 180.0f)


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

int main()
{
    const unsigned short n_POINTS = 1440;
    float wallAngleDeg = 30.0f;   // wall tilted 30° from forward
    float wallDistance = 2.0f;    // wall ~2m in front of sensor

    // Frame 1: stationary
    auto old_nodes = generateSlopedWallScan(n_POINTS, wallAngleDeg, wallDistance);

    // Frame 2: optionally rotated by 0° (no real motion)
    auto new_nodes = generateSlopedWallScan(n_POINTS, wallAngleDeg, wallDistance,
                                            /*sensorRotationDeg=*/0.0f,
                                            /*sensorTranslation=*/{0.0f, 0.0f});

    // Print subset for debugging
    for (int i = 0; i < n_POINTS; i += 30)
    {
        std::cout << i << " angle=" << old_nodes[i].angle * 180 / M_PI
                  << " dist=" << old_nodes[i].dist << std::endl;
    }
	float std;
	float angleDiff;
	ArcNode new_nodesa[1440];
	ArcNode new_nodesb[1440];
	std::copy(new_nodes.begin(), new_nodes.end(), new_nodesa); 
	std::copy(old_nodes.begin(), old_nodes.end(), new_nodesb); 
		Point2D diff = {0,0};
	while(true)
	{
		diff = sum_difference(new_nodesa,new_nodesb,-1,1,diff,angleDiff,std,angleDiff);
		diff.X +=0;
		//printf("%.2f",diff);
	}
    return mag(diff);
}