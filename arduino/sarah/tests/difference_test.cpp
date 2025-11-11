#include <cmath>
#include <vector>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include "..\headers\pid_controller.h"
#define DEG2RAD(x) ((x) * M_PI / 180.0f)

#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>



// Small noise helper (meters)
float addNoise(float val, float noiseLevel = 0.25f)
{
    float r = ((float)rand() / RAND_MAX - 0.5f) * 2.0f; // [-1,1]
    return val + r * noiseLevel;
}

// Rotate a vector by cos/sin
Point2D rotateVec(const Point2D& v, float cosR, float sinR)
{
    return { v.X * cosR - v.Y * sinR,
             v.X * sinR + v.Y * cosR };
}

// Intersect ray (origin, dir) with axis-aligned rectangle defined by rectX,rectY (min corner) and rectW,rectH.
// Returns true if hit and sets hit point (world coords). t is positive along dir.
bool intersectRectangle(const Point2D& origin, const Point2D& dir,
                        float rectX, float rectY, float rectW, float rectH,
                        Point2D& outHit, float& outT)
{
    float tMin = 1e9f;
    bool hitSomething = false;

    // vertical sides: x = rectX and x = rectX+rectW
    for (int i = 0; i < 2; ++i)
    {
        float sideX = rectX + i * rectW;
        if (fabsf(dir.X) < 1e-8f) continue;
        float t = (sideX - origin.X) / dir.X;
        if (t <= 0) continue;
        float yHit = origin.Y + dir.Y * t;
        if (yHit >= rectY - 1e-6f && yHit <= rectY + rectH + 1e-6f)
        {
            if (t < tMin) { tMin = t; outHit = {sideX, yHit}; hitSomething = true; }
        }
    }

    // horizontal sides: y = rectY and y = rectY+rectH
    for (int i = 0; i < 2; ++i)
    {
        float sideY = rectY + i * rectH;
        if (fabsf(dir.Y) < 1e-8f) continue;
        float t = (sideY - origin.Y) / dir.Y;
        if (t <= 0) continue;
        float xHit = origin.X + dir.X * t;
        if (xHit >= rectX - 1e-6f && xHit <= rectX + rectW + 1e-6f)
        {
            if (t < tMin) { tMin = t; outHit = {xHit, sideY}; hitSomething = true; }
        }
    }

    if (hitSomething) outT = tMin;
    return hitSomething;
}
bool intersectCircle(const Point2D& origin, const Point2D& dir,
                     const Point2D& center, float radius,
                     Point2D& outHit, float& outT)
{
    // Vector from ray origin to circle center
    float ox = origin.X - center.X;
    float oy = origin.Y - center.Y;

    // Solve quadratic for ||origin + t*dir - center||^2 = r^2
    float a = dir.X * dir.X + dir.Y * dir.Y;
    float b = 2.0f * (ox * dir.X + oy * dir.Y);
    float c = ox * ox + oy * oy - radius * radius;

    float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f)
        return false; // no intersection

    float sqrtDisc = sqrtf(disc);
    float t1 = (-b - sqrtDisc) / (2.0f * a);
    float t2 = (-b + sqrtDisc) / (2.0f * a);

    // we only care about the first positive intersection
    float t = (t1 > 1e-4f) ? t1 : ((t2 > 1e-4f) ? t2 : -1.0f);
    if (t < 0.0f)
        return false;

    outT = t;
    outHit = { origin.X + dir.X * t, origin.Y + dir.Y * t };
    return true;
}

// Fixed environment scene (HVH N-shape + extras). All in world coords (meters).
// This function finds the closest intersection point between ray (origin,dir) and env.
Point2D rayIntersectionEnvironment(const Point2D& origin, const Point2D& dir)
{
    float tMin = 1e9f;
    Point2D hit = {0.0f, 0.0f};
    Point2D tempHit; float tempT;

    // --- Top horizontal wall (left) ---
    // y = yTop, x in [xTopStart, xTopEnd]
    const float yTop = 5.0f, xTopStart = -3.0f, xTopEnd = 0.0f;
    if (intersectRectangle(origin, dir, xTopStart, yTop, xTopEnd - xTopStart, 0.001f, tempHit, tempT))
    {
        if (tempT < tMin) { tMin = tempT; hit = tempHit; }
    }

    // --- Vertical middle wall ---
    const float xVert = 0.0f, yVertStart = 2.0f, yVertEnd = 5.0f;
    if (intersectRectangle(origin, dir, xVert, yVertStart, 0.001f, yVertEnd - yVertStart, tempHit, tempT))
    {
        if (tempT < tMin) { tMin = tempT; hit = tempHit; }
    }

    // --- Bottom horizontal wall (right) ---
    const float yBottom = 2.0f, xBotStart = 1.0f, xBotEnd = 4.0f;
    if (intersectRectangle(origin, dir, xBotStart, yBottom, xBotEnd - xBotStart, 0.001f, tempHit, tempT))
    {
        if (tempT < tMin) { tMin = tempT; hit = tempHit; }
    }

    // --- Extra rectangle (example) ---
    const float rectX = -2.0f, rectY = 3.0f, rectW = 0.8f, rectH = 1.2f;
    if (intersectRectangle(origin, dir, rectX, rectY, rectW, rectH, tempHit, tempT))
    {
        if (tempT < tMin) { tMin = tempT; hit = tempHit; }
    }

    // --- Circles ---
    struct Circle { Point2D c; float r; };
    Circle circles[3] = { {{1.5f,4.0f},0.4f}, {{-1.0f,1.0f},0.5f}, {{3.5f,0.5f},0.3f} };
    for (int i = 0; i < 3; ++i)
    {
        if (intersectCircle(origin, dir, circles[i].c, circles[i].r, tempHit, tempT))
        {
            if (tempT < tMin) { tMin = tempT; hit = tempHit; }
        }
    }

    // If nothing hit, return a very far point along ray
    if (tMin > 1e8f)
    {
        float farT = 10.0f;
        hit = { origin.X + dir.X * farT, origin.Y + dir.Y * farT };
    }

    // Add small positional noise (meters)
    hit.X = addNoise(hit.X);
    hit.Y = addNoise(hit.Y);

    return hit;
}

// Main generator: sensor rotation (deg) and sensor translation (meters).
// SensorTranslation is the sensor position in world coordinates (i.e., where the sensor moved to).
std::vector<ArcNode> generateEnvironmentScanFixed(unsigned short n_POINTS,
                                                  float sensorRotationDeg = 0.0f,
                                                  Point2D sensorTranslation = {0.0f, 0.0f})
{
    srand((unsigned int)time(nullptr));
    std::vector<ArcNode> nodes(n_POINTS);

    // Precompute cos/sin for sensor rotation
    float sensorRot = sensorRotationDeg * (M_PI / 180.0f);
    float cosR = cosf(sensorRot), sinR = sinf(sensorRot);

    // Sensor position in world coords (sensorTranslation)
    Point2D sensorPos = sensorTranslation; // sensorTranslation is world position of sensor

    for (int i = 0; i < n_POINTS; ++i)
    {
        // ray in sensor frame (unit direction)
        float theta = 2.0f * M_PI * float(i) / float(n_POINTS); // 0..2pi
        Point2D raySensor = { sinf(theta), cosf(theta) }; // as before (X=sin, Y=cos)

        // rotate ray into world frame according to sensor orientation
        Point2D rayWorld = rotateVec(raySensor, cosR, sinR);

        // Cast ray from sensorPos along rayWorld into the fixed environment
        Point2D hitWorld = rayIntersectionEnvironment(sensorPos, rayWorld);

        // Convert hit (already world coords) into sensor-local if you need,
        // but we return world coords as point (consistent with earlier approach)
        Point2D hitRel = { hitWorld.X - sensorPos.X, hitWorld.Y - sensorPos.Y };

        nodes[i].point = hitRel;
        nodes[i].dist  = sqrtf(hitRel.X*hitRel.X + hitRel.Y*hitRel.Y);
        
        nodes[i].angle = toAngle(i,1440);
    }

    return nodes;
}

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
        float theta = DEG2RAD((360.0f * i) / n_POINTS);
        float dirX = sinf(theta + sensorRot);
        float dirY = cosf(theta + sensorRot);

        float t_min = 1e6f;
        bool hit_found = false;
        Point2D p_rel = {0.0f, 0.0f};   // sensor-relative hit (dir * t)

        // --- Top horizontal wall (y = topWallY) ---
        if (fabsf(dirY) > 1e-6f)
        {
            float t = (topWallY - sensorTranslation.Y) / dirY;                // world: sensorY + t*dirY = topWallY
            if (t > 0.0f)
            {
                float x_world = sensorTranslation.X + dirX * t;              // world x of hit
                if (x_world >= topWallXStart && x_world <= topWallXEnd && t < t_min)
                {
                    t_min = t;
                    p_rel = {dirX * t, dirY * t}; // sensor-relative
                    hit_found = true;
                }
            }
        }

        // --- Vertical wall (x = verticalWallX) ---
        if (fabsf(dirX) > 1e-6f)
        {
            float t = (verticalWallX - sensorTranslation.X) / dirX;          // world: sensorX + t*dirX = verticalWallX
            if (t > 0.0f)
            {
                float y_world = sensorTranslation.Y + dirY * t;              // world y of hit
                if (y_world >= verticalWallYStart && y_world <= verticalWallYEnd && t < t_min)
                {
                    t_min = t;
                    p_rel = {dirX * t, dirY * t};
                    hit_found = true;
                }
            }
        }

        // --- Bottom horizontal wall (y = bottomWallY) ---
        if (fabsf(dirY) > 1e-6f)
        {
            float t = (bottomWallY - sensorTranslation.Y) / dirY;             // world: sensorY + t*dirY = bottomWallY
            if (t > 0.0f)
            {
                float x_world = sensorTranslation.X + dirX * t;
                if (x_world >= bottomWallXStart && x_world <= bottomWallXEnd && t < t_min)
                {
                    t_min = t;
                    p_rel = {dirX * t, dirY * t};
                    hit_found = true;
                }
            }
        }

        if (hit_found)
        {
            // addNoise expects world or relative? -- follow your sloped style (they add noise to sensor-relative p)
            p_rel.X = addNoise(p_rel.X);
            p_rel.Y = addNoise(p_rel.Y);

            float dist = mag(p_rel); // distance from sensor
            if (dist < 500.0f) // keep same gating as your sloped example
            {
                nodes[i].angle = toAngle(i, n_POINTS); // match your sloped use
                nodes[i].point = p_rel;                 // sensor-relative point
                nodes[i].dist = dist;
            }
        }
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
    float wallAngle = DEG2RAD(wallAngleDeg + sensorRotationDeg);
    float sensorRot = DEG2RAD(0);

    // Wall normal (points toward the sensor)
    float ny = cosf(wallAngle);
    float nx = sinf(wallAngle);

    // The wall equation: n · p = wallDistance
    // Ray: p = sensorTranslation + t * dir
    // => t = (wallDistance - n·sensorTranslation) / (n·dir)

    for (int i = 0; i < n_POINTS; i++)
    {
        float theta = DEG2RAD((360.0f * i) / n_POINTS);
        float dirX = sinf(theta + sensorRot);
        float dirY = cosf(theta + sensorRot);

        float denom = nx * dirX + ny * dirY;
        float t = 0;

        if (fabs(denom) > 1e-6f)
        {
            t = (wallDistance - (nx * sensorTranslation.X + ny * sensorTranslation.Y)) / denom;
        }
        else
        {
            t = 100.0f; // No hit (parallel)
        }

        Point2D p = {
           dirX * t,
            dirY * t};
        //p.X = addNoise(p.X);
        //p.Y = addNoise(p.Y);
        float dist =  mag(p);
        if(dist < 500)
        {
        nodes[i].angle = toAngle(i,n_POINTS);// fmodf(theta + 2 * M_PI, 2 * M_PI);
        nodes[i].point = p;
        nodes[i].dist =dist;
        }
    }

    return nodes;
}
void exportPointsToCSV(const std::vector<ArcNode>& nodes, const std::string& filename)
{
    std::ofstream file(filename,std::ios::trunc);
    file << "X,Y\n";
    for (const auto& n : nodes)
    {
        file << n.point.X << "," << n.point.Y << "\n";
    }
    file.close();
}

float testAngleFlat(float wallAngleDeg,float wallAngleDist,float yMovement,float yaw,int testnum,int searchSize=2)
{
    const unsigned short n_POINTS = N_POINTS;

    // Frame 1: stationary
  
     auto old_nodes =  generateSlopedWallScan(n_POINTS,wallAngleDeg,wallAngleDist,0,{0,0});
    //  exportPointsToCSV(old_nodes, "old_lidar_points.csv");
  
    auto new_nodes =  generateSlopedWallScan(n_POINTS,wallAngleDeg,wallAngleDist,yaw,{0,yMovement});
 //exportPointsToCSV(new_nodes, "new_lidar_points.csv");
    // Print subset for debugging
   
	float std;
	float angleDiff=0*M_PI/180;
	ArcNode new_nodesa[1440];
	ArcNode new_nodesb[1440];
	std::copy(new_nodes.begin(), new_nodes.end(), new_nodesa); 
	std::copy(old_nodes.begin(), old_nodes.end(), new_nodesb); 
		Point2D diff = {0,0};
    int iter = 1000;
    Point2D newdiff;
    float yawAsRadians = yaw*M_PI/180;
    float total_resid;
	while(iter>0)
	{
        float ad=0;
        float angle_diff_t=0;
		newdiff = sum_difference(new_nodesa,new_nodesb,diff,angleDiff,angle_diff_t,1.57,total_resid,searchSize);
        diff.Y += newdiff.Y / 10;
        diff.X += newdiff.X / 100;//  add(diff,div(newdiff,10.0f));
       
        angleDiff -= angle_diff_t/10;
        if(mag(newdiff) < 0.001 && fabs(angle_diff_t) < 0.001  )
        {
            break;
        }
		//printf("%.2f",diff);
        iter--;
        // if(mag(newdiff) < 0.1)
        // {
        //     break;
        // }
	}
     std::cout << "\n test starting " << testnum;
    float asDegrees = angleDiff * 180/M_PI;
    if(fabs(asDegrees + yaw) > 0.25)
    {
         std::cout << "\nangle failed     \n[input] " << yaw << " [output] " << asDegrees << " wall angle " << wallAngleDeg;
    }
    else
    {
      //   std::cout << "\nangle succeeeded \n[input]" << yawMovement << " [output]" << asDegrees;
    }
    diff.X =0; // the single slopy environment prevents us from determining x position so, it slides around
    if(fabs(fabs(yMovement) - mag(diff)) > 1)
    {
         std::cout << "\nmovement failed \n[input] " << yMovement << " [output] " << mag(diff) << " wall angle " << wallAngleDeg << " yaw " << yaw;
    }
    else
    {
     //    std::cout << "\nmovement succeeded \n[input] " << yMovement << " [output] " << diff.Y;
    }
    return mag(diff);
}

void move_and_rotate_test()
{
    float angle = -0.1;
    Point2D heading = {0,1};
    int index = toIndex(angle,N_POINTS);
    int testnum = 0;
    for(int wallAngle=0;wallAngle<70;wallAngle+=5)
    {         
        for(float yaw=-0;yaw<10;yaw+=0.55)
        {
            for(int move=20;move>0;move-=5)
            {
                float backward = testAngleFlat(wallAngle,100,-move,yaw,testnum++);
                float forward = testAngleFlat(wallAngle,100,move,yaw,testnum++);
            // std::cout <<fwd;
            }
        }
    }
}

void rotate_test()
{
    float angle = -0.1;
    int index = toIndex(angle,N_POINTS);
    int testnum = 0;
    for(int wallAngle=15;wallAngle<70;wallAngle+=5)
    {         
        for(float yaw=-10;yaw<10;yaw+=1.05)
        {
           testAngleFlat(wallAngle,100,-0,yaw,testnum++,3);        
            
        }
    }
}
int main()
{
    move_and_rotate_test();
   // rotate_test();
    return 0;
}