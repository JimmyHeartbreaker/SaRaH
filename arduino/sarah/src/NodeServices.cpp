

#include "..\headers\NodeServices.h"
#define _USE_MATH_DEFINES
#include "..\headers\arduino_ext.h"
#include <cstring>


/// <summary>
/// magnitude of a vector
/// </summary>
/// <param name="y"></param>
/// <param name="x"></param>
/// <returns></returns>



void FindBestFlatSurface(ArcNode* nodes, unsigned short nodeCount,Wall* walls)//always 3 walls
{
    int count =0;
    int get_count =3;
    //Serial.println("FindFlatSurfaces");
    unsigned short wallIndex = 0;
    Transform bestStart = { {0,0},0,0,1 };
    Transform bestEnd = { {0,0},0,0,1 };
    unsigned short bestPoints = 0;
    Wall bestWall;
    Transform previous = { {0,0},0,0,1 };
    Transform start = { {0,0},0,0,1 };
    Transform end = { {0,0},0,0,1 };
    
    bool trackingWall=false;
    unsigned short points = 0;
    
    for (unsigned short i = 0; i < nodeCount; i++)
    {  
        ArcNode* t1 = (nodes + i);
        if (!t1->dist)
            continue;

        float norm_y = cos(t1->angle);
        float norm_x = sin(t1->angle);
        float y = t1->dist * norm_y;
        float x = t1->dist * norm_x;
        if (!previous.IsEmpty)
        {
            float angleSize = fabs(t1->angle - previous.Angle);

            float t1dist = pow(pow(x, 2) + pow(y, 2), 0.5);
            float t2dist = pow(pow(previous.Point.X, 2) + pow(previous.Point.Y, 2), 0.5);
            float distanceBetween = pow(pow(x - previous.Point.X, 2) + pow(y - previous.Point.Y, 2), 0.5);
            float expectedDistance = sqrt(pow(t1dist, 2) + pow(t2dist, 2) - 2 * t1dist * t2dist * cos(angleSize));
            bool collinear = true;
            if (!start.IsEmpty)
            {
               
                float dist_sq = pow(start.Point.X - previous.Point.X, 2) +
                    pow(start.Point.Y - previous.Point.Y, 2);

                if (dist_sq == 0)
                {
                    collinear = true;
                }
                else
                {
                    float cross = fabs((start.Point.X - x) * (previous.Point.Y - y) - (start.Point.Y - y) * (previous.Point.X - x));
                    float collinear_val = fabs(cross) / sqrt(dist_sq);
                    collinear = collinear_val < 0.5;
                }
            }

            if (collinear && fabs(distanceBetween - expectedDistance) < 0.01)
            {
                if (start.IsEmpty)
                {
                    start.IsEmpty = false;
                    start.Point = { x,y };
                    start.Dist = t1->dist;
                    start.Angle = t1->angle;
                }
                trackingWall = true;
                points++;
            }
            else if (trackingWall)
            {
                end = previous;  
                
                if (points > bestPoints)
                {
                    bestWall = { start,end };
                    
                    walls[0] = walls[1];
                    walls[1] = walls[2];
                    walls[2] = bestWall;
                    count++;
                    
                    bestPoints = points;
                }
                else if(points>2 && count < get_count)
                {
                    walls[0] = {start,end };
                }
                points = 0;
                start = { 0,0,0,0,true };
                end = start;
                previous = start;
                trackingWall = false;
            }
        }
        previous = { {x,y}, t1->angle,t1->dist, false };

    }
}


/// <summary>
/// find closest node
/// </summary>
/// <param name="angle"></param>
/// <param name="dist"></param>
/// <param name="node"></param>
/// <param name="flatNodes"></param>
/// <param name="nodeCount"></param>
/// <returns></returns>

int FindClosestNode(float angle, float dist,   ArcNode* oldNodes,int searchSize)
{
    while (angle >= M_PIF * 2)
    {
        angle -= M_PIF * 2;
    }

    while (angle < 0)
    {
        angle += M_PIF * 2;
    }
    Point2D start = toPoint2D(angle, dist);
    
    float best_dist = 1000000;
    
    
    short index =  toIndex(angle ,N_POINTS);
    short bestIndex =index;
    ArcNode* bestNodePtr = oldNodes + index;
    //nothing found in tree, guess at the location in the list
  
    if (bestNodePtr->dist  )
    {
        Point2D bestToStart = sub(bestNodePtr->point,start);
        best_dist = mag(bestToStart);
        if (best_dist < 1)        
        {            
            return  index;
        }
    }
           // Serial.print("best_dist");
           // Serial.println(best_dist);
    for (int i = 1; i <searchSize; i++)
    {      
        int leftIndex = index - i;
        if (leftIndex < 0)
        {
            leftIndex += N_POINTS;
        }
        ArcNode* left = (oldNodes + leftIndex);

        if (left->angle )
        {       
            float leftDist = mag(sub(left->point,start));
            if (leftDist < best_dist)
            {
            //Serial.println("1");
                best_dist = leftDist;
                bestNodePtr = left;
                bestIndex = leftIndex;
            }
        }

        int rightIndex = index + i;
        if (rightIndex >= N_POINTS)
        {
            rightIndex -= N_POINTS;
        }
        ArcNode* right = (oldNodes + rightIndex);

       
        if (right->angle )
        {
            float rightDist = mag(sub(right->point,start));
            if (rightDist < best_dist)
            {  
                //Serial.println("2");
                best_dist = rightDist;
                bestNodePtr = right;
                bestIndex = rightIndex;
            }
        }
    }

    return bestIndex;
}




#define printf(...) serialPrintf(__VA_ARGS__)
void PrintNodes(ArcNode* nodes)
{
    for(int i =0;i<N_POINTS;i++)
    {
       // serialPrintf("%i,%.06f,%.06f",i,nodes[i].dist,nodes[i].angle);
    }
}
void InterpolateMissingNodes(ArcNode* nodes)
{
    const float ANG_STEP = M_2PI / N_POINTS;
    for(int i=0; i<N_POINTS; ++i){
    if(nodes[i].dist <= 0){
        nodes[i].angle = i*ANG_STEP;
        nodes[i].dist = (nodes[i-1].dist + nodes[(i+1)%N_POINTS].dist)/2.0f;
    }
}
}
int CountZeros(ArcNode* nodes)
{
    int zeros=0;
    for(int i =0;i<N_POINTS;i++)
    {
        if(!nodes[i].dist)
        {
            zeros++;
        }
    }
    return zeros;
}
void CalcPoints(ArcNode* nodes)
{
    for (int i = 0; i < N_POINTS; i++)
    {			
        if(nodes[i].angle)
            nodes[i].point = toPoint2D(nodes[i].angle,nodes[i].dist);
    }
}
void Clear(ArcNode* stack, unsigned short nodeCount)
{
    for (int i = 0; i < nodeCount; i++)
    {
        ArcNode* node = stack + i;     
        (node)->dist = 0;
        (node)->angle = 0;
        (node)->point = {0,0};
    }
}


void Copy(ArcNode* src, ArcNode* dst)
{
    std::memcpy(dst,src,sizeof(ArcNode)* N_POINTS);
    
}