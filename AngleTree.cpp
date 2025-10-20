#include "AngleTree.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "Ded.h"
#define M_2PI M_PI*2

#define RESOLUTION M_2PI/1440 //rad
#define SCAN_DIST_TOL 5 //mm

/// <summary>
/// magnitude of a vector
/// </summary>
/// <param name="y"></param>
/// <param name="x"></param>
/// <returns></returns>
double mag(double y, double x)
{
    return pow(pow(y, 2) + pow(x, 2), 0.5);
}
int findNode(double angle, double dist,  AngleNode* node)
{   
    if (node->dist != 0)
    {
        if (abs(node->angle - angle) < RESOLUTION)
        {
            return node->index;              
        }
        else
        {                
            return -1;
        }
    }

    if (angle > node->angle && node->right)//we have an angle, initially 180
    {            
        return findNode(angle, dist,   node->right);
          
    }
    else if(node->left)
    {
        return findNode(angle, dist,  node->left);           
    }   
    return -1;
}
/// <summary>
/// flattens the tree so its easier to walk up and down
/// </summary>
/// <param name="node"></param>
/// <param name="stack"></param>
/// <param name="i"></param>
/// <returns></returns>
int flatten(AngleNode* node, AngleNode* stack, int i)
{
    if (node->dist)
    {
        node->index = i;
        stack[i] = *node;

        return 1;
    }
    int count = 0;
    if (node->left)
    {
        count += flatten(node->left, stack, i);
        i += count;
        
    }
    if (node->right)
    {
        count += flatten(node->right, stack, i);
        
    }
    return count;
}


Wall FindFlatSurface(AngleNode* nodes, int nodeCount)
{
    
    Transform bestStart = { {0,0},0,0,1 };
    Transform bestEnd = { {0,0},0,0,1 };
    int bestPoints = 0;

    Transform previous = { {0,0},0,0,1 };
    Transform start = { {0,0},0,0,1 };
    Transform end = { {0,0},0,0,1 };
    
    bool trackingWall=false;
    int points = 0;
    for (int i = 0; i < nodeCount; i++)
    {
        int offsetIndex = (i + nodeCount / 2) % nodeCount;
        AngleNode* t1 = nodes + offsetIndex;
        if (!t1->dist)
            continue;

        double norm_y = cos(t1->angle);
        double norm_x = sin(t1->angle);
        double y = t1->dist * norm_y;
        double x = t1->dist * norm_x;
        if (!previous.IsEmpty)
        {
            double angleSize = abs(t1->angle - previous.Angle);

            double t1dist = pow(pow(x, 2) + pow(y, 2), 0.5);
            double t2dist = pow(pow(previous.Point.X, 2) + pow(previous.Point.Y, 2), 0.5);
            double distanceBetween = pow(pow(x - previous.Point.X, 2) + pow(y - previous.Point.Y, 2), 0.5);
            double expectedDistance = sqrt(pow(t1dist, 2) + pow(t2dist, 2) - 2 * t1dist * t2dist * cos(angleSize));
            bool collinear = true;
            if (!start.IsEmpty)
            {
                double collinear_val = abs((start.Point.X - x) * (previous.Point.Y - y) - (start.Point.Y - y) * (previous.Point.X - x));
                collinear = collinear_val < 1;
            }

            if (collinear && abs(distanceBetween - expectedDistance) < 0.01)
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
                    bestPoints = points;
                    bestStart = start;
                    bestEnd = end;
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

    return { bestStart,bestEnd };
}

int insertAngle( AngleNode* node, double angle, double dist, double leftEdge, double rightEdge)
{
    if (node->dist == 0 && node->angle == 0 && node->left == NULL && node->right == NULL) //we are a leaf and empty
    {
        node->angle = angle;
        node->dist = dist;
        return 1;
        //done
    }
    else
    {

        if (node->dist != 0) //this was leaf
        {
            if (abs(node->angle - angle) < RESOLUTION)
            {
              //  if (abs(node->dist - dist) < SCAN_DIST_TOL)
              //  {
                    node->dist = (node->dist * node->angle + dist * angle) / (node->angle + angle);
                    node->angle =  angle;
                    return 1;
             //   }
             //   return 0;    
            }
            else
            {
                double current_dist = node->dist;
                node->dist = 0;
                double current_angle = node->angle;
                node->angle = 0;
                node->right = new AngleNode();
                node->left = new AngleNode();
                return insertAngle(node,current_angle, current_dist,  leftEdge, rightEdge);
            }
        }

        double midpoint = leftEdge + (rightEdge - leftEdge) / 2;
        node->angle = midpoint;
       

        if (angle > node->angle)
        {
            return insertAngle(node->right,angle, dist, midpoint, rightEdge );            
        }
        else
        {
            return insertAngle(node->left,angle, dist,  leftEdge, midpoint );
        }
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

Transform FindNode(double angle, double dist,  AngleNode* node, AngleNode* flatNodes,int nodeCount)
{
    if (angle > M_PI * 2)
    {
        angle -= M_PI * 2;
    }
    else if (angle < 0)
    {
        angle += M_PI * 2;
    }
    double norm_y = cos(angle);
    double norm_x = sin(angle);
    double start_y = dist * norm_y;
    double start_x = dist * norm_x;
    double best_y=100000;
    double best_x=100000;
    double best_dist = 1000000;
    int index = findNode(angle, dist,  node);
    AngleNode* bestNode;
    if (index != -1)
    {
        bestNode = flatNodes + index;

        norm_y = cos(bestNode->angle);
        norm_x = sin(bestNode->angle);
        best_y = bestNode->dist * norm_y;
        best_x = bestNode->dist * norm_x;

        best_dist = mag(best_y - start_y, best_x - start_x);
        if (best_dist < 2)
        {
            return { {best_x - start_x,best_y - start_y},bestNode->angle,bestNode->dist };
        }
    }
    else
    {
        //nothing found in tree, guess at the location in the list
        bestNode = flatNodes + int((angle / (2*M_PI)) * nodeCount);
    }
    for (int i = 1; i < 1440 / 4; i++)
    {      
        AngleNode* left = bestNode - i;
        int leftIndex = index - i;
        if (leftIndex < 0)
        {
            left += nodeCount;
        }
        AngleNode* right = bestNode + i;
        int rightIndex = index + i;
        if (leftIndex >= nodeCount)
        {
            right -= nodeCount;
        }
        double norm_y = cos(left->angle);
        double norm_x = sin(left->angle);
        double left_y = left->dist * norm_y;
        double left_x = left->dist * norm_x;

        double leftDist = mag(left_y - start_y, left_x - start_x);
        if (leftDist < best_dist)
        {
            best_dist = leftDist;
            bestNode = left;
            best_x = left_x;
            best_y = left_y;
        }
        norm_y = cos(right->angle);
        norm_x = sin(right->angle);
        double right_y = right->dist * norm_y;
        double right_x = right->dist * norm_x;
        double rightDist = mag(right_y - start_y, right_x - start_x);
        if (rightDist < best_dist)
        {
            best_dist = rightDist;
            bestNode = right;
            best_x = right_x;
            best_y = right_y;
        }
    }

    return { {best_x-start_x,best_y-start_y},bestNode->angle,bestNode->dist };
}

int InsertAngle(AngleNode* tree, double angle, double dist)
{
	return insertAngle(tree,angle,dist,0, M_2PI);
}


int Flatten(AngleNode* node, AngleNode* stack)
{
    return flatten(node, stack,0);
}