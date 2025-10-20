#include "ded.h"
#include "AngleTree.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
using namespace std;

DedState dedState;
MoveState moveState;
int moveMagnitude;
AngleNode tree = { 0,0,NULL,NULL,0 };
int dataCount;
#define MOVE_SCAN_ANGLE (15*M_PI/180)
#define ROT_SCAN_ANGLE (180*M_PI/180)
#define M_2PI M_PI*2

#define N_POINTS  1440
#define RESOLUTION (M_2PI/N_POINTS) //rad
 
double kp = 0.001;
double ki = kp / (5 * RESOLUTION);
double kd = 0.1 * kp * RESOLUTION;
double umax = M_PI * 2;
double dtheta = RESOLUTION;
double Tt = 5 * RESOLUTION;
double deadband = 3;
double integral = 0.0;
double prev_error = 0.0;
Wall wall{};

AngleNode allNodes[N_POINTS];
int allNodeCount;
double currentRange = 0;
/// <summary>
/// fix angles. because of 0-360 range. Todo, move into helper file
/// </summary>
/// <param name="angle"></param>
/// <returns></returns>
double fix(double angle)
{
	while (angle < 0)
	{
		angle += M_PI * 2;
	}
	while (angle > M_PI * 2)
	{
		angle -= M_PI * 2;
	}
	return angle;
}
/// <summary>
/// fix angles. because of 0-360 range. Todo, move into helper file
/// </summary>
/// <param name="angle"></param>
/// <returns></returns>
double fixForAddition(double angle)
{
	if (angle < M_PI * 2)
	{
		angle += M_PI * 2;
	}
	
	return angle;
}
/// <summary>
/// scan and insert data
/// </summary>
/// <param name="nodes"></param>
/// <param name="nodeCount"></param>
void scan(sl_lidar_response_measurement_node_hq_t* nodes, size_t nodeCount)
{
	for (int i = 0; i < nodeCount; i++)
	{
		double angle = (nodes[i].angle_z_q14 * 90.f) / 16384.f * M_PI / 180;
		double dist = nodes[i].dist_mm_q2 / 4.0f;
		if (dist > 0)
		{
			int result = InsertAngle(&tree, angle, dist);
			if (result == 0)
			{
				printf("insert failed [angle] %.f6 [dist] %.f6 ", angle, dist);
			}
		}
	}
}
/// apple the function to all of the nodes with an angle filter
template <typename FilterFn>
double filterApply( sl_lidar_response_measurement_node_hq_t* nodesIn,size_t nodesInCount, double angleLeft, double angleRight,double offset, FilterFn func, double maxRange )
{
	angleLeft = fix(angleLeft);
	angleRight = fix(angleRight);
	double total=0;
	for (int i = 0; i < nodesInCount; i++)
	{
		double angle = (nodesIn[i].angle_z_q14 * 90.f) / 16384.f * M_PI / 180 + offset;
		double dist = nodesIn[i].dist_mm_q2 / 4.0f;
		if (dist < maxRange)
		{
			if (angle < 0)
			{
				angle += M_PI * 2;
			}
			if (angle > M_PI * 2)
			{
				angle -= M_PI * 2;
			}

			if (angleLeft > M_PI_2) //look at >180
			{
				if (angle < angleLeft && angle > angleRight)
				{
					continue;
				}
			}
			else
			{
				if (!(angle > angleLeft && angle < angleRight))
					continue;
			}
			double v = func(nodesIn + i);
			total += v;
		}
	}
	return total;
}
/// <summary>
/// apple the function to all of the nodes
/// </summary>
/// <typeparam name="FilterFn"></typeparam>
/// <param name="nodesIn"></param>
/// <param name="nodesInCount"></param>
/// <param name="func"></param>
/// <returns></returns>
template <typename FilterFn>
double apply(sl_lidar_response_measurement_node_hq_t* nodesIn, size_t nodesInCount,  FilterFn func)
{
	double total=0;
	for (int i = 0; i < nodesInCount; i++)
	{		
		double dist = nodesIn[i].dist_mm_q2 / 4.0f;
		if (dist < 500)
		{
			total += func(nodesIn + i);
		}

	}
	return total;
}

/// <summary>
/// consult uni notes
/// </summary>
/// <param name="y"></param>
/// <returns></returns>
double PIDUpdate(double y)
{
	double e = -y;
	if (abs(y) <= deadband)
	{
		e = 0.0;
	}
	double de_dtheta = (e - prev_error) / dtheta;

	double	u_un = kp * e + ki * integral + kd * de_dtheta;
	
	double u = max(-umax, min(u_un, umax));

	//	# anti - windup correction;
	integral += dtheta * (e + (u - u_un) / Tt);

	prev_error = e;
	printf("[error] %.6f [new input] %.6f\n", y, u);
	return u;
}

/// <summary>
/// the move function first checks the command and sets the viewing angle
/// if we are rotating, set the viewing angle +- the target wall
/// run a PID loop to find the new angle
/// </summary>
/// <param name="nodes"></param>
/// <param name="nodeCount"></param>
/// <returns></returns>
Transform move(sl_lidar_response_measurement_node_hq_t* nodes, size_t nodeCount)
{
	static double averageRad;
	static double sensorOffset=0;
	static double averageReading;
	static double prevResult = 0.0;
	double angleLeft=0, angleRight=0;
	switch (moveState)
	{
	case F:
		angleLeft = M_2PI - MOVE_SCAN_ANGLE/2;
		angleRight = MOVE_SCAN_ANGLE / 2;
		break;
	case B:

		angleLeft = M_2PI/2 - MOVE_SCAN_ANGLE / 2;
		angleRight = M_2PI / 2 + MOVE_SCAN_ANGLE / 2;
		break;
	case P:
		angleLeft = M_PI_2*3 -   MOVE_SCAN_ANGLE / 2;
		angleRight = M_PI_2 * 3  + MOVE_SCAN_ANGLE / 2;
		break;
	case S:
		angleLeft = M_PI_2  - MOVE_SCAN_ANGLE / 2;
		angleRight = M_PI_2 + MOVE_SCAN_ANGLE / 2;
		break;
	case L:
		break;
	case R:
		break;
	default:
		break;
	}
	double total_x = 0;
	double total_y = 0;
	double total_rad = 0;
	int matches = 0;
	auto translateFn = [&](sl_lidar_response_measurement_node_hq_t* node) {
			double angle = (node->angle_z_q14 * 90.f) / 16384.f * M_PI / 180;
			double dist = node->dist_mm_q2 / 4.0f;
			if (dist != 0)
			{
				Transform t = FindNode(angle, dist, &tree,allNodes, allNodeCount);

				//out total translation
				total_x += t.Point.X;
				total_y += t.Point.Y;
				matches++;				
			}
			return 0.0;
		};
	auto rotateFn = [&](sl_lidar_response_measurement_node_hq_t* node) {
		double so = sensorOffset;
		double angle = (node->angle_z_q14 * 90.f) / 16384.f * M_PI / 180 + so;
		if (angle < 0)
		{
			angle += M_PI * 2;
		}
		else if (angle > M_PI * 2)
		{
			angle -= M_PI * 2;
		}

		double dist = node->dist_mm_q2 / 4.0f;
		if (dist != 0)
		{
			Transform t = FindNode(angle, dist, &tree,allNodes, allNodeCount);				

			double norm = 0;

			matches++;
			double total = 0;
			if (t.Angle > 0 && t.Angle <= M_PI)
			{
				double v = t.Point.Y * M_PI / 2 - abs(M_PI / 2 - t.Angle);
				total += v;
			}
			else
			{
				double v = -t.Point.Y * M_PI / 2 - abs(M_PI * 3 / 2 - t.Angle);
				total += v;
			}

			if (t.Angle > M_PI / 2 && t.Angle <= M_PI * 3 / 2)
			{
				double v = t.Point.X * M_PI - abs(M_PI - t.Angle);
				return total + v;
			}
			else
			{
				double angle = t.Angle;
				if (angle > M_PI * 3 / 2)
				{
					angle -= M_PI * 2;
				}
				double v = -t.Point.X * abs(t.Angle);
				return total + v;
			}			
		}
		};
	if (angleLeft != angleRight)//non rotational
	{	
		filterApply( nodes, nodeCount,angleLeft, angleRight,0, translateFn,2000);
		
		double averageX = total_x / matches;
		double averageY = total_y / matches;

		printf("[averageX] %.6f [averageY] %.6f\n",averageX,averageY);
		return { averageX,averageY,0,0 };
	}
	else
	{
		double midAngle = fix( (fixForAddition(wall.Start.Angle) + fixForAddition( wall.End.Angle)) / 2) ;
		angleLeft = midAngle - ROT_SCAN_ANGLE / 2;
		angleRight = midAngle +  ROT_SCAN_ANGLE / 2;		
		
		double gradient=0.01;
		double totalError = 0.0;
		int count = 20;
		while (true)
		{
			matches = 0;
			double res0 = filterApply(nodes, nodeCount, angleLeft,angleRight,sensorOffset, rotateFn,currentRange);
			if (matches == 0)
			{
				break					;
			}
			res0 = res0 / matches;
			if (abs(res0) < deadband)
				break;
			if (isnan(res0))
				return { 0,0,averageRad,0 };
			
			sensorOffset = PIDUpdate(res0);
	
		}

		printf("[averageRad] %.6f [sensorOffset] %.6f [matches] %i\n", averageRad, sensorOffset, matches);
	
		return { 0,0,averageRad,0 };
	}
}
void DataIn(sl_lidar_response_measurement_node_hq_t* nodes, size_t nodeCount)
{
	
	string command = "";
	int mag1;
	int mag2;
	double wallDist = 0;
	switch (dedState)
	{
	case STARTUP:
		if (dataCount > 50)
		{
			dedState = SCAN;
			dataCount = 0;
		}
		break;
	case SCAN:
		scan(nodes, nodeCount);
		if (dataCount > 20)
		{
			dedState = WAITING;
			dataCount = 0;
		}
		break;
	case MOVING:
		move(nodes, nodeCount);
		break;
	case WAITING:
		
		allNodeCount = Flatten(&tree, allNodes);

		wall = FindFlatSurface(allNodes, allNodeCount);
		wallDist = (wall.End.Dist * wall.End.Angle + wall.Start.Dist * wall.Start.Angle) / (wall.End.Angle + wall.Start.Angle);
		currentRange = wallDist*2;

		do {
			printf("Enter command, e.g. F5, L20:");
			cin >> command;
		} while (command.length() < 2);
		
		switch (command[0])
		{
		case 'F':
			moveState = F;
			break;
		case 'B':
			moveState = B;
			break;
		case 'S':
			moveState = S;
			break;
		case 'P':
			moveState = P;
			break;
		case 'L':
			moveState = L;
			break;
		case 'R':
			moveState = R;
			break;
		default:
			printf("unrecognized cmd %c", command[0]);
			return;
		}

		mag1 = command[1] - 48;
		mag2 = command.length() > 2 ? command[2] - 48 : 0;
		moveMagnitude = mag2 * 10 + mag1;


		printf("ready to perform move %c %i ...", command[0],moveMagnitude);

		dedState = MOVING;
		break;
	default:
		break;
	}

	dataCount++;
}