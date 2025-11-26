
#include "..\headers\Ded.h"
#include "..\headers\NodeServices.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "..\headers\sample_filter.h"
#include "..\headers\pid_controller.h"
#include "..\headers\arduino_ext.h"
#include "..\headers\wifi_mpi.h"

#include <Arduino.h>
using namespace std;

#define SCAN_COUNT 30

DedState dedState = STARTUP;


ArcNode nodes_ref[N_POINTS];

ArcNode* GetRefNodes()
{
	return nodes_ref;
}


__attribute__((section(".ccmram"))) ArcNode nodes_cur[N_POINTS]={0};

short dataCount=0;




float currentRange = 0;

Wall prevWalls[3];


float velocity=0;
float angular_velocity = 0;



float totalScanError=0;

void Reset()
{
	Clear(nodes_ref,N_POINTS);
	Clear(nodes_cur,N_POINTS);
	dataCount = 0;
}
void runFilter(ArcNode* src,ArcNode* dst)
{
	//printf("f start");
	float samples[N_POINTS];
	for(unsigned short i =0;i<N_POINTS;i++)
	{
		samples[i] = src[i].dist;//sqrt(src[i].dist)/30;//sqrt because distant objects have more error	
	//	printf("%.8f",src[i].dist);
	}
	Samples::Filter::FilterSamples(samples);

	for(unsigned short i =0;i<N_POINTS;i++)
	{	
		//if(src[i].angle)
	//	{
			dst[i].angle = src[i].angle;					
			dst[i].dist = samples[i];//pow(samples[i]*30,2);	
			dst[i].point = toPoint2D(dst[i].angle,dst[i].dist );
	//	}
		//printf("%.8f",dst[i].dist);
	}
}
void FindWalls(ArcNode* nodes,Wall* walls)
{
	
	
	FindBestFlatSurface(nodes, N_POINTS,walls);	

}
void getFilteredSnapshot(ArcNode* nodes)
{
	runFilter(nodes_cur,nodes);
}
void GetNewNodes(ArcNode* dst )
{
	getFilteredSnapshot(dst);
}


bool TryMakeRefScan()
{
	if(dataCount < 50)
		return false;
	int zeros = CountZeros(nodes_cur);
	if(zeros > 130)
	{
		WIFI_MPI::Printf("> INCOMPETE SCAN DETECTED, %.i ZEROS, CONTINUEING-SCAN",zeros);	
		return false;
	}
	else if(zeros>0)
	{		
		WIFI_MPI::Printf("> PATCHING HOLES",zeros);	
		InterpolateMissingNodes(nodes_cur);
	}
	
	WIFI_MPI::Printf("> ENVIRONMENT ANALYSIS COMPLETE");
	
	getFilteredSnapshot(nodes_ref);
	FindWalls(nodes_ref,prevWalls);
	PrintNodes(nodes_ref);
	dedState = EST_TRANS;
	return true;
}


Point2D avg_pos={0};
float avg_yaw=0;
float expected_rotate=0;
void RotateCompleted(float rotate)
{
	expected_rotate = rotate;
	avg_yaw = rotate;
	avg_pos = {0};
    
	dataCount = 0;
	dedState = EST_ROT;
	totalScanError = 0;
	Clear(nodes_cur,N_POINTS);
	#ifndef TESTING
  	printf("> ESTIMATING ROTATION"); 
	#endif

}


bool portFirst=true;
bool fwdFirst=true;
void SetPos(const Point2D& p)
{
	avg_pos = p;
	avg_yaw = 90 * M_PI/180;
}
Point2D GetPos()
{
	return avg_pos;
}

float GetYaw()
{
	return avg_yaw;
}
void RefUpdate()
{	
	getFilteredSnapshot(nodes_ref);
	FindWalls(nodes_ref,prevWalls);
	serialPrintf("ref updated!");
}
Transform ResolveXY(ArcNode* nodes,float& confidence )
{
	Point2D heading = normalize(Point2D({-avg_pos.X,-avg_pos.Y}));
	// const char nReadings=10;
	// Point2D total;
	// Point2D prev = avg_pos;
	// for(int i=0;i<nReadings;i++)
	// {
		float total_residual;
		bool success;
		//do
		//{
			success = find_pos(nodes,nodes_ref,avg_pos,avg_yaw,total_residual,heading);
			serialPrintf("total r:%.6f",total_residual);
		//} while (total_residual > 15000);
		
	// 	total = add(total,avg_pos);
	// 	avg_pos = mul(add(prev,avg_pos),0.5f);
	// }
	// avg_pos = div(total,10.0f);
	confidence = 25000 / total_residual;
	return {avg_pos,avg_yaw,mag(avg_pos),false};
}

Transform ResolveRotation(ArcNode* nodes,float& confidence)
{

	float total_residual;	
	find_yaw(nodes,nodes_ref,avg_pos,avg_yaw,total_residual);
		
	printf("X:%.2f, Y:%.2f, YAW:%.2f",avg_pos.X,avg_pos.Y,avg_yaw);
	confidence = 25000 / total_residual;
	return {avg_pos,avg_yaw,mag(avg_pos),false};	
}

Transform EstimateTranslation(float x, float y, float yaw,float& confidence)
{
	avg_pos = {-x,-y};
	avg_yaw = -yaw;
	ArcNode nodes_now[N_POINTS];
	getFilteredSnapshot(nodes_now);
	return ResolveXY(nodes_now,confidence );
}


Transform EstimateRotation(float x, float y, float yaw,float& confidence)
{
	avg_pos = {-x,-y};
	avg_yaw = -yaw;
	ArcNode nodes_now[N_POINTS];
	getFilteredSnapshot(nodes_now);
	return ResolveRotation(nodes_now,confidence);
}


void DataIn(LidarScanNormalMeasureRaw* nodes, unsigned short nodeCount)
{		
	//Serial.println("data in");
//	Serial.print(nodeCount);
	dataCount++;
	 totalScanError += read_scan(nodes, nodes_cur, nodeCount);	
}