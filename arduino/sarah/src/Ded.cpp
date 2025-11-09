
#include "..\headers\Ded.h"
#include "..\headers\NodeServices.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "..\headers\sample_filter.h"
#include "..\headers\pid_controller.h"
#include "..\headers\arduino_ext.h"
#include <Arduino.h>
using namespace std;

#define SCAN_COUNT 30

DedState dedState = STARTUP;


__attribute__((section(".ccmram")))  ArcNode nodes_ref[N_POINTS];

ArcNode* GetRefNodes()
{
	return nodes_ref;
}


ArcNode nodes_cur[N_POINTS]={0};

uint8_t dataCount;




float currentRange = 0;

Wall prevWalls[3];


float velocity=0;
float angular_velocity = 0;



float totalScanError=0;


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

unsigned long prev_millis=0;

bool goFromScanningToEstimating()
{
	printf("> TOTAL SCAN NOISE:%.2f",totalScanError);
	int zeros = CountZeros(nodes_cur);
	if(zeros > 130)
	{
		printf("> INCOMPETE SCAN DETECTED, %.i ZEROS, CONTINUEING-SCAN",zeros);	
		return false;
	}
	else if(zeros>0)
	{		
		printf("> PATCHING HOLES",zeros);	
		InterpolateMissingNodes(nodes_cur);
	}
	// if(totalScanError>50000)
	// {				
	// 	printf("> SIGNIFICANT MOVEMENT DETECTED, RE-SCANNING");	
	// 	Clear(nodes_cur,N_POINTS);
	// 	dataCount = 0;
	// 	dedState = SCAN;
	// 	totalScanError = 0;
	// 	return false;
	// }
	#ifndef TESTING
	Serial.println("> ENVIRONMENT ANALYSIS COMPLETE");
	#endif
	
	dataCount = 0;
	
	getFilteredSnapshot(nodes_ref);
	FindWalls(nodes_ref,prevWalls);
	PrintNodes(nodes_ref);
	dedState = EST_TRANS;
		
	//drop to RECON
	prev_millis = millis();
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
void ResolveXY(ArcNode* nodes,float current_step_weight,bool move_x, bool move_y,unsigned long dt )
{
	if(!move_x && !move_y)
		return;
	
	Point2D prev_pos = avg_pos;
	find_pos(nodes,nodes_ref,avg_pos,avg_yaw);		
	
	float new_velocity =dt == 0 ? 0 : mag(div(sub(prev_pos ,avg_pos),dt));
	if(new_velocity > 1) //bs, thats 1 mm per millisecond
		return;
	velocity = new_velocity;
	
	printf("X:%.2f, Y:%.2f, velocity:%.2f",avg_pos.X,avg_pos.Y,velocity);
}

void ResolveRotation(ArcNode* nodes, float current_step_weight, bool rotate, unsigned long dt)
{

	if(!rotate)
		return;		

	float prev = avg_yaw;
	find_yaw(nodes,nodes_ref,avg_pos,avg_yaw);
	float new_angular_velocity =dt == 0 ? 0 : (prev -avg_yaw)/dt;
	if(new_angular_velocity > 1) //bs
		return;
	angular_velocity = new_angular_velocity;
		
	printf("YAW:%.6f",avg_yaw*180/M_PI);
	
}
float durationStopped =0;

void DataIn(LidarScanNormalMeasureRaw* nodes, unsigned short nodeCount,bool  move_x, bool move_y, bool rotate)
{	
	unsigned long  dt= 1;
	#ifndef TESTING
	unsigned long millis_now = millis();
	dt =  millis_now - prev_millis;
	prev_millis = millis_now;
	#endif
	totalScanError += read_scan(nodes, nodes_cur, nodeCount);

	switch (dedState)
	{
	case STARTUP:
		if (dataCount > 5)
		{
			
  			Serial.println("> INITIALIZING EYE FILTER"); 	
  			Samples::Filter::SetupFilter();
			Clear(nodes_cur,N_POINTS);
			dedState = SCAN;
			dataCount = 0;  
  			Serial.println("> EVALUATING ENVIRONMENT 0.0%"); 		
			
			totalScanError = 0;
		}
		break;
	case SCAN:
		
		if (dataCount > SCAN_COUNT)
		{			
  			if(!goFromScanningToEstimating())
			{
				return;
			}
		}
		else
		{			
			printf("> ANALYSING ENVIRONMENT... %.02f\%",pow(dataCount / (float)SCAN_COUNT,2)*100);
  		
			break;
		}
	case EST_TRANS:
	{		
		ArcNode nodes_now[N_POINTS];
		float current_step_weight = std::fmin(0.95,velocity*100+0.2);
		getFilteredSnapshot(nodes_now);

		if(rotate)
		{
			dedState = EST_ROT;
			getFilteredSnapshot(nodes_ref);
			break;
		}

		ResolveXY(nodes_now, current_step_weight,move_x,  move_y, dt );
	
		if(velocity <0.001 )
		{
			durationStopped += dt;
			if(durationStopped > 5000 && dt < 100 )
			{
				//Copy(nodes_now,nodes_ref);
				durationStopped = 0;
			}
		}
		return;
	}	
	case EST_ROT:
	{	
		ArcNode nodes_now[N_POINTS];
		Wall walls[3];
		float current_angular_step_weight =0.5;//min(0.95,angular_velocity*100+0.5);	
		getFilteredSnapshot(nodes_now);		
		FindWalls(nodes_now,walls);
		//we shouldnt be moving when doing this		
				
		ResolveRotation(nodes_now,current_angular_step_weight,rotate,dt);
		// if(angular_velocity == 0)
		// {
		// 	durationStopped += dt;
		// 	updateReferenceScan();
		// }
		if(move_x || move_y)
		{
			dedState = EST_TRANS;
			getFilteredSnapshot(nodes_ref);
			printf("switch to translation");
			return;
		}	
		return;
	}
	default:
		break;
	}

	dataCount++;
}