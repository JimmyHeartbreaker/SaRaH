
#include "ded.h"
#include "NodeServices.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "sample_filter.h"
#include "pid_controller.h"
#include "arduino_ext.h"
using namespace std;

#define SCAN_COUNT 20

DedState dedState = STARTUP;


ArcNode nodes_prev[N_POINTS]={0};
ArcNode nodes_cur[N_POINTS]={0};

uint8_t dataCount;




float currentRange = 0;

Wall prevWalls[3];
Wall walls[3];


float velocity=0;
float angular_velocity = 0;
/// <summary>
/// scan and insert data
/// </summary>
/// <param name="nodes"></param>
/// <param name="nodeCount"></param>
float read_scan(LidarScanNormalMeasureRaw* nodes,ArcNode* dst, unsigned short nodeCount,float new_weight=0.5f,bool calculatePoints=false)
{
//	printf("new_weight:%.2f",new_weight);
	float totalDiff=0;
	for (int i = 0; i < nodeCount; i++)
	{
		float angle = (nodes[i].angle_z_q6 / 128.0f) * M_PI_F / 180.0f;
		while(angle>= M_2PI )
		{
			angle -= M_2PI;
		}
		float dist = nodes[i].dist_mm_q2 / 4.0f ;	
		dist = pow(dist, (1- (0.06*max(0,150 - dist))/150));
		if (dist > 0  )
		{
		
			unsigned short index =toIndex(angle);
			if(!dst[index].dist)
			{
				dst[index].angle =   (index / (float)N_POINTS * M_2PI);
				dst[index].dist = dist;
				if(calculatePoints)
				{
					dst[index].point = toPoint2D(dst[index].angle,dist);
				}
			}
			else
			{
				totalDiff += abs(dst[index].dist- dist);
				
				dst[index].dist = (dst[index].dist * (1-new_weight)+ dist * new_weight);
				if(calculatePoints)
				{
					dst[index].point = toPoint2D(dst[index].angle,	dst[index].dist);
				}

			}		
		}
	}
	return totalDiff;
}


float totalScanError=0;


void runFilter(ArcNode* nodes)
{
	float samples[N_POINTS];
	for(unsigned short i =0;i<N_POINTS;i++)
	{
		samples[i] = sqrt(nodes[i].dist)/30;//sqrt because distant objects have more error	
	}
	Samples::Filter::FilterSamples(samples);

	for(unsigned short i =0;i<N_POINTS;i++)
	{
		short newIndex = i-2;//shift everything right 2 measures to account for delay, confirmed in matlab
		if(newIndex <0)
		{
			newIndex += N_POINTS;
		}
						
		nodes[newIndex].dist = pow(samples[i]*30,2);	
		nodes[newIndex].point = toPoint2D(nodes[newIndex].angle,nodes[newIndex].dist );	
		
	}
}
void FindWalls()
{
	prevWalls[0] = walls[0];
	prevWalls[1] = walls[1];
	prevWalls[2] = walls[2];
	
	FindBestFlatSurface(nodes_cur, N_POINTS,walls);	

	for(int i=0;i<3;i++)
	{
		float wallDist = (walls[i].End.Dist * walls[i].End.Angle + walls[i].Start.Dist * walls[i].Start.Angle) / (walls[i].End.Angle + walls[i].Start.Angle);
		if(isnan(wallDist))
		{
			walls[i] = {};
		}
		currentRange = max(currentRange, wallDist * 2);
		printf("[Wall Start] X:%.2f Y:%.2f Angle:%.2f",walls[i].Start.Point.X,walls[i].Start.Point.Y,walls[i].Start.Angle*180/M_PI);
		printf("[Wall End] X:%.2f Y:%.2f Angle:%.2f",walls[i].End.Point.X,walls[i].End.Point.Y,walls[i].End.Angle*180/M_PI);
		printf("[Wall Dist] %.2f",wallDist);
		printf("[Wall Length] %.2f",mag(sub(walls[i].Start.Point,walls[i].End.Point) ));
	}
}

unsigned long prev_millis=0;

bool goFromScanningToEstimating()
{
	printf("> TOTAL SCAN NOISE:%.2f",totalScanError);
	// if(totalScanError>300000)
	// {				
	// 	printf("> SIGNIFICANT MOVEMENT DETECTED, RE-SCANNING");	
	// 	Clear(nodes_cur,N_POINTS);
	// 	dataCount = 0;
	// 	dedState = SCAN;
	// 	totalScanError = 0;
	// 	return false;
	// }
	Serial.println("> ENVIRONMENT ANALYSIS COMPLETE");
	
	
	dataCount = 0;
	
	 CalcPoints(nodes_cur);
	Copy(nodes_cur,nodes_prev);
	
	//PrintNodes(nodes_cur);
	dedState = EST_TRANS;
	FindWalls();		
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
  	printf("> ESTIMATING ROTATION"); 

}



void ResolveXY(float current_step_weight,bool move_x, bool move_y,unsigned long dt )
{
	if(!move_x && !move_y)
		return;

	float halfAngle = MOVE_SCAN_ANGLE;
	float port = M_PIF +  M_PIF/2.0f;
	float starboard = M_PIF / 2.0f;
	float back = M_PIF;
	
	float power =2;
	Point2D found={0};
	
	if(move_x)
	{
		float portStd;
		float sbStd;
		Point2D pos_port = avg_pos;
		float angle_port = avg_yaw;
		Point2D pos_sb = avg_pos;
		float angle_sb = avg_yaw;
		bool portR = find_x(nodes_cur,nodes_prev,pos_port,angle_port,port-halfAngle/2,port+halfAngle/2,portStd);
		//bool sbR = find_x(nodes_cur,nodes_prev,pos_sb,angle_sb, starboard-halfAngle/2,starboard+halfAngle/2,sbStd);
		 //if(!portR || isnan(pos_port.X) )
		/// {
		// 	found = pos_sb;
		// }
	//	 else if(!sbR || isnan(pos_sb.X) )
	//	 {
			found = pos_port;
			avg_yaw = angle_port;
	//	 }
		//  else
		//  {
		//  	portStd = pow(portStd,power);
		//  	sbStd =pow( sbStd,power);
			
		// 	found = div(add(mul(pos_port ,(portStd)),mul(pos_sb ,sbStd )),portStd + sbStd);
		// 	avg_yaw =  (sbStd * angle_sb +  portStd * angle_port )/ (portStd + sbStd);
		// }
	}
	else if(move_y)
	{
		float fwdStd;
		float bckStd;
		Point2D pos_fwd = avg_pos;
		float angle_fwd = avg_yaw;
		Point2D pos_bck = avg_pos;
		float angle_bck = avg_yaw;
		bool fwdR = find_y(nodes_cur,nodes_prev,pos_fwd,angle_fwd,port-halfAngle/2,port+halfAngle/2,fwdStd);
		bool bckR = find_y(nodes_cur,nodes_prev,pos_bck,angle_bck, starboard-halfAngle/2,starboard+halfAngle/2,bckStd);
		 if(!fwdR || isnan(pos_fwd.X) )
		 {
		 	found = pos_bck;
		 }
		 else if(!bckR || isnan(pos_bck.X) )
		 {
			found = pos_fwd;
		 }
		 else
		 {
		 	fwdStd = pow(fwdStd,power);
		 	bckStd =pow( bckStd,power);
			
			found = div(add(mul(pos_fwd ,(fwdStd)),mul(pos_bck ,bckStd )),fwdStd + bckStd);
			avg_yaw =  (bckStd * angle_bck +  fwdStd * angle_fwd )/ (fwdStd + bckStd);
		}
	
	}
	
	Point2D new_avg_pos =  add(mul(avg_pos , 1-current_step_weight),{found.X*current_step_weight,found.Y*current_step_weight});
	float new_velocity =dt == 0 ? 0 : mag(div(sub(new_avg_pos ,avg_pos),dt));
	if(new_velocity > 1) //bs, thats 1 mm per millisecond
		return;
	velocity = new_velocity;
	avg_pos =new_avg_pos;	
	
	printf("X:%.2f, Y:%.2f, velocity:%.2f",avg_pos.X,avg_pos.Y,velocity);
}

void ResolveRotation(float current_step_weight, bool rotate, unsigned long dt)
{
	if(!rotate)
		return;

	float total_found=0;
	int total_match=0;
	float wrongAvg = 0;
	int wrongMatchCount = 0;
	float prevWallAvg = avg_yaw;
	float results[3];
	
	for(int i =0;i<3;i++)
	{
		Wall wall = walls[i];
		if(wall.Start.Dist)
		{
			float left = wall.Start.Angle;
			float right = wall.End.Angle;
			
			if(angleDiffFast(left, right ) > 0)
			{
				left = right;
				right =  wall.Start.Angle;
			}
			
		
			while(left<0)left+=M_2PI;
			while(left>=M_2PI)left-=M_2PI;
			while(right<0)right+=M_2PI;
			while(right>=M_2PI)right-=M_2PI;
			float dist = (wall.End.Dist * wall.End.Angle + wall.Start.Dist * wall.Start.Angle) / (wall.End.Angle +wall.Start.Angle);
			float maxRange = dist*1.5;
			float found=0;
			if( find_yaw(nodes_cur,nodes_prev,left-ROT_SCAN_ANGLE/2,right+ROT_SCAN_ANGLE/2,prevWallAvg,maxRange,found) && !isnan(found))
			{
				results[total_match] = found;
				prevWallAvg = found;
				total_found += found;
				total_match++;
			}
			else
			{
				//prevWallAvg = found;
				wrongAvg += found;
				wrongMatchCount++;
				//avg_yaw = (found + avg_yaw)/2;
			}
		}
	}
	if(total_match !=0)
	{
		///find consensus
		float std = standardDeviation(results,total_match);
		
		///end find consensus
		total_found /=total_match;
		
		//printf("%.2f, %.2f, %.2f",avg_yaw,total_match,total_found);
		float new_avg_yaw =  avg_yaw *( 1-current_step_weight) + total_found*current_step_weight;


		float new_angular_velocity =dt == 0 ? 0 : (new_avg_yaw -avg_yaw)/dt;
		if(new_angular_velocity > 1) //bs
			return;
		angular_velocity = new_angular_velocity;
		avg_yaw =new_avg_yaw;	
		
	}
	else
	{
		avg_yaw = wrongAvg /wrongMatchCount;
		printf("YAW(bad):%.6f",avg_yaw*180/M_PI);

	}
}
float durationStopped =0;

void updateReferenceScan()
{
	
		printf("NO MOVEMENT DETECTED FOR 3 SECONDS, UPDATING REFERENCE SCAN");
		CalcPoints(nodes_cur);
		Copy(nodes_cur,nodes_prev);
		
		FindWalls();
	
}
void DataIn(LidarScanNormalMeasureRaw* nodes, unsigned short nodeCount,bool  move_x, bool move_y, bool rotate)
{	
	unsigned long millis_now = millis();
	unsigned long  dt =  millis_now - prev_millis;
	prev_millis = millis_now;
	
	switch (dedState)
	{
	case STARTUP:
		if (dataCount > 5)
		{
			
  			Serial.println("> INITIALIZING EYE FILTER"); 	
  			Samples::Filter::SetupFilter();
			Clear(nodes_cur,N_POINTS);
			Clear(nodes_prev,N_POINTS);
			dedState = SCAN;
			dataCount = 0;  
  			Serial.println("> EVALUATING ENVIRONMENT 0.0%"); 		
			
			totalScanError = 0;
		}
		break;
	case SCAN:
		totalScanError += read_scan(nodes, nodes_cur, nodeCount);
		if (dataCount > SCAN_COUNT)
		{			
  			if(!goFromScanningToEstimating())
			{
				return;
			}
			// if(rotate)
			// {
			// 	Clear(nodes_cur,N_POINTS);
			// }
		}
		else
		{			
			printf("> ANALYSING ENVIRONMENT... %.02f\%",pow(dataCount / (float)SCAN_COUNT,2)*100);
  		
			break;
		}
	case EST_TRANS:
	{		
		float current_step_weight = min(0.95,velocity*100+0.2);
		read_scan(nodes,nodes_cur, nodeCount,0.5,true);
	//	runFilter(nodes_cur);

		if(rotate)
		{
			dedState = ROTATING;
			updateReferenceScan();
			break;
		}

		ResolveXY(current_step_weight,move_x,  move_y, dt );
	
		if(velocity <0.001 )
		{
			durationStopped += dt;
			if(durationStopped > 5000 && dt < 100 )
			{
				//updateReferenceScan();
				durationStopped = 0;
			}
		}
		return;
	}
	case ROTATING:
		read_scan(nodes,nodes_cur, nodeCount,0.5,true);
		//runFilter(nodes_cur);	
		if(move_x || move_y)
		{
			dedState = EST_TRANS;
			updateReferenceScan();
			printf("switch to translation");
			return;
		}			
		break;
	case EST_ROT:
	{	
		
		//we shouldnt be moving when doing this	
		float current_angular_step_weight =0.5;//min(0.95,angular_velocity*100+0.5);		
		
		read_scan(nodes,nodes_cur, nodeCount,current_angular_step_weight,true);	
		//runFilter(nodes_cur);			
		ResolveRotation(current_angular_step_weight,rotate,dt);
		// if(angular_velocity == 0)
		// {
		// 	durationStopped += dt;
		// 	updateReferenceScan();
		// }
		if(move_x || move_y)
		{
			dedState = EST_TRANS;
			updateReferenceScan();
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