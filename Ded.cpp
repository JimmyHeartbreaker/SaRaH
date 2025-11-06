
#include "ded.h"
#include "NodeServices.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "sample_filter.h"
#include "pid_controller.h"
#include "arduino_ext.h"
using namespace std;

#define SCAN_COUNT 30

DedState dedState = STARTUP;


ArcNode nodes_ref[N_POINTS]={0};
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
	avg_yaw = 0 * M_PI/180;
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

	const float halfAngle = MOVE_SCAN_ANGLE/2;;
	const float port = M_PIF +  M_PIF/2.0f;
	const float starboard = M_PIF / 2.0f;
	const float back = M_PIF;
	
	const float power =2;
	Point2D found={0};
	
	if(move_x)
	{
		float port_score=10000;
		float sb_score=10000;
		Point2D pos_port = avg_pos;
		float angle_port = avg_yaw;
		Point2D pos_sb = avg_pos;
		float angle_sb = avg_yaw;
		bool portR=false;
		bool sbR=false;
		if(portFirst)
		{	
			port:
			
			portR = find_x(nodes,nodes_ref,pos_port,angle_port,port-halfAngle,port+halfAngle,port_score,{-1,0});
			if(portR && abs(port_score) < 1)
			{			
				found = pos_port;
				avg_yaw = angle_port;		
			}
			else if(portFirst)//sb didnt run yet
				goto sb;
		}
		else
		{
			sb:
			sbR = false;//find_x(nodes,nodes_ref,pos_sb,angle_sb, starboard-halfAngle,starboard+halfAngle,sb_score,{1,0});
			if(sbR && sb_score < 1)
			{			
				found = pos_sb;
				avg_yaw = angle_sb;	
			}
			else if(!portFirst)//port didnt run yet
				goto port;
		}	
		
	//	printf("port_score:%.6f, sb_score:%.6f",port_score,sb_score);	
	//	printf("port:%.6f, sb:%.6f",pos_port.X,pos_sb.X);			
		if(!(sbR ^ portR) )//we got both or got neither, must have dropped through both and need to choose
		{
			if(fabs(sb_score - port_score) / (fmax(sb_score, port_score) + 1e-6f) < 0.15f)
			{
				found = div(add(mul(pos_port ,sb_score),mul(pos_sb ,port_score  )),port_score + sb_score);
				avg_yaw =  (sb_score * angle_port +  port_score * angle_sb  )/ (port_score + sb_score);
			}
			else if(sb_score < port_score)
			{
				found = pos_sb;
				avg_yaw = angle_sb;
				portFirst = false;
			}
			else
			{
				found = pos_port;
				avg_yaw = angle_port;				
				portFirst = true;
			}
		}
		
	}
	else if(move_y)
	{
		float fwd_score=0;
		float bck_score=0;
		Point2D pos_fwd = avg_pos;
		float angle_fwd = avg_yaw;
		Point2D pos_bck = avg_pos;
		float angle_bck = avg_yaw;
		bool portR=false;
		bool sbR=false;
		if(fwdFirst)
		{	
			fwd:
			
			portR = find_y(nodes,nodes_ref,pos_fwd,angle_fwd,port-halfAngle,port+halfAngle,fwd_score,{-1,0});
			if(portR && fwd_score < 0.5)
			{			
				found = pos_fwd;
				avg_yaw = angle_fwd;		
			}
			else if(fwdFirst)//sb didnt run yet
				goto bck;
		}
		else
		{
			bck:
			sbR =  find_y(nodes,nodes_ref,pos_bck,angle_bck, starboard-halfAngle,starboard+halfAngle,bck_score,{1,0});
			if(sbR && bck_score < 0.5)
			{			
				found = pos_bck;
				avg_yaw = angle_bck;	
			}
			else if(!fwdFirst)//port didnt run yet
				goto fwd;
		}	
					
		if(sbR && portR )//we got both, must have dropped through both and need to choose
		{
			if(fabs(bck_score - fwd_score) / (fmax(bck_score, fwd_score) + 1e-6f) < 0.15f)
			{
				found = div(add(mul(pos_fwd ,bck_score),mul(pos_bck ,fwd_score  )),fwd_score + bck_score);
				avg_yaw =  (bck_score * angle_fwd +  fwd_score * angle_bck  )/ (fwd_score + bck_score);
			}
			else if(bck_score < fwd_score)
			{
				found = pos_bck;
				avg_yaw = angle_bck;
				fwdFirst = false;
			}
			else
			{
				found = pos_fwd;
				avg_yaw = angle_fwd;				
				fwdFirst = true;
			}
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

void ResolveRotation(ArcNode* nodes,Wall* walls, float current_step_weight, bool rotate, unsigned long dt)
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
			
			//printf("left:%.2f,right:%.2f, dist:%.6f, length:%.6f",left*180/M_PI,right*180/M_PI,dist,mag(sub(wall.Start.Point,wall.End.Point)));
			float maxRange = dist*1.5;
			float found=0;
			//if( 
				find_yaw(nodes,nodes_ref,left-ROT_SCAN_ANGLE/2,right+ROT_SCAN_ANGLE/2,prevWallAvg,maxRange,found);// && !isnan(found)
			//)
			//{
				results[total_match] = found;
				prevWallAvg = found;
				total_found += found;
				total_match++;
			// }
			// else
			// {
			// 	//prevWallAvg = found;
			// 	wrongAvg += found;
			// 	wrongMatchCount++;
			// 	//avg_yaw = (found + avg_yaw)/2;
			// }
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
		printf("YAW:%.6f",avg_yaw*180/M_PI);
		
	}
	else
	{
		avg_yaw = wrongAvg /wrongMatchCount;
		printf("YAW(bad):%.6f",avg_yaw*180/M_PI);

	}
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
				
		ResolveRotation(nodes_now,walls,current_angular_step_weight,rotate,dt);
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