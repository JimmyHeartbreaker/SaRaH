#include "NodeServices.h"
#include "pid_controller.h"
#ifndef TESTING
#include "arduino_ext.h"
#include "arduino.h"
#endif
#include "robot_math.h"

#define MOVE_SCAN_ANGLE (180* (M_PI_F/180))
#define ROT_SCAN_ANGLE (180*M_PI_F/180)
#define RESOLUTION (M_2PI/N_POINTS) //rad

/// @brief PID section

void serialPrintf2(const char *format, ...)
{
    char buf[256];             // adjust size if needed
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);  // format string with params
    va_end(args);
#ifndef TESTING
    Serial.println(buf);         // send to Serial
#endif
}
const float kp_rotate = 0.04f;
pid_state pid_rotate = 
    { 
        kp_rotate,  //kp
        0.01f * kp_rotate , //ki
        0.0f * kp_rotate, //kd
        5* M_PI/180, //max rotate
        -5* M_PI/180, //min rotate
        360.0f/N_POINTS, //dt
        5, //Tt
        RESOLUTION/2,  //deadband
        0.0, //integral
        0.0, //prev error
    };

const float kp_move_rotate = 0.01f;
pid_state pid_move_rotate = 
    { 
        kp_rotate,  //kp
        0.01f * kp_rotate , //ki
        0,//0.01f * kp_rotate, //kd
        5* M_PI/180, //max rotate
        -5* M_PI/180, //min rotate
        360.0f/N_POINTS, //dt
        5, //Tt
        RESOLUTION,  //deadband
        0.0, //integral
        0.0, //prev error
    };

const float kp_move_master = 0.8f;
pid_state pid_move_master = 
    { 
        kp_move_master,  //kp
       0.01f *kp_move_master , //ki
       0.0f * kp_move_master, //kd
        1000, //max dist
        -1000, //min dist
        0.1f, //dt
        5000, //Tt
        0.2,  //deadband
        0.0, //integral
        0.0, //prev error
    };

const float kp_move_slave = 0.0001f;
pid_state pid_move_slave = 
    { 
        kp_move_slave,  //kp
      0,//  0.01f *kp_move_slave , //ki
      0,//  0.07f * kp_move_slave, //kd
        5, //max
        -5, //min
        0.1f, //dt
        5000, //Tt
        1,  //deadband
        0.0, //integral
        0.0, //prev error
    };

/// <summary>
/// consult uni notes
/// </summary>
/// <param name="y"></param>
/// <returns></returns>
float PIDUpdate(pid_state* pid, float y,float prev_u)
{
	float e = y;
	if (fabs(y) <= pid->deadband)
	{
		e = 0.0;
	}
	float de_dt = (e - pid->prev_error) / pid->dt;

	float	u_un = pid->kp * e + pid->ki * pid->integral + pid->kd * de_dt;
	
	float u = std::fmax(pid->min, std::fmin(u_un + prev_u, pid->max));

	//	# anti - windup correction;
	pid->integral += pid->dt * e;// + (u - (u_un+prev_u)) / pid->Tt);

	pid->prev_error = e;
	return u;
}



float sum_difference_ranged(ArcNode* new_nodes, ArcNode* old_nodes,float left,float right,float guess_yaw,float maxRange)
{
    float total=0;
    int matches=0;
    for(unsigned short i=0;i<N_POINTS;i++)
    {
        ArcNode* new_node = new_nodes+i;
        if(new_node->angle&& new_node->dist < maxRange)
        {       
            float new_angle = new_node->angle + guess_yaw;
           // left-= guess_yaw;
          //  right -= guess_yaw;
            while(new_angle<0)
            {
                new_angle += M_2PI;
            }
            while(new_angle>=M_2PI)
            {
                new_angle -= M_2PI;
            }
            if( angleDiffFast(new_angle, left) > 0 && angleDiffFast(new_angle, right) < 0 )
            {  
               // 
             
                int index =  FindClosestNode(new_angle, new_node->dist,old_nodes,90);
                ArcNode t = old_nodes[index];
                if(t.angle )
                {               

                    matches++;
                    float dff =  angleDiffFast(new_angle,t.angle);
                 
                    total+=dff;
                }
            }
        }
    }
   // serialPrintf2("matches:%i",matches);
   if(matches==0)
        return 0;
    return total/matches;
}

ArcNode getInterpolatedNode(float angle,float dist, ArcNode* nodes)
{
    unsigned short old_index= FindClosestNode(angle,dist,nodes,20);
    ArcNode* found = nodes + old_index;
    angle = found->angle;
    float roundedAngle  = toAngle(old_index ,N_POINTS);   
    float angleRemainder = angle - roundedAngle;
    if(fabs(angleRemainder) < 1e-6 )
        return *found;

    float percent= angleRemainder / RESOLUTION;

    unsigned short next_index = ((percent > 0 ? old_index + 1 : old_index - 1) + N_POINTS)%N_POINTS;

    percent = fabs(percent);
    ArcNode* prev = nodes+old_index;
    if(!prev->dist)
        return {0};
    ArcNode* next = nodes+next_index;
    ArcNode old_frame_arc;
    old_frame_arc.point = add( mul(next->point,1-percent), mul(prev->point,percent));
    old_frame_arc.dist = next->dist*(1-percent)+ prev->dist*percent;
    old_frame_arc.angle =  next->angle*(1-percent)+ prev->angle*percent;
    return old_frame_arc;
}
Point2D sum_difference(ArcNode* new_nodes, ArcNode* old_nodes,float left,float right,Point2D guess,float angleGuess,float& std,float& angle_diff,Point2D heading)
{
    Point2D total = {0,0};
    short matches=0;
    short anglematches=0;
    float distances[1000];
    float totalWight=0;
    int potentialResultCount=0;
    int searchRange = (int)round(MOVE_SCAN_ANGLE / RESOLUTION)/2;
    Point2D prev_new_normal={0,0};
    float angleChange=0;
    for(unsigned short i=0;i<N_POINTS;i++)
    {
        ArcNode* new_node = new_nodes+i;
        if(new_node->angle )
        {   
             float new_angle ;
            if(new_node->angle >= M_PIF)
            {
                new_angle =(new_node->angle-M_2PI) +  angleGuess;
            }
            else
            {
                new_angle =new_node->angle +  angleGuess;
            }

                while (new_angle >= M_PIF * 2)
                {
                    new_angle -= M_PIF * 2;
                }

                while (new_angle < 0)
                {
                    new_angle += M_PIF * 2;
                }
            if( angleDiffFast(new_angle, left) > 0 && angleDiffFast(new_angle, right) < 0 )
            {  
                
                distances[potentialResultCount++] = new_node->dist;

                ArcNode* prev_frame_point = new_nodes + ((i == 0) ? N_POINTS - 1 : i - 1);
                float prev_angle = prev_frame_point->angle + angleGuess;

                Point2D new_point = toPoint2D(new_angle,new_node->dist);
                Point2D expected_old_frame_point = add(new_point ,guess);                             
               
                Point2D prev_point = toPoint2D(prev_angle,prev_frame_point->dist);
                Point2D expected_prev_old_frame_point = add(prev_point ,guess);  

                Point2D expected_v = sub(expected_prev_old_frame_point,expected_old_frame_point);                     
                Point2D expected_n = normalize(normal(expected_v));//clockwise
                angleChange =  angleChange*0.5 + angleBetween(prev_new_normal,expected_n)*0.5;
                float angleToHeading;
                //corner 
                if(prev_new_normal.X !=0 && fabs(angleChange)> 0.4 )
                { 
                //    serialPrintf2("corner? %.2f skip",angleChange);
                    continue;
                    //looks like a corner
               //    
                }
                //angle to heading, slope > 60 degrees, we might rolling past it
                else if( angleToHeading = fabs( angleBetween(heading,expected_n)) > M_PIF/3)
                {
                //    serialPrintf2("steep slope %.2f, skip",angleToHeading);
                    continue;
                }
                else
                {
                    float angle = toAngle(expected_old_frame_point);  

                    while (angle >= M_PIF * 2)
                    {
                        angle -= M_PIF * 2;
                    }

                    while (angle < 0)
                    {
                        angle += M_PIF * 2;
                    }

                    ArcNode old_frame_arc = getInterpolatedNode(new_angle, mag(expected_old_frame_point),old_nodes);
                    if(!old_frame_arc.dist)
                        continue;

                // serialPrintf2("old_index:%i,angle:%.6f",old_index,angle);
                    
                    ArcNode prev_old_frame_arc = getInterpolatedNode(old_frame_arc.angle-RESOLUTION, mag(expected_old_frame_point),old_nodes); ;

                    Point2D v = sub(prev_old_frame_arc.point,old_frame_arc.point);      
                    if(mag(v)==0)
                    {
                 //       serialPrintf2("skip angle due to alignment 1");
                        //not able to match because both nodes found the same closest
                                    
                    }
                    else
                    {
                        Point2D n = normalize(normal(v));//clockwise

                        float a = angleBetween(expected_n,n);   
                        if(fabs(a)> oneDegree*15)//this doesnt seem right, lets ignore it 
                        {
                   //     serialPrintf2("skip angle due to alignment 2");
                           
                        }
                        else
                        {
                            angle_diff += a;                            
                            anglematches++;
                        }
                    }
                    
                // 
                 //   serialPrintf2("nx:%.6f,ny:%.6f",n.X,n.Y);
                
                    matches++;
                    Point2D old_midpoint = {(old_frame_arc.point.X * old_frame_arc.dist + prev_old_frame_arc.point.X * prev_old_frame_arc.dist)/(prev_old_frame_arc.dist + old_frame_arc.dist ),
                                            (old_frame_arc.point.Y * old_frame_arc.dist + prev_old_frame_arc.point.Y * prev_old_frame_arc.dist)/(prev_old_frame_arc.dist + old_frame_arc.dist )};
                    Point2D new_midpoint = {(expected_old_frame_point.X * new_node->dist + expected_prev_old_frame_point.X * prev_frame_point->dist )/(new_node->dist +  prev_frame_point->dist ),
                                            (expected_old_frame_point.Y * new_node->dist + expected_prev_old_frame_point.Y * prev_frame_point->dist )/(new_node->dist +  prev_frame_point->dist )};
                    
                    
                    
                     
                    float weight = 1;//  mag(new_midpoint);
                    Point2D change = sub(old_midpoint,new_midpoint);
                
                    total.X += change.X* weight;                           
                    total.Y += change.Y* weight;
                    
                    totalWight += weight;
                                     
                }
                prev_new_normal = expected_n;
            }
        }
    }
     if(matches >0)
     {
        total.X /= matches;
        total.Y /= matches;
     }
     if(anglematches > 0)
     {
        angle_diff /= anglematches;
     }
     std = standardDeviation_special(distances,potentialResultCount);
    // serialPrintf2("matches:%i anglematches:%i",matches,anglematches);
    return total;
}



bool find_y(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& guess,float& angle,float left, float right,float& std,Point2D heading)
{        
    int timeout=500;  
    
    float avgAngleError=0.0;
    #ifndef TESTING
    unsigned long mills_start = millis();
    #endif
    while(true)
    {  
        #ifndef TESTING
        if( millis() - mills_start>timeout)                
           return false;    
        #endif
        float error_angle=0; 
      
        Point2D prev_guess = guess;
        Point2D error= sum_difference(nodes_new,nodes_old, left,right,guess,angle,std,error_angle,heading); 
        if(fabs(error.Y) > pid_move_master.max  ||fabs(guess.X) > pid_move_slave.max )        
            guess = add(prev_guess,{1e-6,1e-6});        

        avgAngleError = avgAngleError == 0 ? error_angle : (avgAngleError*0.8 + error_angle*0.2);           
       
        angle = PIDUpdate(&pid_rotate,-avgAngleError,angle); 
        guess.X = PIDUpdate(&pid_move_slave,error.Y,guess.Y);   
        guess.Y = PIDUpdate(&pid_move_master,error.X,guess.X); 
         
      //  serialPrintf2("guess_x: %.6f, error_x: %.6f,error_y: %.6f, error_angle: %.6f, angle: %.6f",guess.X,error.X,error.Y,avgAngleError,angle);
       
        if(fabs(error.Y) < pid_move_master.deadband  )
        {
            break;
        } 
    }  

    return true;    
}


bool find_x(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& guess,float& angle,float left, float right,float& std,Point2D heading)
{        
    int timeout=500;  
    
    float avgAngleError=0.0;
    #ifndef TESTING
    unsigned long mills_start = millis();
    #endif
    while(true)
    {  
        #ifndef TESTING
        if( millis() - mills_start>timeout)                
           return false;    
        #endif
        float error_angle=0; 
      
        Point2D prev_guess = guess;
        Point2D error= sum_difference(nodes_new,nodes_old, left,right,guess,angle,std,error_angle,heading); 
        if(fabs(error.X) > pid_move_master.max  ||fabs(guess.Y) > pid_move_slave.max )        
            guess = add(prev_guess,{1e-6,1e-6});

        avgAngleError = avgAngleError == 0 ? error_angle : (avgAngleError*0.8 + error_angle*0.2);           
       
        angle = PIDUpdate(&pid_rotate,-avgAngleError,angle); 
      //  guess.Y = PIDUpdate(&pid_move_slave,error.Y,guess.Y);   
        guess.X = PIDUpdate(&pid_move_master,error.X,guess.X); 
         
   //     serialPrintf2("guess_x: %.6f, error_x: %.6f,error_y: %.6f, error_angle: %.6f, angle: %.6f",guess.X,error.X,error.Y,avgAngleError,angle);
       
        if(fabs(error.X) < pid_move_master.deadband  )
        {
            break;
        } 
    }  

    return true;    
}

bool find_yaw(ArcNode* nodes_new, ArcNode* nodes_old,float left, float right, float assumed_yaw,float maxRange, float& outGuess)
{
     int timeout=5000;
  
    float guess_yaw=assumed_yaw;
     
   // pid_move_master.deadband = 
       // printf("tolerance:%.2f",tolerance );
    
    //serialPrintf2("guess_yaw: %.2f",guess_yaw);
   // unsigned long mills_start = millis();
   // Serial.println("1");
    while(true)
    {
     //   if( millis() - mills_start>timeout)   
        // {
            
        //     outGuess = guess_yaw  ;           
        //     return false;     
        // }
        float error = sum_difference_ranged(nodes_new,nodes_old,left,right,guess_yaw,maxRange);    
        
       // float magError =abs(error.X*error.X + error.Y * error.Y);
      //  tolerance = magError;
        if(fabs(error)< pid_rotate.deadband)
        {
         //   serialPrintf2("deadband hit: %.8f",error);
        //    Serial.println("3");
            break;
        }
    //    serialPrintf2("error: %.2f",error);
        guess_yaw = PIDUpdate(&pid_rotate,-error,guess_yaw);     
        
       // serialPrintf2("guess_yaw: %.2f, error: %.8f",guess_yaw*180/M_PI,error);
             //  serialPrintf2("guess_y: %.2f, error_y: %.2f",guess_y,error.Y);
    }
   
    outGuess = guess_yaw;
    return true;    
}
