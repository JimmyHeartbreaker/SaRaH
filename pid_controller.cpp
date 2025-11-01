#include "NodeServices.h"
#include "pid_controller.h"
#include "arduino_ext.h"
#include "arduino.h"
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

    Serial.println(buf);         // send to Serial
}
const float kp_rotate = 0.001f;
pid_state pid_rotate = 
    { 
        kp_rotate,  //kp
        0.0f * kp_rotate , //ki
        0.0f * kp_rotate, //kd
        5* M_PI/180, //max rotate
        -5* M_PI/180, //min rotate
        360.0f/N_POINTS, //dt
        5, //Tt
        RESOLUTION,  //deadband
        0.0, //integral
        0.0, //prev error
    };

const float kp_move_rotate = 0.01f;
pid_state pid_move_rotate = 
    { 
        kp_rotate,  //kp
        0.0f * kp_rotate , //ki
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
       0,// 0.01f *kp_move_master , //ki
      0,//  0.02f * kp_move_master, //kd
        1000, //max dist
        -1000, //min dist
        0.1f, //dt
        5000, //Tt
        1,  //deadband
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
	if (abs(y) <= pid->deadband)
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

Point2D sum_difference(ArcNode* new_nodes, ArcNode* old_nodes,float left,float right,Point2D guess,float angleGuess,float& std,float& angle_diff)
{
    Point2D total;
    short matches=0;
    float distances[1000];
    float totalWight=0;
    int potentialResultCount=0;
    int searchRange = (int)round(MOVE_SCAN_ANGLE / RESOLUTION)/2;
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

                float angle = toAngle(expected_old_frame_point);  

                while (angle >= M_PIF * 2)
                {
                    angle -= M_PIF * 2;
                }

                while (angle < 0)
                {
                    angle += M_PIF * 2;
                }

                unsigned short old_index=  FindClosestNode(angle, mag(expected_old_frame_point),old_nodes,1);
               // serialPrintf2("old_index:%i,angle:%.6f",old_index,angle);
                ArcNode* old_frame_arc = old_nodes+old_index;
                ArcNode* prev_old_frame_arc = old_nodes +  ((old_index == 0) ? N_POINTS - 1 : old_index - 1);

                Point2D v = sub(prev_old_frame_arc->point,old_frame_arc->point);                     
                Point2D n = normalize(normal(v));//clockwise
                
              //  serialPrintf2("enx:%.6f,eny:%.6f",expected_n.X,expected_n.Y);
               // serialPrintf2("nx:%.6f,ny:%.6f",n.X,n.Y);
               
                matches++;
                Point2D old_midpoint = {(old_frame_arc->point.X * old_frame_arc->dist + prev_old_frame_arc->point.X * prev_old_frame_arc->dist)/(prev_old_frame_arc->dist + old_frame_arc->dist ),
                                        (old_frame_arc->point.Y * old_frame_arc->dist + prev_old_frame_arc->point.Y * prev_old_frame_arc->dist)/(prev_old_frame_arc->dist + old_frame_arc->dist )};
                Point2D new_midpoint = {(expected_old_frame_point.X * new_node->dist + expected_prev_old_frame_point.X * prev_frame_point->dist )/(new_node->dist +  prev_frame_point->dist ),
                                         (expected_old_frame_point.Y * new_node->dist + expected_prev_old_frame_point.Y * prev_frame_point->dist )/(new_node->dist +  prev_frame_point->dist )};
                
                
             //  float weight =   mag(new_midpoint);
                Point2D change = sub(old_midpoint,new_midpoint);
               
                    total.X += change.X;//* weight;                           
                    total.Y += change.Y;// * weight;
                
               // totalWight += weight;
                float a = angleBetween(expected_n,n);
                
                    angle_diff += a;// * weight;
                
                
            }
        }
    }
    
     total.X /= matches;
     total.Y /= matches;
     angle_diff /= matches;
     std = standardDeviation_special(distances,potentialResultCount);
    return total;
}



bool find_y(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& guess,float& angle,float left, float right,float& std)
{        
    int timeout=500;  
    
    float avgAngleError=0.0;
    //unsigned long mills_start = millis();
    
    while(true)
    {  
        
      //  if( millis() - mills_start>timeout)                
      //      return false;    
       
        float error_angle=0; 
        Point2D prev_guess = guess;
        Point2D error= sum_difference(nodes_new,nodes_old, left,right,guess,angle,std,error_angle); 
        if(abs(error.Y) > 10000)
        {
            guess = add(prev_guess,{1e-6,1e-6});
        }
         if(abs(guess.Y) == pid_move_master.max)
        {
            guess = {0,0};
            angle = 0;
        }
        avgAngleError = avgAngleError == 0 ? error_angle : (avgAngleError*0.8 + error_angle*0.2);
     //   serialPrintf2("guess_y: %.2f, error_y: %.2f,error_x: %.2f, error_angle: %.6f, angle: %.6f",guess.Y,error.Y,error.X,error_angle,avgAngleError);
           
        if(abs(error.Y) < pid_move_master.deadband && abs(error_angle) < pid_rotate.deadband )
        {
            break;
        } 
        angle = PIDUpdate(&pid_rotate,-avgAngleError,angle); 
        guess.X = PIDUpdate(&pid_move_slave,error.Y,guess.Y);   
        guess.Y = PIDUpdate(&pid_move_master,error.X,guess.X); 
        if(abs(guess.X) == pid_move_slave.max)
            guess.X =0;
       
      //  serialPrintf2("guess_y: %.2f, error_y: %.2f",guess_y,error.Y);
    }
   

    return true;    
}

bool find_x(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& guess,float& angle,float left, float right,float& std)
{        
    int timeout=500;
  
    
    float avgAngleError=0.0;
    unsigned long mills_start = millis();
    angle = 0;
    while(true)
    {  
        
        if( millis() - mills_start>timeout)                
           return false;    
       
        float error_angle=0; 
      
        Point2D prev_guess = guess;
        Point2D error= sum_difference(nodes_new,nodes_old, left,right,guess,angle,std,error_angle); 
        if(abs(error.X) > 10000)        
            guess = add(prev_guess,{1e-6,1e-6});
        
        if(abs(guess.X) > pid_move_master.max)
        {
            guess = {0,0};
            angle = 0;
        }

        if(abs(guess.Y) > pid_move_slave.max)
            guess.Y =0;

        avgAngleError = avgAngleError == 0 ? error_angle : (avgAngleError*0.8 + error_angle*0.2);
           
       
        angle = PIDUpdate(&pid_rotate,-avgAngleError,angle); 
        guess.Y = PIDUpdate(&pid_move_slave,error.Y,guess.Y);   
        guess.X = PIDUpdate(&pid_move_master,error.X,guess.X); 
         
        serialPrintf2("guess_x: %.6f, error_x: %.6f,error_y: %.6f, error_angle: %.6f, angle: %.6f",guess.X,error.X,error.Y,avgAngleError,angle);
       
        if(abs(error.X) < pid_move_master.deadband && abs(avgAngleError) < pid_rotate.deadband )
        {
            break;
        } 
      //  serialPrintf2("guess_y: %.2f, error_y: %.2f",guess_y,error.Y);
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
        if(abs(error)< pid_rotate.deadband)
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
