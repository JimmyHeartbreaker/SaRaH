#include "..\headers\NodeServices.h"
#include "..\headers\pid_controller.h"
#ifndef TESTING
#include "..\headers\arduino_ext.h"
#include "arduino.h"
#endif
#include "..\headers\robot_math.h"


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

const float kp_move_rotate = 0.2f;
pid_state pid_move_rotate = 
    { 
        kp_move_rotate,  //kp
        0.0f * kp_move_rotate , //ki
        0.0f * kp_move_rotate, //kd
        10* M_PI/180, //max rotate
        -10* M_PI/180, //min rotate
        360.0f/N_POINTS, //dt
        5, //Tt
        0,  //deadband
        0.0, //integral
        0.0, //prev error
    };

const float kp_rotate = 0.001f;
pid_state pid_rotate = 
    { 
        kp_rotate,  //kp
        0.0f * kp_rotate , //ki
        0,//0.01f * kp_rotate, //kd
        M_PI, //max rotate
        -M_PI, //min rotate
        360.0f/N_POINTS, //dt
        5, //Tt
        0.0005,// RESOLUTION,  //deadband
        0.0, //integral
        0.0, //prev error
    };

const float kp_rotate_move = 0.0001f;
pid_state pid_rotate_move = 
    { 
        kp_rotate_move,  //kp
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

const float kp_move_master = 0.1f;
pid_state pid_move_master = 
    { 
        kp_move_master,  //kp
       0.00f *kp_move_master , //ki
       0.00f * kp_move_master, //kd
        2000, //max dist
        -2000, //min dist
        0.1f, //dt
        5000, //Tt
        0.2,  //deadband
        0.0, //integral
        0.0, //prev error
    };

const float kp_move_slave = 0.001f;
pid_state pid_move_slave = 
    { 
        kp_move_slave,  //kp
      0,//  0.01f *kp_move_slave , //ki
      0,//  0.07f * kp_move_slave, //kd
        5, //max
        -5, //min
        0.1f, //dt
        5000, //Tt
        0,  //deadband
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

void calculate_angle_and_direction(float distanceRatio,float attenuatioNew,float max_Dist, float strengthAdj,const Point2D& n,const Point2D& v,const Point2D& midPoint, const Point2D& heading,ArcNode* left, ArcNode* right,float& angle_error,float& distance_error,float& angle,Point2D& dist,float maxAngle)
{
    Point2D cv = sub(right->point,left->point);
    
    Point2D cn = normalize( normal_ccw(cv));      
    Point2D cmidPoint = mul(add(right->point,left->point),0.5);  
    float cmidPointMag = mag(cmidPoint);
    
    
    Point2D cmidPoint_normed = div(cmidPoint,cmidPointMag);
   
    Point2D midPointDiff =    sub(midPoint,cmidPoint) ;
    float norm_angle = angleBetween(n,cn);
    

    float distanceToLine =  (cross_scalar(midPointDiff,normalize(cv)) * attenuatioNew)* ( 1-strengthAdj) ;

    dist = mul(cmidPoint_normed,distanceToLine);
 
    float distanceRemainingRatio = distanceToLine/max_Dist; //smaller better
    
    float normal_angle_error = norm_angle*norm_angle ; //smaller better

    distance_error =  normal_angle_error  + distanceRemainingRatio * distanceRemainingRatio +   strengthAdj ;
  
    
    angle_error =    normal_angle_error +   strengthAdj;
   
    angle=norm_angle * ( 1-strengthAdj) ;   
}

int findAngleAndDistanceOffset(float max_dist, float est_dist_ratio,Point2D& nl, Point2D& nr, ArcNode* old_nodes,Point2D heading,float sample_angle, float& adjust_angle,Point2D& bestDist, int searchSize,float& angle_error, float& distance_error,float maxAngle)
{
    
    Point2D v = sub(nr,nl);
    Point2D n = normalize(normal_ccw(v));
    Point2D midPoint = mul(add(nl,nr),0.5);
    
    float magMidp = mag(midPoint);
     float nmp_angle = angleBetween(n,div(midPoint,magMidp));     
    #ifdef TESTING
   float attenuationNew=1.0;//no lighting attenuaton in simulation
   #else
    float attenuationNew = 1-fabs(nmp_angle);// dot(normalize(n),normalize(midPoint));
    #endif
   
    
    float segmentSize =mag(v);
    float segmentSizeWeight = (1-magMidp/ (segmentSize+magMidp));

    float strengthAdj = segmentSizeWeight * (1- fmax(0,maxAngle-fabs(nmp_angle)) * 1/maxAngle);
    strengthAdj *=strengthAdj;
    float best_angle_error = 1000000;        
    float best_dist_error = 1000000;        
    float initialAngle=sample_angle * est_dist_ratio;
    short index =  toIndex(initialAngle ,N_POINTS);
    
    short bestIndex =-1;
    float bestAngle=0;
    ArcNode* prev_left = old_nodes + index;
    ArcNode* prev_right = prev_left;
    int prev_right_index = index;
    int prev_left_index = index;
    float old_sample_step_size = est_dist_ratio * RESOLUTION;

    
     float distanceRatio = magMidp/max_dist;
     distanceRatio *= distanceRatio;
    for (int i = 1; i <searchSize; i++)
    {                  
        float nextRightAngle = initialAngle + old_sample_step_size*i;
        int rightIndex =  std::fmax(prev_right_index+1,toIndex(nextRightAngle ,N_POINTS));
        if (rightIndex >= N_POINTS)
        {
            rightIndex -= N_POINTS;
        }

        ArcNode* right = (old_nodes + rightIndex);
        if(right->dist && prev_right->dist)
        {
            float error;
            float angle;
            Point2D dist;
            calculate_angle_and_direction(distanceRatio,attenuationNew,max_dist,strengthAdj,n,v,midPoint,heading,prev_right,right,angle_error,distance_error,angle,dist,maxAngle);
       
              if( (distance_error  < best_dist_error ) && fabs(angle)<maxAngle)
                {
                    bestIndex = (rightIndex);//always return the right most index                
                    best_angle_error =  angle_error;
                    best_dist_error = distance_error;
                    bestAngle=angle;     
                    bestDist = dist;
                }
            
            prev_right = right;
            prev_right_index = rightIndex;
        }

         float nextLeftAngle = initialAngle - old_sample_step_size*i;
        int leftIndex =  std::fmin(prev_left_index-1,toIndex(nextLeftAngle ,N_POINTS));
        if (leftIndex < 0)
        {
            leftIndex += N_POINTS;
        }        
       
        ArcNode* left = (old_nodes + leftIndex);
         if(left->dist && prev_left->dist )
        {
            float error;
            float angle;
            Point2D dist;
            calculate_angle_and_direction(distanceRatio,attenuationNew,max_dist, strengthAdj,n,v,midPoint,heading,left,prev_left,angle_error,distance_error,angle,dist,maxAngle);
     
            if( (distance_error  < best_dist_error ) && fabs(angle)<maxAngle)
            {
                bestIndex = (rightIndex);//always return the right most index                
                best_angle_error =  angle_error;
                best_dist_error = distance_error  ;
                bestAngle=angle ;    
                bestDist = dist;
            }
           
            prev_left = left;
            prev_left_index = leftIndex;
        }      
    }
    angle_error =  best_angle_error;
    distance_error = best_dist_error;
    adjust_angle = bestAngle;
    return (bestIndex);
}

Point2D sum_difference(ArcNode* new_nodes, ArcNode* old_nodes,float left,float right,Point2D guess,float angleGuess,float& score,float& angle_diff,Point2D heading,float  maxAngle, int searchRange)
{
    bool lookingForward=true;
    Point2D total = {0,0};
    short matches=0;
    short anglematches=0;
    float total_distance_weight=0;
    float total_angle_weight=0;
    float angleChange=0;
    int aheadIndex = 0;
    bool isTurning = mag(heading)==0;
    if(heading.X > 0)
    {
        aheadIndex = (int)std::round( 0.25 * N_POINTS);
    }
    else if(heading.X < 0)
    {
        aheadIndex =  (int)std::round( 0.75 * N_POINTS);
    }
    else if(heading.Y < 0)
    {
        aheadIndex =  (int)std::round( 0.5 * N_POINTS);
    }
    float new_dist =new_nodes[aheadIndex].dist;
    float old_dist =old_nodes[aheadIndex].dist;
    float max_dist =old_dist;
  //  float simpleDistanceCalc = old_dist - new_dist;


    for(unsigned short i=0;i<N_POINTS;i++)
    {
        ArcNode* new_node = new_nodes+i;
        if(new_node->dist )
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
            if( (angleDiffFast(new_angle, left) > 0 && angleDiffFast(new_angle, right) < 0 ))
            {
                if(!lookingForward)
                {
                    lookingForward = true;
                    heading = mul(heading,-1);//will hit this once as it rotates around
                }
                //heading should be positive
            }
            else if(angleDiffFast(new_angle, left+M_PIF) > 0 && angleDiffFast(new_angle, right+M_PIF) < 0 )
            {
                if(lookingForward)
                {
                    lookingForward = false;
                    heading = mul(heading,-1);//will hit this once as it rotates around
                }
            }
            else
            {
                continue;
            }
                            
            ArcNode* next_node = new_nodes + ((i + 1) % N_POINTS);
            if(!next_node->dist)
                continue;

            float next_angle = next_node->angle + angleGuess;
        
            
            Point2D new_point = toPoint2D(new_angle,new_node->dist);
            int oldIndex = toIndex(new_angle,N_POINTS);
            float oldDist = mag(old_nodes[oldIndex].point);
            float est_dist_ratio = oldDist >0 ? mag(new_point)/oldDist : 1.0;

            Point2D expected_old_frame_point =add(new_point ,guess);                             
            
            Point2D next_point = toPoint2D(next_angle,next_node->dist);
            Point2D expected_next_old_frame_point = add(next_point ,guess);  

            float matchAngle = (angleGuess);
            Point2D matchdist = {0,0};
            float angle_error=0;
            float distance_error=0;
            int index = findAngleAndDistanceOffset(max_dist, est_dist_ratio,expected_old_frame_point,expected_next_old_frame_point,old_nodes,heading, new_angle ,matchAngle,matchdist,searchRange,angle_error,distance_error,maxAngle);
            
            if(index>=0)
            {                            
                float distMag = mag(matchdist);                
                
                // float distFromSimpleCalc=  abs(distMag-simpleDistanceCalc)/max_dist;
                //   distFromSimpleCalc = distFromSimpleCalc * distFromSimpleCalc;
                float angle_weight = angle_error == 0 ? 0 : 1/(angle_error  );
                float distance_weight =distance_error == 0 ? 0 : 1/(distance_error  );
                Point2D incDist = matchdist;
                float incDistDistMag =mag(incDist);

                matches++;
                angle_diff += matchAngle * angle_weight;
                
                score+=fabs(matchAngle) + distMag ;
            
                total.X += incDist.X * distance_weight;                           
                total.Y += incDist.Y * distance_weight;
                
                total_angle_weight += angle_weight;    
                total_distance_weight += distance_weight;                            
            }    
            
        }
    }
     if(total_distance_weight >0)
     {
        total.X /= total_distance_weight;
        total.Y /= total_distance_weight;
       
     }
     if(total_angle_weight > 0)
     { 
        angle_diff /= total_angle_weight;
        score /=matches;
     }
    return total;
}





bool find_y(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& guess,float& angle,float left, float right,float& std,Point2D heading)
{        
    std =0;
    int timeout=500;  
    
    float avgAngleError=0.0;
    float scoreCount=0;
    #ifndef TESTING
    unsigned long mills_start = millis();
    #endif
    while(true)
    {  
        #ifndef TESTING
        if( millis() - mills_start>timeout || isnan(guess.Y))                
           return false;    
        #endif
        float error_angle=0; 
      
        Point2D prev_guess = guess;
        Point2D error= sum_difference(nodes_new,nodes_old, left,right,guess,angle,std,error_angle,heading,oneDegree*5,2); 
         
       
        angle = PIDUpdate(&pid_move_rotate,-error_angle,angle);  
        guess.Y = PIDUpdate(&pid_move_master,error.Y,guess.Y); 
     
        serialPrintf2("guess_x: %.6f,guess_y: %.6f,  error_x: %.6f,error_y: %.6f, error_angle: %.6f, angle: %.6f",guess.X,guess.Y,error.X,error.Y,error_angle,angle);
       
        if(fabs(error.Y) < pid_move_master.deadband  )
        {
            break;
        } 
    }  

    return true;    
}

Point2D* map_nodes(ArcNode* nodes,Point2D* dst, Point2D trans, float rot,float left, float right, bool onlyVisible)
{   
 float const buffer = 0.0f;

    for(unsigned short i=0;i<N_POINTS;i++)
    {
        ArcNode* new_node = nodes+i;
        if(new_node->dist )
        {   
             float new_angle ;
            if(new_node->angle >= M_PIF)
            {
                new_angle =(new_node->angle-M_2PI) +  rot;
            }
            else
            {
                new_angle =new_node->angle +  rot;
            }

            if( (angleDiffFast(new_angle, left) > 0 && angleDiffFast(new_angle, right) < 0 ))
            {
                //heading should be positive
            }
            else if(angleDiffFast(new_angle, left+M_PIF) > 0 && angleDiffFast(new_angle, right+M_PIF) < 0 )
            {
            }
            else
            {
                dst[i].X = 0;
                dst[i].Y =0 ;
                continue;
            }
                
            ArcNode* next_node = nodes + ((i + 1) % N_POINTS);
            if(!next_node->dist)
                continue;

            float next_angle = next_node->angle + rot;
            
            Point2D new_point = toPoint2D(new_angle,new_node->dist);

            Point2D expected_old_frame_point =add(new_point ,trans);                             
            dst[i] = expected_old_frame_point;
           
        }
    }
}

bool find_x(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& guess, float& angle,float left, float right,float& std,Point2D heading)
{        
    std =0;
    int timeout=500;  
    
    float avgAngleError=0.0;
    float scoreCount=0;
    #ifndef TESTING
    unsigned long mills_start = millis();
    #endif
    while(true)
    {  
        #ifndef TESTING
        if( millis() - mills_start>timeout || isnan(guess.X))                
           return false;    
        #endif
        float error_angle=0; 
      
        Point2D prev_guess = guess;
        Point2D error= sum_difference(nodes_new,nodes_old, left,right,guess,angle,std,error_angle,heading,1.57,3); 
         
       
        angle = PIDUpdate(&pid_move_rotate,-error_angle,angle); 
      //  guess.Y = PIDUpdate(&pid_move_slave,error.Y,guess.Y); 
        guess.X = PIDUpdate(&pid_move_master,error.X,guess.X); 
     
        serialPrintf2("guess_x: %.6f,guess_y: %.6f,  error_x: %.6f,error_y: %.6f, error_angle: %.6f, angle: %.6f",guess.X,guess.Y,error.X,error.Y,error_angle,angle);
       
        if(fabs(error.X) < pid_move_master.deadband  )
        {
            break;
        } 
    }  

    return true;    
}

bool find_yaw(ArcNode* nodes_new, ArcNode* nodes_old,float left, float right, float assumed_yaw,float maxRange, float& outGuess)
{
    float  std =0;
    int timeout=50000;  
    
    float avgAngleError=0.0;
    float scoreCount=0;
    float guess= assumed_yaw;
    #ifndef TESTING
    unsigned long mills_start = millis();
    #endif
    while(true)
    {  
        #ifndef TESTING
        if( millis() - mills_start>timeout || isnan(guess))       
        {         
            outGuess= guess;
           return false;    
        }
        #endif
        float error_angle=0; 
      
        
        Point2D error= sum_difference(nodes_new,nodes_old, left,right,{0,0},guess,std,error_angle,{0,0},oneDegree*90,10); 
         
       
        guess = PIDUpdate(&pid_rotate,-error_angle,guess); 
        //guess.Y = PIDUpdate(&pid_rotate_move_y,error.Y,guess.Y);   
        //guess.X = PIDUpdate(&pid_rotate_move,error.X,guess.X); 
     
        serialPrintf2("error_x: %.6f,error_y: %.6f, error_angle: %.6f, angle: %.6f",error.X,error.Y,error_angle,guess);
       
        if(fabs(guess) < pid_rotate.deadband  )
        {
            break;
        } 
    }  
    outGuess = guess;
    return true;    
}


static inline float pointToSegmentDistSq(const Point2D &p, const Point2D &q, const Point2D &r, Point2D &closest) {
    float vx = q.X - p.X, vy = q.Y - p.Y;
    float wx = r.X - p.X, wy = r.Y - p.Y;
    float vlen2 = vx*vx + vy*vy;
    float t = (vlen2 > 1e-8f) ? (vx*wx + vy*wy) / vlen2 : 0.0f;
    t = fminf(fmaxf(t, 0.0f), 1.0f);
    closest.X = p.X + vx * t;
    closest.Y = p.Y + vy * t;
    float dx = r.X - closest.X;
    float dy = r.Y - closest.Y;
    return dx*dx + dy*dy;
}

/// <summary>
/// scan and insert data
/// </summary>
/// <param name="nodes"></param>
/// <param name="nodeCount"></param>
float read_scan(
    LidarScanNormalMeasureRaw* nodes,
    ArcNode* dst,
    unsigned short nodeCount,
    float new_weight ,
    bool calculatePoints )
{
    float totalDiff = 0.0f;
    const float ANG_STEP = M_2PI / N_POINTS;
    const float ON_SEG_TOL = 0.5f;    // mm tolerance for “on segment”
    const float NEAR_SEG_TOL = 20.0f; // mm tolerance for “near segment”

    for (int i = 0; i < nodeCount; ++i)
    {
       // serialPrintf2("quality:%i angle:%i dist:%i",nodes[i].quality,nodes[i].angle_z_q6,nodes[i].dist_mm_q2);
        // Continuous angle from LiDAR
        float angle = (nodes[i].angle_z_q6 / 128.0f) * M_PI_F / 180.0f;
        while (angle >= M_2PI) angle -= M_2PI;

        float dist = nodes[i].dist_mm_q2 / 4.0f;
        dist = pow(dist, (1 - (0.06f * std::fmax(0.0f, 150.0f - dist)) / 150.0f));// * 95/98;
        if (dist <= 0.0f)
            continue;

        unsigned short index = toIndex(angle, N_POINTS);
        ArcNode &curr = dst[index];

        // Initialize missing node
        if (curr.dist <= 0.0f)
        {
            curr.angle = index * ANG_STEP;
            curr.dist = dist;
            if (calculatePoints)
                curr.point = toPoint2D(curr.angle, dist);
            continue;
        }

        // --- build left and right segments (index-1→index, index→index+1) ---
        int leftIdx = (index - 1 + N_POINTS) % N_POINTS;
        int rightIdx = (index + 1) % N_POINTS;

        bool haveLeft = dst[leftIdx].dist > 0.0f;
        bool haveRight = dst[rightIdx].dist > 0.0f;

        Point2D pNew = toPoint2D(angle, dist);
        float bestPerpDist = 1e9f;
        float segmentDist = curr.dist;

        // Compute perpendicular distance to left segment
        if (haveLeft)
        {
            Point2D pL = toPoint2D(dst[leftIdx].angle, dst[leftIdx].dist);
            Point2D pC = toPoint2D(curr.angle, curr.dist);
            Point2D closest;
            float d2 = pointToSegmentDistSq(pL, pC, pNew, closest);
            float d = sqrtf(d2);
            if (d < bestPerpDist)
            {
                bestPerpDist = d;
                // interpolate expected distance based on projection ratio
                float vx = pC.X - pL.X;
                float vy = pC.Y - pL.Y;
                float wx = pNew.X - pL.X;
                float wy = pNew.Y - pL.Y;
                float t = (vx * wx + vy * wy) / (vx * vx + vy * vy + 1e-8f);
                t = fminf(fmaxf(t, 0.0f), 1.0f);
                segmentDist = dst[leftIdx].dist + t * (curr.dist - dst[leftIdx].dist);
            }
        }

        // Compute perpendicular distance to right segment
        if (haveRight)
        {
            Point2D pC = toPoint2D(curr.angle, curr.dist);
            Point2D pR = toPoint2D(dst[rightIdx].angle, dst[rightIdx].dist);
            Point2D closest;
            float d2 = pointToSegmentDistSq(pC, pR, pNew, closest);
            float d = sqrtf(d2);
            if (d < bestPerpDist)
            {
                bestPerpDist = d;
                float vx = pR.X - pC.X;
                float vy = pR.Y - pC.Y;
                float wx = pNew.X - pC.X;
                float wy = pNew.Y - pC.Y;
                float t = (vx * wx + vy * wy) / (vx * vx + vy * vy + 1e-8f);
                t = fminf(fmaxf(t, 0.0f), 1.0f);
                segmentDist = curr.dist + t * (dst[rightIdx].dist - curr.dist);
            }
        }

        // If no valid neighbors, fallback to direct update
        if (!haveLeft && !haveRight)
        {
            totalDiff += fabs(curr.dist - dist);
            curr.dist = curr.dist * (1.0f - new_weight) + dist * new_weight;
            if (calculatePoints)
                curr.point = toPoint2D(curr.angle, curr.dist);
            continue;
        }

        // --- decide blending weight based on perpendicular distance ---
        float w = new_weight;
        if (bestPerpDist < ON_SEG_TOL)
        {
          //  serialPrintf2("on seg");
            // On segment → barely change
            w = 0;//0.1f * new_weight;
        }
        else if (bestPerpDist < NEAR_SEG_TOL)
        {
         //   serialPrintf2("near seg");
            // Near segment → soft update
            float t = bestPerpDist / NEAR_SEG_TOL;
            w = new_weight * (0.3f + 0.7f * t);
        }
        else
        {
           // serialPrintf2("far seg");
            // Far away → treat as new structure
            w = new_weight;
        }

        totalDiff += fabs(dist - segmentDist);
        curr.dist = curr.dist * (1.0f - w) + dist * w;

        if (calculatePoints)
            curr.point = toPoint2D(curr.angle, curr.dist);
    }

    return totalDiff;
}
