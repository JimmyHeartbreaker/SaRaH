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



const float kp_rotate = 1.5f;
pid_state pid_rotate = 
    { 
        kp_rotate,  //kp
        0.00f * kp_rotate , //ki
        0.0* kp_rotate,//0.01f * kp_rotate, //kd
        M_PI, //max rotate
        -M_PI, //min rotate
        360.0f/N_POINTS, //dt
        5, //Tt
        0.0,// RESOLUTION,  //deadband
        0.0, //integral
        0.0, //prev error
    };

const float kp_rotate_translate = 0.1f;
pid_state pid_rotate_translate = 
    { 
        kp_rotate_translate,  //kp
      0,//  0.01f *kp_move_slave , //ki
      0,//  0.07f * kp_move_slave, //kd
        10, //max
        -10, //min
        0.1f, //dt
        5000, //Tt
        0,  //deadband
        0.0, //integral
        0.0, //prev error
    };

const float kp_y = 0.3f;
pid_state pid_y = 
    { 
        kp_y,  //kp
       0.01f *kp_y , //ki
       0.0f * kp_y, //kd
        2000, //max dist
        -2000, //min dist
        0.1f, //dt
        1, //Tt
        0.1,  //deadband
        0.0, //integral
        0.0, //prev error
    };

    const float kp_x = 0.3f;
pid_state pid_x = 
    { 
        kp_x,  //kp
       0.01f *kp_x , //ki
       0.0f * kp_x, //kd
        2000, //max dist
        -2000, //min dist
        0.1f, //dt
        1, //Tt
        0.1,  //deadband
        0.0, //integral
        0.0, //prev error
    };
const float kp_translate_rotate = 0.1f;
pid_state pid_translate_rotate = 
    { 
        kp_translate_rotate,  //kp
        0.01f * kp_translate_rotate , //ki
        0.0f * kp_translate_rotate, //kd
        20* M_PI/180, //max rotate
        -20* M_PI/180, //min rotate
        360.0f/N_POINTS, //dt
        5, //Tt
        0,  //deadband
        0.0, //integral
        0.0, //prev error
    };


/// <summary>
/// consult uni notes
/// </summary>
/// <param name="y"></param>
/// <returns></returns>
float PIDUpdate(pid_state* pid, float y,float prev_input)
{
	float error = y;
	// if (fabs(y) <= pid->deadband)
	// {
	// 	e = 0.0;
	// }
	float de_dt = (error - pid->prev_error) / pid->dt;

	float	input_delta = 
                    pid->kp * error + 
                    pid->ki * pid->integral + 
                    pid->kd * de_dt;
	
	float new_input = std::fmax(pid->min, std::fmin(input_delta + prev_input, pid->max));

	//	# anti - windup correction;
	pid->integral += pid->dt * error;// + (new_input - (input_delta+prev_input)) / pid->Tt;

	pid->prev_error = error;
	return new_input;
}

void calculate_angle_and_direction(float distanceRatio,float attenuatioNew,float max_Dist, float strengthAdj,const Point2D& n,const Point2D& v,const Point2D& midPoint,ArcNode* left, ArcNode* right,float& angle_error,float& distance_error,float& angle,Point2D& dist,float maxAngle)
{ 
    Point2D leftP = left->point;
    Point2D rightP = right->point;
    float cross_origin = leftP.X * rightP.Y - leftP.Y * rightP.X;
    if (cross_origin > 0.0f) {
        // points are anti clockwise; swap them
        leftP = right->point;
        rightP = left->point;
       // serialPrintf2("ref swap points");
    }

    Point2D ref_v = sub(rightP,leftP);   
    Point2D ref_vn = normalize(ref_v);
    Point2D ref_midPoint = mul(add(rightP,leftP),0.5);  
    float ref_midPointMag = mag(ref_midPoint);
    Point2D ref_midPoint_norm = div(ref_midPoint,ref_midPointMag);
    
    Point2D ref_n =  normal_ccw(ref_vn);    
    

    Point2D midPointDiff =    sub(midPoint,ref_midPoint) ;    
    
    // Point2D toOrigin = normalize( mul(ref_midPoint, -1.0f)); 
    // if (dot(ref_n, toOrigin) > 0.0f) {
    //     ref_vn = mul(ref_vn, -1.0f);
    //     serialPrintf2("ref swap vector");
    // }  

    float distanceToLine =  (cross_scalar(midPointDiff,ref_vn) * attenuatioNew)* ( 1-strengthAdj) ;


    Point2D ref_midPoint_normed = div(ref_midPoint,ref_midPointMag);
    dist = mul(ref_midPoint_normed,distanceToLine); 
    
    float norm_angle =  angleBetween(n,ref_n);    
    float normal_angle_error = norm_angle*norm_angle ; //smaller better
    angle=norm_angle * ( 1-strengthAdj) ;   

    float distanceRemainingRatio = distanceToLine/max_Dist; //smaller better
    distanceRemainingRatio*= distanceRemainingRatio;
    distance_error =  normal_angle_error  + distanceRemainingRatio  +   strengthAdj ;      
    angle_error =    normal_angle_error +   strengthAdj;
   
}

int findAngleAndDistanceOffset(float max_dist, float est_dist_ratio,Point2D& nl, Point2D& nr, ArcNode* old_nodes,float sample_angle, float& adjust_angle,Point2D& bestDist, int searchSize,float& angle_error, float& distance_error,float maxAngle)
{
    float cross_origin = nl.X *nr.Y - nl.Y * nr.X;
    if (cross_origin > 0.0f) {
        // points are anti clockwise; swap them
        Point2D tmp =nl;
       nl= nr;
       nr = tmp;
      //  serialPrintf2("new swap points");
    }

    Point2D v = sub(nr,nl);
    Point2D vn = normalize(v);
    Point2D midPoint = mul(add(nl,nr),0.5);
    float magMidp = mag(midPoint);
    Point2D midPoint_norm = div(midPoint,magMidp);  

    Point2D n = normal_ccw(vn);
    
     float nmp_angle = angleBetween(n,midPoint_norm);     
    #ifdef TESTING
   float attenuationNew=1.0;//no lighting attenuaton in simulation
   #else
    float attenuationNew = 1-fabs(nmp_angle);
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
            calculate_angle_and_direction(distanceRatio,attenuationNew,max_dist,strengthAdj,n,v,midPoint,prev_right,right,angle_error,distance_error,angle,dist,maxAngle);
       
              if( (distance_error  < best_dist_error ) && fabs(angle)<maxAngle)
                {
                    bestIndex = (rightIndex);//always return the right most index                
                    best_angle_error =  angle_error +  (i-1) * old_sample_step_size;
                    best_dist_error = distance_error;
                    bestAngle= angle+  (i-1) * old_sample_step_size;     
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
            calculate_angle_and_direction(distanceRatio,attenuationNew,max_dist, strengthAdj,n,v,midPoint,left,prev_left,angle_error,distance_error,angle,dist,maxAngle);
     
            if( (distance_error  < best_dist_error ) && fabs(angle)<maxAngle)
            {
                bestIndex = (rightIndex);//always return the right most index                
                best_angle_error =  angle_error +  (i-1) * old_sample_step_size;
                best_dist_error = distance_error  ;
                bestAngle=angle - (i-1) * old_sample_step_size;    
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

Point2D sum_difference(ArcNode* new_nodes, ArcNode* old_nodes,Point2D guess,float angleGuess,float& angle_diff,float  maxAngle, int searchRange)
{
    Point2D total = {0,0};
    float total_distance_weight=0;
    float total_angle_weight=0;
    
    float max_dist =0;
    for(int i =0;i<N_POINTS;i++)
    {
        max_dist += old_nodes[i].dist;
    }
    max_dist /= N_POINTS;

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
            int index = findAngleAndDistanceOffset(max_dist, est_dist_ratio,expected_old_frame_point,expected_next_old_frame_point,old_nodes, new_angle ,matchAngle,matchdist,searchRange,angle_error,distance_error,maxAngle);
            
            if(index>=0)
            {                            
                float distMag = mag(matchdist);                
                
                float angle_weight = angle_error == 0 ? 0 : 1/angle_error;
                float distance_weight =distance_error == 0 ? 0 : 1/distance_error;

                angle_diff += matchAngle * angle_weight;                
            
                total.X += matchdist.X * distance_weight;                           
                total.Y += matchdist.Y * distance_weight;
                
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
     }
    return total;
}

Point2D* map_nodes(ArcNode* nodes,Point2D* dst, Point2D trans, float rot, bool onlyVisible)
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

bool find_pos(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& pos, float& angle)
{        
    int timeout=500;  
    
    float avgAngleError=0.0;
    float scoreCount=0;
    #ifndef TESTING
    unsigned long mills_start = millis();
    #endif
    while(true)
    {  
        #ifndef TESTING
        if( millis() - mills_start>timeout || isnan(pos.X) || isnan(pos.Y))                
           return false;    
        #endif
        float error_angle=0; 
      
        Point2D error= sum_difference(nodes_new,nodes_old, pos,angle,error_angle,1.57,3);          
       
        angle = PIDUpdate(&pid_translate_rotate,-error_angle,angle); 
        pos.Y = PIDUpdate(&pid_y,error.Y,pos.Y); 
        pos.X = PIDUpdate(&pid_x,error.X,pos.X); 
     
        serialPrintf2("guess_x: %.6f,guess_y: %.6f,  error_x: %.6f,error_y: %.6f, error_angle: %.6f, angle: %.6f, angle integral %.6f",pos.X,pos.Y,error.X,error.Y,error_angle,angle,pid_translate_rotate.integral);
       
        if(mag(error) < pid_x.deadband  )
        {
            break;
        } 
    }  

    return true;    
}

bool find_yaw(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& pos,float& angle)
{
    int timeout=500;  
    
    float avgAngleError=0.0;
    float scoreCount=0;
    #ifndef TESTING
    unsigned long mills_start = millis();
    #endif
    while(true)
    {  
        #ifndef TESTING
        if( millis() - mills_start>timeout || isnan(angle) || isnan(pos.X) || isnan(pos.Y))                
           return false;    
        #endif
        float error_angle=0; 
      
        Point2D error= sum_difference(nodes_new,nodes_old, pos,angle,error_angle,1.57,5); 
         
       
        angle = PIDUpdate(&pid_rotate,-error_angle,angle); 
        pos.Y = PIDUpdate(&pid_rotate_translate,error.Y,pos.Y); 
        pos.X = PIDUpdate(&pid_rotate_translate,error.X,pos.X); 
     
        serialPrintf2("guess_x: %.6f,guess_y: %.6f,  error_x: %.6f,error_y: %.6f, error_angle: %.6f, angle: %.6f",pos.X,pos.Y,error.X,error.Y,error_angle,angle);
       
        if(fabs(error.X) < pid_rotate.deadband  )
        {
            break;
        } 
    }  

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

        unsigned char quality = nodes[i].quality >> 2;
        if(quality < 15)
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
