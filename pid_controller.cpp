#include "NodeServices.h"
#include "pid_controller.h"
#ifndef TESTING
#include "arduino_ext.h"
#include "arduino.h"
#endif
#include "robot_math.h"


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
const float kp_move_rotate = 0.05f;
pid_state pid_move_rotate = 
    { 
        kp_move_rotate,  //kp
        0.0f * kp_move_rotate , //ki
        0.0f * kp_move_rotate, //kd
        5* M_PI/180, //max rotate
        -5* M_PI/180, //min rotate
        360.0f/N_POINTS, //dt
        5, //Tt
        0.0,  //deadband
        0.0, //integral
        0.0, //prev error
    };

const float kp_rotate = 0.01f;
pid_state pid_rotate = 
    { 
        kp_rotate,  //kp
        0.0f * kp_rotate , //ki
        0,//0.01f * kp_rotate, //kd
        M_PI, //max rotate
        -M_PI, //min rotate
        360.0f/N_POINTS, //dt
        5, //Tt
        RESOLUTION,  //deadband
        0.0, //integral
        0.0, //prev error
    };

const float kp_move_master = 0.2f;
pid_state pid_move_master = 
    { 
        kp_move_master,  //kp
       -0.0f *kp_move_master , //ki
       0.00f * kp_move_master, //kd
        200, //max dist
        -200, //min dist
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






void find_dir(const float nmp_angle,const Point2D& n,const Point2D& midPoint, const Point2D& heading,ArcNode* left, ArcNode* right,float& error,float& angle,Point2D& dist)
{
    Point2D cv = sub(right->point,left->point);
    Point2D cn = normalize(normal_ccw(cv));      
    Point2D cmidPoint = mul(add(right->point,left->point),0.5);  
    float cnmp_angle = angleBetween(cn,normalize(cmidPoint));

    float norm_angle = angleBetween(n,cn);

    float distanceToLine = cross_scalar(sub(midPoint,cmidPoint),cv) / mag(cv);
    error = fabs(distanceToLine) * 0.001 + fabs(norm_angle)*10 ; //bias towards 0

    if(error<0.5)
    {
          //serialPrintf2("distanceToLine %.6f error %.6f midPointX %.6f midPointY %.6f cmidPointX %.6f cmidPointY %.6f norm_angle %.6f" ,distanceToLine,error,midPoint.X,midPoint.Y,cmidPoint.X,cmidPoint.Y,norm_angle);
    }       
    angle=norm_angle;
    
    dist = mul(heading,distanceToLine/cos(cnmp_angle));
       
}

int findAngleAndDistanceOffset(float old_sample_step_size,Point2D& nl, Point2D& nr, ArcNode* old_nodes,Point2D heading,float sample_angle, float& adjust_angle,Point2D& bestDist, int searchSize,Point2D buffer)
{
    Point2D v = sub(nr,nl);
    Point2D n = normalize(normal_ccw(v));
    Point2D midPoint = mul(add(nl,nr),0.5);
    float nmp_angle = angleBetween(n,normalize(midPoint));     
    
    float best_error = 1000000;        
    
    short index =  toIndex(sample_angle ,N_POINTS);
    short bestIndex =-1;
    float bestAngle=0;
    ArcNode* prev_left = old_nodes + index;
    ArcNode* prev_right = prev_left;
   
    for (int i = 1; i <searchSize; i++)
    {                  
        float nextRightAngle = sample_angle + old_sample_step_size*i;
        int rightIndex =  toIndex(nextRightAngle ,N_POINTS);
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
            find_dir(nmp_angle,n,midPoint,heading,prev_right,right,error,angle,dist);
              if(error < best_error && error < 0.5)
                {
                    bestIndex = (rightIndex);//always return the right most index
                    best_error = error;
                    bestAngle=angle;     
                    bestDist = dist;

                    if(error < 0.01)
                    {
                        break;    
                    }
                }
            
            prev_right = right;
        }

         float nextLeftAngle = sample_angle - old_sample_step_size*i;
        int leftIndex =  toIndex(nextLeftAngle ,N_POINTS);
        if (leftIndex < 0)
        {
            leftIndex += N_POINTS;
        }        
       
        ArcNode* left = (old_nodes + leftIndex);
         if(left->dist && prev_left->dist)
        {
            float error;
            float angle;
            Point2D dist;
            find_dir(nmp_angle,n,midPoint,heading,left,prev_left,error,angle,dist);
            if(error < best_error && error < 0.5)
            {
                bestIndex = (leftIndex+1);
                best_error = error;
                bestAngle=angle;               
                bestDist = dist;            
                
                if(error < 0.01)
                {
                    break;    
                }
            }
           
            prev_left = left;
        }
       
      
    }
    adjust_angle = bestAngle;
    return (bestIndex);
}


int findAngle(Point2D& nl, Point2D& nr, ArcNode* old_nodes,float sample_angle, float& adjust_angle, int searchSize)
{
    Point2D v = sub(nr,nl);
    Point2D n = normalize(normal_ccw(v));
    Point2D midPoint = mul(add(nl,nr),0.5);
    float nmp_angle = angleBetween(n,normalize(midPoint));     
    
    float best_error = 1000000;    
    
    short index =  toIndex(sample_angle ,N_POINTS);
    short bestIndex =-1;
    float bestAngle=0;
    ArcNode* prev_left = old_nodes + index;
    ArcNode* prev_right = prev_left;

    for (int i = 1; i <searchSize; i++)
    {         
          int rightIndex = index + i;
        if (rightIndex >= N_POINTS)
        {
            rightIndex -= N_POINTS;
        }

        ArcNode* right = (old_nodes + rightIndex);
        if(right->dist && prev_right->dist)
        {
            Point2D cv = sub(right->point,prev_right->point);
            Point2D cn = normalize(normal_ccw(cv));      
            Point2D cmidPoint = mul(add(right->point,prev_right->point),0.5);  
            float angleDiff = angleBetween(n,cn);
        
            float error = fabs(angleDiff);
            if( error < best_error  )
            {                    
                bestIndex = rightIndex;
                best_error = error;
                bestAngle=angleDiff;   
                
                if(error < 0.01)
                {
                    break;    
                }
            }
            prev_right = right;
        }
        
        int leftIndex = index - i;
        if (leftIndex < 0)
        {
            leftIndex += N_POINTS;
        }
        ArcNode* left = (old_nodes + leftIndex);
         if(left->dist && prev_left->dist)
        {
            Point2D cv = sub(prev_left->point,left->point);
            
            Point2D cn = normalize(normal_ccw(cv));
            
            Point2D cmidPoint = mul(add(left->point,prev_left->point),0.5);
        
           float angleDiff = angleBetween(n,cn);

            float error = fabs(angleDiff); 
            if(error < best_error  )
            {                    
                bestIndex = (leftIndex+1);
                best_error = error;
                bestAngle=angleDiff;    
                
                if(error < 0.01)
                {
                    break;    
                }
            }
            prev_left = left;
        }
      
    }
    adjust_angle = bestAngle;
    return (bestIndex);
}

Point2D sum_difference(ArcNode* new_nodes, ArcNode* old_nodes,float left,float right,Point2D guess,float angleGuess,float& score,float& angle_diff,Point2D heading, int searchRange)
{
                       // serialPrintf2("BEGIN");
    Point2D total = {0,0};
    short matches=0;
    short anglematches=0;
    float totalWight=0;
    Point2D runningAvgDist={0,0};
    Point2D prev_new_normal={0,0};
    float angleChange=0;
    int aheadIndex = 0;
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
    float old_sample_step_size_est = mag(new_nodes[aheadIndex].point)/mag(old_nodes[aheadIndex].point)  * RESOLUTION;
   
    //serialPrintf2("old_sample_step_size_est %.6f",old_sample_step_size_est);
 float const buffer = 0.05f;

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
                
                
                ArcNode* next_node = new_nodes + ((i + 1) % N_POINTS);
                if(!next_node->dist)
                    continue;


                float next_angle = next_node->angle + angleGuess;
                Point2D offsetBuffer = mul(guess,buffer);
                Point2D offset = mul(guess,1-buffer);
                
                Point2D new_point = toPoint2D(new_angle,new_node->dist);
                Point2D expected_old_frame_point =add(new_point ,offset);                             
               
                Point2D next_point = toPoint2D(next_angle,next_node->dist);
                Point2D expected_next_old_frame_point = add(next_point ,offset);  

                Point2D expected_v = sub(expected_next_old_frame_point,expected_old_frame_point); 
                Point2D _n = normal(expected_v);                    
                Point2D expected_n = normalize(_n);//clockwise
                angleChange =  angleChange*0.3 + angleBetween(prev_new_normal,expected_n)*0.7;
                float angleToHeading;
                //corner 
            //     if(mag(prev_new_normal) >0 && fabs(angleChange)> 0.7 )
            //     { 
            //         serialPrintf2("corner? %.2f skip",angleChange);
            //         prev_new_normal = expected_n;
            //         score++;
            //         continue;
            //         //looks like a corner
            //    //    
            //     }
            //     else
            //     {
                    prev_new_normal = expected_n;
                    float matchAngle = (angleGuess);
                    Point2D matchdist = {0,0};

                    //serialPrintf2("heading x:%.6f heading y:%.6f expected_old_frame_point x %.6f y %.6f expected_next_old_frame_point x %.6f y %.6f",heading.X, heading.Y,expected_old_frame_point.X,expected_old_frame_point.Y,expected_next_old_frame_point.X,expected_next_old_frame_point.Y);
                    int index = findAngleAndDistanceOffset(old_sample_step_size_est,expected_old_frame_point,expected_next_old_frame_point,old_nodes,heading, new_angle ,matchAngle,matchdist,searchRange,offsetBuffer);
                    
                    if(index>=0)
                    { 
                        //start artifact filtering
                        float errorDot = dot(expected_n,heading);
                        if(errorDot>-0.5)
                        {
                            serialPrintf2("bad angle, nx %.2f ny %.2f matchdist %.2f errorDot %.2f",expected_n.X,expected_n.Y,matchdist.X,errorDot);
                            continue;
                        }
                            
                       // 
                        
                        float distMag = mag(matchdist);
                        float magrunningAvgDist = mag(runningAvgDist);
                        if(magrunningAvgDist == 0 )
                        {
                            runningAvgDist = matchdist;
                            magrunningAvgDist = distMag;
                        }

                        float deltaDist = dot(runningAvgDist,matchdist);
                        float weight =  mag(expected_old_frame_point);
                        
                        if(deltaDist > -1 || distMag==0)//make sure we didnt swap suddenly
                        {
                            Point2D incDist = sub(matchdist,offsetBuffer);
                            float incDistDistMag =mag(incDist);
                          //  if( fabs( magrunningAvgDist - incDistDistMag )/ incDistDistMag  < 2 )
                           // {
                                runningAvgDist = mul(add(runningAvgDist,matchdist),0.5f);
                                matches++;
                                angle_diff += ( matchAngle);
                                score+=matchAngle;
                                score+=incDistDistMag*0.001;
                            
                                total.X += incDist.X * weight;                           
                                total.Y += incDist.Y *weight;
                                
                                
                                totalWight += weight;    
                          //  }                              
                          //  else
                          //  {
                          //      score++;
                         //   }
                        } 
                         
                    }                 
               // }
            }
        }
    }
     if(matches >0)
     {
        total.X /= totalWight;
        total.Y /= totalWight;
        score /= matches;
     }
     if(matches > 0)
     {
        angle_diff /= matches;
     }
    // serialPrintf2("matches:%i totalWight:%i",matches,totalWight);
    return total;
}


Point2D sum_difference_rotation(ArcNode* new_nodes, ArcNode* old_nodes,float left,float right,float angleGuess,float& angle_diff, int searchRange)
{
    Point2D total = {0,0};
    short matches=0;
    short anglematches=0;
    float totalWight=0;
    Point2D runningAvgDist={0,0};
    float angleChange=0;
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
                

                ArcNode* next_node = new_nodes + ((i + 1) % N_POINTS);
                if(!next_node->dist)
                    continue;

                float next_angle = next_node->angle + angleGuess;
               
                
                Point2D new_point = toPoint2D(new_angle,new_node->dist);
                Point2D expected_old_frame_point =new_point;                             
               
                Point2D next_point = toPoint2D(next_angle,next_node->dist);
                Point2D expected_next_old_frame_point = next_point ;  
               
                float matchAngle = 0;
                //serialPrintf2("heading x:%.6f heading y:%.6f expected_old_frame_point x %.6f y %.6f expected_next_old_frame_point x %.6f y %.6f",heading.X, heading.Y,expected_old_frame_point.X,expected_old_frame_point.Y,expected_next_old_frame_point.X,expected_next_old_frame_point.Y);
                int index = findAngle(expected_old_frame_point,expected_next_old_frame_point,old_nodes, new_angle ,matchAngle,searchRange);
                if(index>=0 && fabs(matchAngle) < 1)
                { 
                    matches++;
                    angle_diff += ( matchAngle);                        
                }    
                else
                {
                    totalWight += 0.00001;
                }                    
                
            }
        }
    }
     
     if(matches > 0)
     {
        angle_diff /= matches;
     }
    // serialPrintf2("matches:%i totalWight:%i",matches,totalWight);
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
        if( millis() - mills_start>timeout || isnan(guess.Y))                
           return false;    
        #endif
        float error_angle=0; 
      
        Point2D prev_guess = guess;
        Point2D error= sum_difference(nodes_new,nodes_old, left,right,guess,angle,std,error_angle,heading); 
        if(fabs(error.Y) > pid_move_master.max  ||fabs(guess.X) > pid_move_slave.max )        
            guess = add(prev_guess,{1e-6,1e-6});        

        avgAngleError = avgAngleError == 0 ? error_angle : (avgAngleError*0.8 + error_angle*0.2);           
       
        angle = PIDUpdate(&pid_move_rotate,-avgAngleError,angle); 
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
        Point2D error= sum_difference(nodes_new,nodes_old, left,right,guess,angle,std,error_angle,heading,2); 
    //     if(std> 4)
    //     {            
    //         scoreCount+=std;  
    //         if(scoreCount>40)
    //         { 
    //             //serialPrintf2("match failed, bad data:%.6f",scoreCount);
    //           //  return false;
    //    //get new data next round     
    //         } 
    //         continue;
    //     }
        
       // avgAngleError = avgAngleError == 0 ? error_angle : (avgAngleError*0.8 + error_angle*0.2);           
       
        angle = PIDUpdate(&pid_move_rotate,-error_angle,angle); 
       // guess.Y = PIDUpdate(&pid_move_slave,error.Y,guess.Y);   
        guess.X = PIDUpdate(&pid_move_master,error.X,guess.X); 
        
        if(fabs(guess.X) >= pid_move_master.max  ||fabs(guess.Y) >= pid_move_slave.max )    
        {    
            guess = add(prev_guess,{1e-6,1e-6});            
        }
      //  serialPrintf2("guess_x: %.6f,guess_y: %.6f,  error_x: %.6f,error_y: %.6f, error_angle: %.6f, angle: %.6f",guess.X,guess.Y,error.X,error.Y,error_angle,angle);
       
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
    
    // 
    #ifndef TESTING
    unsigned long mills_start = millis();
    #endif
   // Serial.println("1");
    while(true)
    {
         #ifndef TESTING
       if( millis() - mills_start>timeout || isnan(guess_yaw))   
        {            
            outGuess = guess_yaw  ;           
            return false;     
        }
        #endif 
        Point2D xyguess={0,0};
        float score;
        float errorAngle=0;
        Point2D xyerror= sum_difference_rotation(nodes_new,nodes_old, left,right,guess_yaw,errorAngle,15);    
      

        if(fabs(errorAngle)< pid_rotate.deadband)
        {
     
            break;
        }
        guess_yaw = PIDUpdate(&pid_rotate,-errorAngle,guess_yaw);     
        serialPrintf2("error: %.6f, guess_yaw:%.6f",errorAngle,guess_yaw);
    }
   
    outGuess = guess_yaw;
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
        dist = pow(dist, (1 - (0.06f * std::fmax(0.0f, 150.0f - dist)) / 150.0f));
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
