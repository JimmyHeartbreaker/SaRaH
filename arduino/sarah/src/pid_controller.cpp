#include "..\headers\NodeServices.h"
#include "..\headers\pid_controller.h"
#ifndef TESTING
#include "..\headers\arduino_ext.h"
#include "arduino.h"
#endif
#include "..\headers\robot_math.h"
#include <vector>
#include <algorithm>
#include <cmath>


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



const float kp_rotate = 0.6f;
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

const float kp_rotate_translate = 0.3f;
pid_state pid_rotate_translate = 
    { 
        kp_rotate_translate,  //kp
      0,//  0.01f *kp_move_slave , //ki
      0,//  0.07f * kp_move_slave, //kd
        20, //max
        -20, //min
        0.1f, //dt
        5000, //Tt
        0.01,  //deadband
        0.0, //integral
        0.0, //prev error
    };

const float kp_y = 0.05f;
pid_state pid_y = 
    { 
        kp_y,  //kp
       0.0f *kp_y , //ki
       0.0f * kp_y, //kd
        2000, //max dist
        -2000, //min dist
        0.1f, //dt
        1, //Tt
        0.001,  //deadband
        0.0, //integral
        0.0, //prev error
    };

    const float kp_x = 1.0f;
pid_state pid_x = 
    { 
        kp_x,  //kp
       0.0f *kp_x , //ki
       0.0f * kp_x, //kd
        2000, //max dist
        -2000, //min dist
        0.1f, //dt
        1, //Tt
        0.001,  //deadband
        0.0, //integral
        0.0, //prev error
    };
const float kp_translate_rotate = 0.1f;
pid_state pid_translate_rotate = 
    { 
        kp_translate_rotate,  //kp
        0.0f * kp_translate_rotate , //ki
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

struct ScanResult
{
    float angle_error;
    float angle;
    Point2D distance_error;
    Point2D distance;
};


inline void calculate_angle_and_direction(float attenuatioNew,float avg_dist_inv, float strengthAdj,const Point2D& n,const Point2D& midPoint,ArcNode* ref_left, ArcNode* ref_right,ScanResult& scanResult,const Point2D& heading )
{ 
      Point2D& leftP = ref_left->point;
    Point2D& rightP = ref_right->point;
   
   
    Point2D ref_v = sub(rightP,leftP);
    Point2D ref_midPoint = mul(add(rightP,leftP),0.5);  
    float ref_midPointmag = mag(ref_midPoint);
    Point2D vectorAlong = div(ref_midPoint,ref_midPointmag);
  
    
   
    float distanceToLine =  (cross_scalar( sub(midPoint,ref_midPoint) ,normalize(ref_v)) * attenuatioNew)* ( 1-strengthAdj) ;
    
    float errorToheading = dot(heading,vectorAlong);
        scanResult.distance = mul(normalize(ref_midPoint),distanceToLine); 
    //if(errorToheading < 0 && ref_midPointmag < 10)
   // {
      //  scanResult.distance = mul(normalize(ref_midPoint),-distanceToLine); 
        // if(heading.X==0)
        // {
        //     scanResult.distance.Y = -scanResult.distance.Y;
        // }
        // else
        // {
        //     scanResult.distance.X = -scanResult.distance.X;
        // }
   // }
    float invErrorToheading = 1 - errorToheading;
    errorToheading *= errorToheading;
    invErrorToheading *= invErrorToheading;
   //  serialPrintf2("X:%.6f, Y:%.6f",scanResult.distance.X,scanResult.distance.Y);  
    
    
    float norm_angle =  angleBetween(n,normal_ccw(ref_v));    
    float normal_angle_error = norm_angle*norm_angle ; //smaller better
    scanResult.angle=norm_angle * ( 1-strengthAdj) ;   

    float distanceRemainingRatio = distanceToLine * avg_dist_inv; //smaller better
    distanceRemainingRatio*= distanceRemainingRatio;
    scanResult.distance_error.Y =   distanceRemainingRatio  +   strengthAdj + errorToheading;     
    scanResult.distance_error.X = std::fmax(0.01f,  normal_angle_error  + distanceRemainingRatio  +   strengthAdj + invErrorToheading);      
    scanResult.angle_error =   normal_angle_error +   strengthAdj;

//      Point2D& leftP = ref_left->point;
//      Point2D& rightP = ref_right->point;
//     // float cross_origin = leftP.X * rightP.Y - leftP.Y * rightP.X;
//     // if (cross_origin > 0.0f) {
//     //     // points are anti clockwise; swap them
//     //     leftP = ref_right->point;
//     //     rightP = ref_left->point;
//     //     Serial.println("swap1");
//     // }

//     Point2D ref_v = sub(rightP,leftP);
//     Point2D ref_midPoint = mul(add(rightP,leftP),0.5);  
//   //  float midPM = size(midPoint);
//     float ref_midpm = mag(ref_midPoint);
//     float distanceToLine = (cross_scalar( sub(midPoint,ref_midPoint) , normalize(ref_v)) * attenuatioNew)* ( 1-strengthAdj) ;
//     

//     //  if(midPM < (ref_midpm))
//     //  {
//         scanResult.distance = mul(vectorAlong,distanceToLine); 
//     //  }
//     //  else
//     //  {
//     //      scanResult.distance = mul(vectorAlong,-fabs(distanceToLine)); 
//     //  }

//     float norm_angle =  angleBetween(n,normal_ccw(ref_v));    
//     float normal_angle_error = norm_angle*norm_angle ; //smaller better
//      scanResult.angle=norm_angle * ( 1-strengthAdj) ;   

//     float distanceRemainingRatio = distanceToLine * avg_dist_inv; //smaller better
//     distanceRemainingRatio*= distanceRemainingRatio;
//     scanResult.distance_error =  normal_angle_error +   distanceRemainingRatio  +   strengthAdj + errorToheading;   

//    // serialPrintf2("error x:%.6f, weight x:%.6f",scanResult.distance.X,scanResult.distance_error);   
//     scanResult.angle_error =  std::fmax(0.01f, normal_angle_error +   strengthAdj + errorToheading);
   
}

int findAngleAndDistanceOffset(float avg_dist_inv, float est_dist_ratio,Point2D& nl, Point2D& nr, ArcNode* old_nodes,float sample_angle, int searchSize,ScanResult& bestScanResult,float maxAngle, float recipMaxAngle,const Point2D& heading)
{
    Point2D v = sub(nr,nl);
    
    Point2D midPoint = mul(add(nl,nr),0.5);
    float magMidp = mag(midPoint);

    Point2D n = normal_ccw(v);
    
     float nmp_angle = angleBetween(n,midPoint);     
    #ifdef TESTING
   float attenuationNew=1.0;//no lighting attenuaton in simulation
   #else
    float attenuationNew = 1-fabs(nmp_angle);
    #endif   
    
    float segmentSizeWeight = (1-magMidp/ (mag(v)+magMidp));

    float strengthAdj = segmentSizeWeight * (1- fmax(0,maxAngle-fabs(nmp_angle)) * recipMaxAngle);
    strengthAdj *=strengthAdj;      
    float initialAngle=sample_angle * est_dist_ratio;
    short index =  toIndex(initialAngle ,N_POINTS);
    
    short bestIndex =-1;
    ArcNode* prev_left = old_nodes + index;
    ArcNode* prev_right = prev_left;
    int prev_right_index = index;
    int prev_left_index = index;
    float old_sample_step_size = est_dist_ratio * RESOLUTION;
    ScanResult scanResult;

    for (int i = 1; i <searchSize; i++)
    {                  
        float nextRightAngle = initialAngle + old_sample_step_size*i;
        int rightIndex = toIndex(nextRightAngle ,N_POINTS);
        if(rightIndex == prev_right_index)
            rightIndex++;

        if (rightIndex >= N_POINTS)
        {
            rightIndex -= N_POINTS;
        }

        ArcNode* right = (old_nodes + rightIndex);
        #ifdef TESTING
        if(right->dist && prev_right->dist)
        {
        #endif
            calculate_angle_and_direction(attenuationNew,avg_dist_inv,strengthAdj,n,midPoint,prev_right,right,scanResult,heading);
       
              if( (scanResult.distance_error.X + scanResult.distance_error.Y    < bestScanResult.distance_error.X + bestScanResult.distance_error.Y  ) && fabs(scanResult.angle)<maxAngle)
                {
                    bestIndex = (rightIndex);//always return the right most index       
                    bestScanResult = scanResult;    
                    bestScanResult.angle= scanResult.angle+  (i-1) * old_sample_step_size;     
                    
                }
            
            prev_right = right;
            prev_right_index = rightIndex;
        #ifdef TESTING
        }
        #endif

         float nextLeftAngle = initialAngle - old_sample_step_size*i;
        int leftIndex = toIndex(nextLeftAngle ,N_POINTS);
        if(leftIndex == prev_left_index)
            leftIndex--;
        if (leftIndex < 0)
        {
            leftIndex += N_POINTS;
        }        
       
        ArcNode* left = (old_nodes + leftIndex);
        #ifdef TESTING
         if(left->dist && prev_left->dist )
        {
        #endif
            calculate_angle_and_direction(attenuationNew,avg_dist_inv, strengthAdj,n,midPoint,left,prev_left,scanResult,heading);
     
            if( (scanResult.distance_error.X + scanResult.distance_error.Y    < bestScanResult.distance_error.X + bestScanResult.distance_error.Y  ) && fabs(scanResult.angle)<maxAngle)
            {
                bestIndex = (rightIndex);//always return the right most index                
                bestScanResult = scanResult;    
                bestScanResult.angle= scanResult.angle - (i-1) * old_sample_step_size;    
              
                
            }
           
            prev_left = left;
            prev_left_index = leftIndex;
        #ifdef TESTING
        }      
        #endif
    }
    return (bestIndex);
}
struct WeightedAngle {
    float angle;
    float weight;
};


void removeOutliersPointMAD(float* values, int size, float thresh = 3.0f) 
{    
    float total=0;
    float temp[N_POINTS] = {0}; 
    for (size_t i = 0; i < size; i++)
    {
        temp[i] = values[i];
        total += values[i];
    }

    std::nth_element(temp, temp + size/2, temp+size,
          [](const float& a, const float& b) {
              return a < b;
          });

    float median = temp[size/ 2];

    // Compute absolute deviations
    if(median < -0.1 && total < 0)
    {
        for (int i =0;i<size;i++)
        {
            if(values[i] > 0)
            {
                values[i] = 0;
            }
        }
    }
    else if(median > 0.1 && total > 0)
    {
        for (int i =0;i<size;i++)
        {
            if(values[i] < 0)
            {
                values[i] = 0;
            }
        }
    }
    // for (int i =0;i<size;i++)
    // {

    //     float modified_z = 0.6745f * (values[i]- median) / mad;
    //     if (std::fabs(modified_z) >= thresh)
    //     {
    //         values[i] = 0;
    //     }
    // }
}
void removeOutliersMAD(WeightedAngle* data, int size, float thresh = 2.0f) 
{    
    float temp[N_POINTS]; 
    for (size_t i = 0; i < size; i++)
    {
        temp[i] = data[i].angle;
    }

    std::nth_element(temp, temp + size/2, temp+size,
          [](const float& a, const float& b) {
              return a < b;
          });

    float median = temp[size/ 2];

    // Compute absolute deviations
    for (int i =0;i<size;i++)
        temp[i] =std::fabs(angleDiffFast(data[i].angle ,median));

    std::nth_element(temp, temp + size/2, temp+size);    

    float mad = temp[size / 2];
    if(mad==0)
    {        
        return;
    }

    for (int i =0;i<size;i++)
    {
        float modified_z = 0.6745f * angleDiffFast(data[i].angle , median) / mad;
        if (std::fabs(modified_z) >= thresh)
        {
            data[i].angle = 0;
            data[i].weight = 0;
        }
    }
}

WeightedAngle weightedAngles[N_POINTS] = {0};
Point2D sum_difference(ArcNode* __restrict  new_nodes, ArcNode* __restrict  ref_nodes,const Point2D& guess,float angleGuess,float& angle_diff,float  maxAngle,float& totalDistResidual,const Point2D& heading, int searchRange)
{
    float recipMaxAngle  = 1.0f/maxAngle;
    int aCount=0;
    
    float prevAngleDiff = angle_diff;
    angle_diff = 0;
    totalDistResidual = 0;
    Point2D total = {0,0};    
    float total_distance_x_weight=0;
    float total_distance_y_weight=0;
    float total_angle_weight=0;
    float avg_dist_inv =0;
    for(int i =0;i<N_POINTS;i++)
    {
        avg_dist_inv += ref_nodes[i].dist;
    }
    avg_dist_inv = N_POINTS/ avg_dist_inv;
    float cos_f = cosf(angleGuess);
    float sin_f = sinf(angleGuess);
    for(unsigned short i=0;i<N_POINTS;i++)
    {
        ArcNode* new_node = new_nodes+i;       
        ArcNode* next_node = new_nodes + ((i + 1) % N_POINTS);
        #ifdef TESTING
        if(!next_node->dist || !new_node->dist )
            continue;
        #endif  

        float next_angle = next_node->angle;    
        
        float new_angle = new_node->angle;    
        int ref_index = toIndex(new_angle+angleGuess,N_POINTS);
        float ref_dist = ref_nodes[ref_index].dist;
        float est_dist_ratio =  ref_dist == 0 ? 1 : new_node->dist/ref_dist;
       
        Point2D expected_old_frame_point = rotate(sub(new_node->point ,guess),cos_f,sin_f);    
        Point2D expected_next_old_frame_point = rotate(sub(next_node->point ,guess),cos_f,sin_f);  

        ScanResult bestScanResult;
        bestScanResult.distance_error = {900000000,900000000};
        int index = findAngleAndDistanceOffset(avg_dist_inv, est_dist_ratio,expected_old_frame_point,expected_next_old_frame_point,ref_nodes, new_angle+angleGuess ,searchRange,bestScanResult,maxAngle,recipMaxAngle,heading);
        
        if(index>=0)
        {   
            float res = mag(bestScanResult.distance);
            if(res>1 && res < 200)
            {
                totalDistResidual += res;
            }
            
            float angle_weight = bestScanResult.angle_error == 0 ? 0 : 1/bestScanResult.angle_error  ;
            float distance_weight_x =bestScanResult.distance_error.X == 0 ? 0 : 1/bestScanResult.distance_error.X;
            float distance_weight_y =bestScanResult.distance_error.Y == 0 ? 0 : 1/bestScanResult.distance_error.Y;
            
            weightedAngles[aCount++] = {bestScanResult.angle,angle_weight};   
        //    distanceWeights[aCount] =distance_weight;   
            // xValues[aCount] =bestScanResult.distance.X;
            // yValues[aCount] =bestScanResult.distance.Y;
            total_distance_x_weight +=distance_weight_x;      
            total_distance_y_weight +=distance_weight_y;      
            total.X  += bestScanResult.distance.X * distance_weight_x;          
            total.Y  += bestScanResult.distance.Y * distance_weight_y;          
                             
        }
    }
  
   
    removeOutliersMAD(weightedAngles,aCount);
    for(int i =0;i<aCount;i++)
    {
        angle_diff += weightedAngles[i].angle * weightedAngles[i].weight;
        total_angle_weight += weightedAngles[i].weight;
    }
     if(total_angle_weight > 0)
     { 
        angle_diff /= total_angle_weight;  
     }
    //  serialPrintf2("start");
    // for(int i =0;i<aCount;i++)
    // {
    //     serialPrintf2("X:%.6f, Y:%.6f, W:%.6f",xValues[i],yValues[i],distanceWeights[i]);   
    // }
   // removeOutliersPointMAD(xValues, aCount);    
   // removeOutliersPointMAD(yValues, aCount);
    //  serialPrintf2("filtered");
    // for(int i =0;i<aCount;i++)
    // {
    //     serialPrintf2("X:%.6f, Y:%.6f",xValues[i],yValues[i]);   
    // }
    // for(int i =0;i<aCount;i++)
    // {       
    //     total.X  += xValues[i] * distanceWeights[i];          
    //     total.Y  += yValues[i] * distanceWeights[i];        
    // }
     if(total_distance_x_weight > 0)
     { 
        total.X /= total_distance_x_weight;  
     }
    if(total_distance_y_weight > 0)
     { 
        total.Y /= total_distance_y_weight;  
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
                new_angle =(new_node->angle-M_2PI) ;
            }
            else
            {
                new_angle =new_node->angle ;
            }
                
            ArcNode* next_node = nodes + ((i + 1) % N_POINTS);
            if(!next_node->dist)
                continue;

            float next_angle = next_node->angle;
            
            Point2D new_point = toPoint2D(new_angle,new_node->dist);

            Point2D expected_old_frame_point =sub(new_point ,trans);                             
            dst[i] = rotate(expected_old_frame_point,-rot);
           
        }
    }
}

bool find_pos(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& pos, float& angle, float& total_residual,const Point2D& heading)
{        
    int timeout=10000;  
    
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
        Point2D error= sum_difference(nodes_new,nodes_old, pos,angle,error_angle,1.57,total_residual,heading,2);          
       
        angle = PIDUpdate(&pid_translate_rotate,-error_angle,angle); 
        pos.Y = PIDUpdate(&pid_y,-error.Y,pos.Y); 
        pos.X = PIDUpdate(&pid_x,-error.X,pos.X); 
     
        serialPrintf2("guess_x: %.6f,guess_y: %.6f,  error_x: %.6f,error_y: %.6f, error_angle: %.6f, angle: %.6f",pos.X,pos.Y,error.X,error.Y,error_angle,angle);
        if(fabs(error.X) < pid_x.deadband && fabs(error.Y) < pid_y.deadband && fabs(error_angle) < 0.0001 )
        {
            break;
        } 
    }  

    return true;    
}

bool find_yaw(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& pos,float& angle, float& total_residual)
{
    Point2D heading;
    int timeout=5000;  
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
      
        Point2D error= sum_difference(nodes_new,nodes_old, pos,angle,error_angle,1.57,total_residual,heading,5); 
         
       
        angle = PIDUpdate(&pid_rotate,-error_angle,angle); 
        pos.Y = PIDUpdate(&pid_rotate_translate,-error.Y,pos.Y); 
        pos.X = PIDUpdate(&pid_rotate_translate,-error.X,pos.X); 
     
       // serialPrintf2("guess_x: %.6f,guess_y: %.6f,  error_x: %.6f,error_y: %.6f, error_angle: %.6f, angle: %.6f",pos.X,pos.Y,error.X,error.Y,error_angle,angle);
       
         if(fabs(error.X) < pid_rotate_translate.deadband && fabs(error.Y) < pid_rotate_translate.deadband && fabs(error_angle) < 0.0002 )
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
        //serialPrintf2("quality:%i angle:%i dist:%i",nodes[i].quality,nodes[i].angle_z_q6,nodes[i].dist_mm_q2);
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
