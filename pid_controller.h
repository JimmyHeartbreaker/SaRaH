#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "NodeServices.h"
#include "arduino_ext.h"

#define MOVE_SCAN_ANGLE (20* (M_PI_F/180))
#define ROT_SCAN_ANGLE (180*M_PI_F/180)
#define M_2PI (M_PI_F*2)
#define RESOLUTION (M_2PI/N_POINTS) //rad
typedef struct LidarScanNormalMeasureRaw
{
    unsigned char    quality;
    unsigned short   angle_z_q6;
    unsigned short   dist_mm_q2;
} __attribute__((packed)) LidarScanNormalMeasureRaw;
struct pid_state
{
	float kp;
	float ki;
	float kd;
	float max;
	float min;
	float dt;
	float Tt;
	float deadband;
	float integral; 
	float prev_error;
};

inline float angleDiffFast(float a, float b) {
	float d = a - b;
	d -= M_2PI * std::floor((d + M_PI_F) * INV_TWO_PI);
	return d;
}
bool find_y(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& guess,float& angle,float left, float right,float& std,Point2D heading);
bool find_x(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& guess,float& angle,float left, float right,float& std,Point2D heading);
bool find_yaw(ArcNode* nodes_new, ArcNode* nodes_old,float left, float right, float assumed_yaw,float maxRange, float& outGuess);
float read_scan(
    LidarScanNormalMeasureRaw* nodes,
    ArcNode* dst,
    unsigned short nodeCount,
    float new_weight = 0.5f,
    bool calculatePoints = false);

Point2D sum_difference_rotation(ArcNode* new_nodes, ArcNode* old_nodes,float left,float right,float angleGuess,float& angle_diff, int searchRange);
float sum_difference(ArcNode* new_nodes, ArcNode* old_nodes,float left,float right,float angleGuess,float maxRange);
Point2D sum_difference(ArcNode* new_nodes, ArcNode* old_nodes,float left,float right,Point2D guess,float angleGuess,float& std,float& angle_diff,Point2D heading, int searchRange=3);

#endif