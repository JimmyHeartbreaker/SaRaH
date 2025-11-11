#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "NodeServices.h"
#include "arduino_ext.h"

#define MOVE_SCAN_ANGLE (90* (M_PI_F/180))
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
bool find_pos(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& pos,float& angle, float& total_residual);
bool find_yaw(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& pos,float& angle, float& total_residual);
float read_scan(
    LidarScanNormalMeasureRaw* nodes,
    ArcNode* dst,
    unsigned short nodeCount,
    float new_weight = 1.0f,
    bool calculatePoints = false);

Point2D sum_difference(ArcNode* new_nodes, ArcNode* old_nodes,Point2D guess,float angleGuess,float& angle_diff,float  maxAngle,float& total_residual, int searchRange=3);

Point2D* map_nodes(ArcNode* nodes,Point2D* dst, Point2D trans, float rot, bool onlyVisible=true);
#endif