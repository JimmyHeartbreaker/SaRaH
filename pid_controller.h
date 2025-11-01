#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "NodeServices.h"
#include "arduino_ext.h"

#define MOVE_SCAN_ANGLE (10* (M_PI_F/180))
#define ROT_SCAN_ANGLE (180*M_PI_F/180)
#define M_2PI (M_PI_F*2)

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
bool find_y(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& guess,float& angle,float left, float right,float& std);
bool find_x(ArcNode* nodes_new, ArcNode* nodes_old, Point2D& guess,float& angle,float left, float right,float& std);
bool find_yaw(ArcNode* nodes_new, ArcNode* nodes_old,float left, float right, float assumed_yaw,float maxRange, float& outGuess);

Point2D sum_difference(ArcNode* new_nodes, ArcNode* old_nodes,float left,float right,Point2D guess,float angleGuess,float& std,float& angle_diff);
#endif