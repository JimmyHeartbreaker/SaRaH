#pragma once

#include "sl_lidar.h" 
enum DedState
{
    STARTUP,
    SCAN,
    WAITING,
    MOVING
};


enum MoveState
{
    F, //forward
    B, //back
    P, //port, left
    S, //starboard, right
    L,//rotate left, anti clockwise
    R //rotate right, clockwise
};


void DataIn(sl_lidar_response_measurement_node_hq_t* nodes, size_t nodeCount);