#ifndef __DRONE_H__
#define __DRONE_H__
#include <string>
#include "utils.hpp"
#include <math.h>
#include <iostream>
// #include <vector>
// #include <algorithm>
using namespace std;

class Drone
{
private:
    float _drone_drone = 125.0;
    float _drone_base = 70.0;
    float _v = 5.0;
    float _d_intraorbit = 90.0;
    float _d_interorbit = 80.0;
    float _H = 10.0;
    float _tf = 0.1;
    float _xishu = 0.0001;

public:
    /**
     * @brief Construct a new Drone object
     *
     */
    Drone();

    /**
     * @brief at every position, find the best next drone or base station
     * 
     * @param stp one step, include t m n
     * @param time_now the time when this drone get the signal
     * @param position_now the position of signal
     * @param destination the destination base station position
     * @param finish finished the process or not
     */
    void find_best_drone(
        Step &stp,
        float &time_now,
        Position &position_now,
        Position destination,
        bool &finish);


    bool is_distance_suit(Position pos1,Position pos2,float &time_now);

    float get_distance(Position pos1,Position pos2);

    Position get_position_from_mn(int m,int n,float time_now);

    /**
     * @brief Destroy the Drone object
     *
     */
    ~Drone();
};

#endif