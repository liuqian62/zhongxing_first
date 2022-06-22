#ifndef __UTILS_H__
#define __UTILS_H__
#include <string>
#include <vector>
// #include <algorithm>
using namespace std;

struct Position
{
    bool is_base_station;
    float x;
    float y;
    float z;
    int m=0;
    int n=0;
    void init(bool _is_base_station, float _x, float _y,float _z){
        is_base_station=_is_base_station;
        x = _x;
        y = _y;
        z = _z;
    }
};

struct Base_Station
{
    int id;
    float x;
    float y;
    float z;
    void init(int _id, float _x, float _y, float _z)
    {
        id = _id;
        x = _x;
        y = _y;
        z = _z;
    }
};

struct Demand
{
    Base_Station sta_station;
    Base_Station end_station;
    float time;
    void init(float _time,Base_Station _strat,Base_Station _end){
        sta_station =_strat;
        end_station =_end;
        time =_time;
    }
};

struct Step
{
    float t;
    int m;
    int n;
};

struct Solution
{
    float time;
    int start_id;
    int end_id;
    float delay;
    vector<Step> steps;
};

#endif