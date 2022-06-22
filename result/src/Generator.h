#ifndef __GENERATOR_H__
#define __GENERATOR_H__
#include <string>
#include <fstream>
#include "utils.hpp"
#include "Drone.h"
#include <iostream>
#include<iomanip>

// #include <vector>
// #include <algorithm>
using namespace std;

class Generator
{
private:
    string _Output = "result.txt";
    ofstream ofs;


    vector<Base_Station> base_stations;
    vector<Demand> demans;
    vector<Solution> solutions;

public:
    /**
     * @brief Construct a new Generator object
     *
     * @param Output
     */
    Generator(string Output);

    /**
     * @brief 
     * 
     */
    void do_generate();

    /**
     * @brief Get the base station object
     *
     */
    void get_base_station();

    /**
     * @brief Get the demand object
     *
     */
    void get_demand();

    /**
     * @brief
     *
     */
    void generate_result();

    /**
     * @brief
     *
     */
    void output();

    void output_to_terminal();

    /**
     * @brief Destroy the Generator object
     *
     */
    ~Generator();
};

#endif