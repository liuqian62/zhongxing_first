#include <iomanip>
#include <fstream>
#include <string>
#include <iostream>
#include <math.h>
#include <vector>
// #include "Generator.h"

using namespace std;

struct Position
{
    bool is_base_station;
    float x;
    float y;
    float z;
    int m;
    int n;
    void init(bool _is_base_station, float _x, float _y, float _z)
    {
        is_base_station = _is_base_station;
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
    void init(float _time, Base_Station _strat, Base_Station _end)
    {
        sta_station = _strat;
        end_station = _end;
        time = _time;
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

    bool is_distance_suit(Position pos1, Position pos2, float &time_now);

    float get_distance(Position pos1, Position pos2);

    Position get_position_from_mn(int m, int n, float time_now);

    /**
     * @brief Destroy the Drone object
     *
     */
    ~Drone();
};

Drone::Drone(/* args */)
{
}

void Drone::find_best_drone(
    Step &stp,
    float &time_now,
    Position &position_now,
    Position destination,
    bool &finish)
{
    Position tmp_position = position_now;
    // tmp_position.init(position_now.is_base_station,position_now.x,position_now.y,position_now.z);
    // tmp_position.m=position_now.m;
    // position_now.n=position_now.n;
    if (position_now.is_base_station)
    {
        // position_now.m=int(position_now.x/_d_intraorbit);
        // position_now.n=int(position_now.y/_d_intraorbit);\

        int m = int((position_now.x - _v * time_now) / _d_intraorbit);
        int n = int(position_now.y / _d_intraorbit);
        vector<int> m_range, n_range;
        for (int i = 0; i < 3; i++)
        {
            m_range.push_back(m - 1 + i);
            n_range.push_back(n - 1 + i);
        }

        float best_distance = get_distance(position_now, destination) + 200.0f;
        int best_m = INT64_MAX;
        int best_n = INT64_MAX;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                Position po1 = get_position_from_mn(m_range[i], n_range[j], time_now);
                float distan_1 = get_distance(po1, position_now);
                Position po2 = get_position_from_mn(m_range[i], n_range[j], time_now + _tf + distan_1 * _xishu);
                float distan_2 = get_distance(po2, position_now);
                float dis_to_destination = get_distance(po1, destination);
                if (max(distan_1, distan_2) < _drone_base && dis_to_destination < best_distance)
                {
                    best_distance = dis_to_destination;
                    best_m = m_range[i];
                    best_n = n_range[j];
                }
            }
        }
        position_now.m = best_m;
        position_now.n = best_n;
        Position tmp = get_position_from_mn(best_m, best_n, time_now);
        float tmp_dis = get_distance(tmp, position_now);
        tmp_dis = floor(tmp_dis);
        time_now += _xishu * tmp_dis;
        tmp_position.m = position_now.m;
        tmp_position.n = position_now.n;

        // cout<<position_now.m<<","<<position_now.n<<endl;

        position_now.is_base_station = false;
    }
    else
    {
        float x_distance = fabs(position_now.x - destination.x);
        float y_distance = fabs(position_now.y - destination.y);
        if (x_distance > _drone_base && y_distance > _drone_base)
        {
            if (x_distance > _d_intraorbit)
            {
                if (position_now.x < destination.x)
                {
                    position_now.m += 1;
                }
                else
                {
                    position_now.m -= 1;
                }
            }
            if (y_distance > _d_interorbit)
            {
                if (position_now.y < destination.y)
                {
                    position_now.n += 1;
                }
                else
                {
                    position_now.n -= 1;
                }
            }
            // cout<<position_now.m<<","<<position_now.n<<endl;
        }
        else if (!is_distance_suit(position_now, destination, time_now))
        {
            if (x_distance >= _d_intraorbit)
            {
                if (position_now.x < destination.x)
                {
                    position_now.m += 1;
                }
                else
                {
                    position_now.m -= 1;
                }
            }
            else
            {
                float tmp_y_distance;
                tmp_y_distance = y_distance - floor(y_distance / _d_interorbit) * _d_interorbit;
                if (tmp_y_distance > _d_interorbit / 2)
                {
                    tmp_y_distance = _d_interorbit - tmp_y_distance;
                }
                float tmp_dis_2 = pow(x_distance, 2) + pow(tmp_y_distance, 2);
                float tmp_dis_inv_2 = pow((_d_intraorbit - x_distance), 2) + pow(tmp_y_distance, 2);
                float tmp_dis = sqrt(tmp_dis_2);
                float tmp_dis_inv = sqrt(tmp_dis_inv_2);
                if (tmp_dis > _drone_base || _d_interorbit + tmp_dis >= 120.0f + tmp_dis_inv)
                {
                    if (x_distance > _d_intraorbit / 2)
                    {
                        if (position_now.x < destination.x)
                        {
                            position_now.m += 1;
                        }
                        else
                        {
                            position_now.m -= 1;
                        }
                    }
                }
            }
            if (y_distance >= _d_interorbit)
            {
                if (position_now.y < destination.y)
                {
                    position_now.n += 1;
                }
                else
                {
                    position_now.n -= 1;
                }
            }
            else
            {
                float tmp_x_distance;
                tmp_x_distance = x_distance - floor(x_distance / _d_intraorbit) * _d_intraorbit;
                // cout << tmp_x_distance << endl;
                if (tmp_x_distance > _d_intraorbit / 2)
                {
                    tmp_x_distance = _d_intraorbit - tmp_x_distance;
                }
                float tmp_dis_2 = pow(y_distance, 2) + pow(tmp_x_distance, 2);
                float tmp_dis_inv_2 = pow((_d_interorbit - y_distance), 2) + pow(tmp_x_distance, 2);
                float tmp_dis = sqrt(tmp_dis_2);
                float tmp_dis_inv = sqrt(tmp_dis_inv_2);
                if (tmp_dis > _drone_base || _d_intraorbit + tmp_dis >= 120.0f + tmp_dis_inv)
                {
                    if (y_distance > _d_interorbit / 2)
                    {
                        if (position_now.y < destination.y)
                        {
                            position_now.n += 1;
                        }
                        else
                        {
                            position_now.n -= 1;
                        }
                    }
                }
            }

            // cout<<position_now.m<<","<<position_now.n<<endl;
        }
        else
        {
            finish = true;
        }
    }
    if (!finish)
    {
        stp.m = position_now.m;
        stp.n = position_now.n;
        float dis_x_2 = pow(_d_intraorbit * (tmp_position.m - position_now.m), 2);
        float dis_y_2 = pow(_d_interorbit * (tmp_position.n - position_now.n), 2);
        float dis_2 = dis_x_2 + dis_y_2;
        // float dis_2 = pow(_d_intraorbit * (tmp_position.m - position_now.m), 2) + pow(_d_interorbit * (tmp_position.n - position_now.n), 2);
        float dis = sqrt(dis_2);
        dis = floor(dis);
        // if (dis_x_2 > 0.0f && dis_y_2 > 0.0f)
        // {
        //     dis -= 0.4158;
        // }
        time_now += _tf + _xishu * dis;
        // if(dis_2>1000000){
        //     cout<<dis_2<<endl;

        // }

        position_now.x = _v * time_now + position_now.m * _d_intraorbit;
        position_now.y = position_now.n * _d_interorbit;
        position_now.z = _H;
        stp.t = time_now;
        // cout<<_xishu<<endl;
    }
}

float Drone::get_distance(Position pos1, Position pos2)
{
    float dis, dis_2;
    dis_2 = pow((pos2.x - pos1.x), 2) + pow((pos2.y - pos1.y), 2) + pow((pos2.z - pos1.z), 2);
    dis = sqrt(dis_2);
    return dis;
}

Position Drone::get_position_from_mn(int m, int n, float time_now)
{
    Position posi;
    posi.x = _v * time_now + m * _d_intraorbit;
    posi.y = n * _d_interorbit;
    posi.z = _H;
    return posi;
}

bool Drone::is_distance_suit(Position pos1, Position pos2, float &time_now)
{
    float distance1, distance1_2;
    float distance2, distance2_2;
    distance1_2 = pow((pos2.x - pos1.x), 2) + pow((pos2.y - pos1.y), 2) + pow((pos2.z - pos1.z), 2);
    distance1 = sqrt(distance1_2);
    distance1 = floor(distance1);
    float delta_t = _tf + _xishu * distance1;
    distance2_2 = pow((pos2.x - pos1.x - _v * delta_t), 2) + pow((pos2.y - pos1.y), 2) + pow((pos2.z - pos1.z), 2);
    distance2 = sqrt(distance2_2);
    distance2 = floor(distance2);
    if (distance1 < _drone_base && distance2 < _drone_base)
    {
        time_now += _tf + _xishu * min(distance1, distance2);
        return true;
    }
    else
    {
        return false;
    }
}

Drone::~Drone()
{
}
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

Generator::Generator(string Output)
{
    this->_Output = Output;
}

void Generator::do_generate()
{
    get_base_station();
    get_demand();
    generate_result();
    output();
    output_to_terminal();
}

void Generator::get_base_station()
{
    Base_Station sta0, sta1, sta2;
    sta0.init(0, 45.73, 45.26, 0.0);
    sta1.init(1, 1200, 700, 0.0);
    sta2.init(2, -940, 1100, 0.0);

    base_stations.push_back(sta0);
    base_stations.push_back(sta1);
    base_stations.push_back(sta2);
}

void Generator::get_demand()
{
    float t[] = {0, 4.7, 16.4};
    for (int i = 0; i < 3; i++)
    {
        Demand dem1, dem2;
        dem1.init(t[i], base_stations[0], base_stations[1]);
        dem2.init(t[i], base_stations[0], base_stations[2]);
        this->demans.push_back(dem1);
        this->demans.push_back(dem2);
    }
}

void Generator::generate_result()
{
    Drone drone;
    for (Demand dem : demans)
    {
        Solution sol;
        sol.time = dem.time;
        sol.start_id = dem.sta_station.id;
        sol.end_id = dem.end_station.id;

        bool finish = false;
        float time_now = dem.time;
        Position position_now, destination;
        position_now.init(true, dem.sta_station.x, dem.sta_station.y, dem.sta_station.z);
        destination.init(true, dem.end_station.x, dem.end_station.y, dem.end_station.z);
        while (!finish)
        {
            Step stp;
            drone.find_best_drone(stp, time_now, position_now, destination, finish);

            // stp.t = 0.0;
            // stp.m = 1;
            // stp.n = 2;
            if (!finish)
            {
                sol.steps.push_back(stp);
            }
        }

        sol.delay = time_now - dem.time;
        this->solutions.push_back(sol);
    }
}

void Generator::output()
{

    ofs.open(_Output, ios::trunc);
    float total = 0.0f;
    for (Solution sol : solutions)
    {
        total += sol.delay;
        ofs << setiosflags(ios::fixed) << setprecision(4) << sol.time << "," << sol.start_id << "," << sol.end_id << "," << sol.delay << endl;
        // cout<<sol.time<<endl;
        // cout<<setiosflags(ios::fixed)<<setprecision(4)<<sol.time<<endl;
        for (int i = 0; i < sol.steps.size(); i++)
        {
            if (i != 0)
            {
                ofs << ",";
            }
            ofs << "(" << sol.steps[i].t << "," << sol.steps[i].m << "," << sol.steps[i].n << ")";
            // cout<<sol.steps[i].t<<endl;
        }
        if (sol.steps.size() != 0)
        {
            ofs << endl;
        }
    }
    ofs << "total:" << total;

    ofs.close();
}

void Generator::output_to_terminal()
{
    for (Solution sol : solutions)
    {
        cout << setiosflags(ios::fixed) << setprecision(4) << sol.time << "," << sol.start_id << "," << sol.end_id << "," << sol.delay << endl;
        // cout<<sol.time<<endl;
        // cout<<setiosflags(ios::fixed)<<setprecision(4)<<sol.time<<endl;
        for (int i = 0; i < sol.steps.size(); i++)
        {
            if (i != 0)
            {
                cout << ",";
            }
            cout << "(" << setiosflags(ios::fixed) << setprecision(4) << sol.steps[i].t << "," << sol.steps[i].m << "," << sol.steps[i].n << ")";
            // cout<<sol.steps[i].t<<endl;
        }
        if (sol.steps.size() != 0)
        {
            cout << endl;
        }
    }
}

Generator::~Generator()
{
}

string Output = "result.txt";

int main()
{
    Generator generator(Output);
    generator.do_generate();
    return 0;
}