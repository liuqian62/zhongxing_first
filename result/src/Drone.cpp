#include "Drone.h"
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
        tmp_dis=floor(tmp_dis);
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
                if(tmp_y_distance>_d_interorbit/2){
                    tmp_y_distance=_d_interorbit-tmp_y_distance;
                }
                float tmp_dis_2 = pow(x_distance, 2) + pow(tmp_y_distance, 2);
                float tmp_dis_inv_2 = pow((_d_intraorbit - x_distance), 2) + pow(tmp_y_distance, 2);
                float tmp_dis = sqrt(tmp_dis_2);
                float tmp_dis_inv = sqrt(tmp_dis_inv_2);
                if (tmp_dis > _drone_base || _d_interorbit + tmp_dis >= 120.0f + tmp_dis_inv)
                {
                    if(x_distance>_d_intraorbit/2){
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
                if(tmp_x_distance>_d_intraorbit/2){
                    tmp_x_distance=_d_intraorbit-tmp_x_distance;
                }
                float tmp_dis_2 = pow(y_distance, 2) + pow(tmp_x_distance, 2);
                float tmp_dis_inv_2 = pow((_d_interorbit - y_distance), 2) + pow(tmp_x_distance, 2);
                float tmp_dis = sqrt(tmp_dis_2);
                float tmp_dis_inv = sqrt(tmp_dis_inv_2);
                if (tmp_dis > _drone_base || _d_intraorbit + tmp_dis >= 120.0f + tmp_dis_inv)
                {
                    if(y_distance>_d_interorbit/2){
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
        dis=floor(dis);
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
    distance1 =floor(distance1);
    float delta_t = _tf + _xishu * distance1;
    distance2_2 = pow((pos2.x - pos1.x - _v * delta_t), 2) + pow((pos2.y - pos1.y), 2) + pow((pos2.z - pos1.z), 2);
    distance2 = sqrt(distance2_2);
    distance2=floor(distance2);
    if (distance1 < _drone_base && distance2 < _drone_base)
    {
        time_now += _tf + _xishu * min(distance1, distance2);
        // cout<<pos1.x<<","<<pos1.y<<endl;
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