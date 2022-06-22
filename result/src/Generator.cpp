#include "Generator.h"

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
            if(!finish){
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
    float total=0.0f;
    for (Solution sol : solutions)
    {
        total+=sol.delay;
        ofs << setiosflags(ios::fixed)<<setprecision(4)<<sol.time << "," << sol.start_id << "," << sol.end_id << "," << sol.delay << endl;
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
    ofs<<"total:"<<total;

    ofs.close();
}

void Generator::output_to_terminal(){
    for (Solution sol : solutions)
    {
        cout << setiosflags(ios::fixed)<<setprecision(4)<<sol.time << "," << sol.start_id << "," << sol.end_id << "," << sol.delay<<endl;
        // cout<<sol.time<<endl;
        // cout<<setiosflags(ios::fixed)<<setprecision(4)<<sol.time<<endl;
        for (int i = 0; i < sol.steps.size(); i++)
        {
            if (i != 0)
            {
                cout << ",";
            }
            cout << "(" << setiosflags(ios::fixed)<<setprecision(4)<< sol.steps[i].t << "," << sol.steps[i].m << "," << sol.steps[i].n << ")";
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