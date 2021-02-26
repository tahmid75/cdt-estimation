#include <iostream>
#include <stdlib.h>
#include <string>
#include <map>
using namespace std;
#include "veins/base/utils/Coord.h"

class Registry
{
    public:

    std::map<int, std::vector<std::tuple<veins::Coord, double, double>>> vehicleRegistry;
                                   // (vehicleCoord, vehicleSpeed, simulationTime)
    std::map<int, veins::Coord> rsuRegistry;


    //       rsu         vehicleid      messageTime, DD, DT
    // This if I want to keep historic data
    //std::map<int, std::map<int, std::vector<std::tuple<int, double, int>>>> msgRegistry;

    std::map<int, std::map<int, std::tuple<int, double, int, int>>> msgRegistry;

    //       RSU          msgTime        vehicleId(i,...i+n)
    std::map<int, std::map<int, std::vector<int>>> inRangeVehicle;
    //std::map<int, std::map<int, int>> dtPrediction;

        //   RSU          time  volume
    std::map<int, std::map<int, int>> predictedVolume;

};
