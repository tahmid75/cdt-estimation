//
// Copyright (C) 2006-2011 Christoph Sommer <christoph.sommer@uibk.ac.at>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include <veins/modules/application/traci/Registry.h>
#include "veins/modules/application/traci/TraCIDemo11p.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include "veins/modules/application/traci/mlr.h"


#include <iostream>
#include <string>
#include <cmath>
using namespace std;
using namespace veins;


Define_Module(veins::TraCIDemo11p);


Registry registry;
int edgeRadius = 800;
int threshold = 8000;
auto rsuRange = 800.0;


double linearDistance(Coord& a,  Coord& b) {
    Coord dist(a - b);
    return dist.length();
}


using Point = std::pair<double, double>;
constexpr auto eps = 1e-14;

std::ostream &operator<<(std::ostream &os, const Point &p) {
    auto x = p.first;
    if (x == 0.0) {
        x = 0.0;
    }
    auto y = p.second;
    if (y == 0.0) {
        y = 0.0;
    }
    return os << '(' << x << ", " << y << ')';
}

template <typename T> std::ostream &operator<<(std::ostream &os, const std::vector<T> &v) {
    auto itr = v.cbegin();
    auto end = v.cend();

    os << '[';
    if (itr != end) {
        os << *itr;
        itr = std::next(itr);
    }
    while (itr != end) {
        os << ", " << *itr;
        itr = std::next(itr);
    }
    return os << ']';
}

double sq(double x) {
    return x * x;
}

std::vector<Point> intersects(const Point &p1, const Point &p2, const Point &cp, double r, bool segment) {
    std::vector<Point> res;
    auto x0 = cp.first;
    auto y0 = cp.second;
    auto x1 = p1.first;
    auto y1 = p1.second;
    auto x2 = p2.first;
    auto y2 = p2.second;
    auto A = y2 - y1;
    auto B = x1 - x2;
    auto C = x2 * y1 - x1 * y2;
    auto a = sq(A) + sq(B);
    double b, c;
    bool bnz = true;
    if (abs(B) >= eps) {
        b = 2 * (A * C + A * B * y0 - sq(B) * x0);
        c = sq(C) + 2 * B * C * y0 - sq(B) * (sq(r) - sq(x0) - sq(y0));
    } else {
        b = 2 * (B * C + A * B * x0 - sq(A) * y0);
        c = sq(C) + 2 * A * C * x0 - sq(A) * (sq(r) - sq(x0) - sq(y0));
        bnz = false;
    }
    auto d = sq(b) - 4 * a * c; // discriminant
    if (d < 0) {
        return res;
    }

    // checks whether a point is within a segment
    auto within = [x1, y1, x2, y2](double x, double y) {
        auto d1 = sqrt(sq(x2 - x1) + sq(y2 - y1));  // distance between end-points
        auto d2 = sqrt(sq(x - x1) + sq(y - y1));    // distance from point to one end
        auto d3 = sqrt(sq(x2 - x) + sq(y2 - y));    // distance from point to other end
        auto delta = d1 - d2 - d3;
        return abs(delta) < eps;                    // true if delta is less than a small tolerance
    };

    auto fx = [A, B, C](double x) {
        return -(A * x + C) / B;
    };

    auto fy = [A, B, C](double y) {
        return -(B * y + C) / A;
    };

    auto rxy = [segment, &res, within](double x, double y) {
        if(!isnan(x) && !isnan(y)){ // removing NaN
            if (!segment || within(x, y)) {
                res.push_back(std::make_pair(x, y));
            }
        }

    };

    double x, y;
    if (d == 0.0) {
        // line is tangent to circle, so just one intersect at most
        if (bnz) {
            x = -b / (2 * a);
            y = fx(x);
            rxy(x, y);
        } else {
            y = -b / (2 * a);
            x = fy(y);
            rxy(x, y);
        }
    } else {
        // two intersects at most
        d = sqrt(d);
        if (bnz) {
            x = (-b + d) / (2 * a);
            y = fx(x);
            rxy(x, y);
            x = (-b - d) / (2 * a);
            y = fx(x);
            rxy(x, y);
        } else {
            y = (-b + d) / (2 * a);
            x = fy(y);
            rxy(x, y);
            y = (-b - d) / (2 * a);
            x = fy(y);
            rxy(x, y);
        }
    }

    return res;
}


double calculateVelocity(Coord& a,  Coord& b, double latestTime, double oldestTime){
    Coord dist(a - b);
    return dist.length()/(latestTime-oldestTime);
}


// Fires for every car loading.
void TraCIDemo11p::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);

}


tuple<int, double> findNearestEdge(Coord vehicleCoord){

    double lowest=35000; // set to max map size.
    int nearest;
    double runningDistance;

    for (auto const& rsu : registry.rsuRegistry)
    {
        Coord dist(vehicleCoord - rsu.second);
        runningDistance = abs( sqrt( pow(vehicleCoord.x - rsu.second.x,2) + pow(vehicleCoord.y - rsu.second.y, 2) ) - edgeRadius );

        if(runningDistance < lowest){
            lowest= runningDistance;
            nearest = rsu.first;
        }
    }

    return make_tuple(nearest, lowest);
}


// About the vehicle position. Everytime the vehicle changes position this is fired.
void TraCIDemo11p::handlePositionUpdate(cObject* obj)
{

    DemoBaseApplLayer::handlePositionUpdate(obj);

    if(((int) simTime().dbl()) % (rand() % 6 + 13) == 0 ){

        // Getting vehicle data
        Coord vehicleCoord = mobility->getPositionAt(simTime().dbl());
        double vehicleSpeed = mobility->getSpeed();
        int vehicleID = mobility->getId();
        double simulationTime = simTime().dbl();

        //std::cout << "Heading Angle: " << mobility->getHeading().getRad() << endl ;

        // Creating tuple of information
        std::tuple<veins::Coord, double, double> vehicleData (vehicleCoord, vehicleSpeed, simulationTime);

        // Adding information into vehicle registry.
        registry.vehicleRegistry[vehicleID].insert(registry.vehicleRegistry[vehicleID].begin(), vehicleData) ;

        // Removing last data
        int size = registry.vehicleRegistry[vehicleID].size();

        if( size > 10){
            registry.vehicleRegistry[vehicleID].pop_back();
        }

        if(size> 2){
            // Checking if path crosses any RSU range in any way
            for (auto const& rsu : registry.rsuRegistry)
            {
                Coord currentDist(rsu.second - vehicleCoord);

                if(currentDist.length() < 800){ // within range

                    // Sending a WSM Message to currentRSU
                    TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
                    populateWSM(wsm);
                    wsm->setSenderType(1);
                    wsm->setSenderAddress(vehicleID);
                    wsm->setTargetAddress(rsu.first);
                    wsm->setSenderSpeedRLDCO(vehicleSpeed);
                    wsm->setSenderPositionRLDCO(vehicleCoord);
                    wsm->setHopCountRLDCO(0);
                    sendDown(wsm);

                    //std::cout << "----------------------------------" << endl;
                }


                else{

                    Coord oldest = get<0>(registry.vehicleRegistry[vehicleID][size-1]) ;
                    //          first element is coord          id for map    last tuple

                    auto cp = std::make_pair(rsu.second.x, rsu.second.y);
                    auto p1 = std::make_pair(vehicleCoord.x, vehicleCoord.y);
                    auto p2 = std::make_pair(oldest.x, oldest.y);

                    std::vector<Point> output = intersects(p1, p2, cp, rsuRange, false);

                    if(!output.empty()) {
                        //std::cout << "Line containing the points " << p1.first << "," << p1.second << " and " ;
                        //std::cout << p2.first << "," << p2.second << " intersects RSU "  << rsu.first << "("<< rsu.second.x << "," << rsu.second.y << ") at: " << endl;


                        Coord initDist(rsu.second - oldest);

                        if(initDist.length() > currentDist.length()){ // vehicle moving towards the RSU

                            if(currentDist.length() < 2400){
                                /*
                                std::cout << "Vehicle ID: " << vehicleID << endl;
                                std::cout << "Trajectory intersects: "<< rsu.first << endl;
                                std::cout << "Current Distance to this RSU is: " << currentDist.length() << endl;
                                std::cout << "Distance to oldest location: " << initDist.length() << endl;
                                std::cout << "Oldest Position: (" << oldest.x <<"," << oldest.y <<")" << endl;
                                std::cout << "Current Position: (" << vehicleCoord.x <<"," << vehicleCoord.y <<")" << endl;
                                std::cout << "RSU Position: (" << rsu.second.x <<"," << rsu.second.y <<")" << endl;
                                std::cout << "Entry Point: (" << get<0>(output[0]) << "," << get<1>(output[0]) << ")" << endl ;
                                std::cout << "Exit Point: (" << get<0>(output[1]) << "," << get<1>(output[1]) << ")" << endl ;
                                */

                                Coord entry;
                                Coord exit;
                                Coord a(get<0>(output[0]), get<1>(output[0]));
                                Coord b(get<0>(output[1]), get<1>(output[1]));

                                if(linearDistance(a,vehicleCoord) < linearDistance(b, vehicleCoord)){
                                    entry = a;
                                    exit = b;
                                }
                                else{
                                    entry = b;
                                    exit = a;
                                }

                                double oldestTime = get<2>(registry.vehicleRegistry[vehicleID][size-1]) ;
                                double velocity = calculateVelocity(vehicleCoord, oldest, simulationTime, oldestTime);

                                double totalSpeed = 0;
                                int count = 0;

                                std::vector<std::vector<double>> X;
                                std::vector<double> Y;

                                for(int j=0; j < size; j++){
                                    totalSpeed += std::get<1>(registry.vehicleRegistry[vehicleID][j]);
                                    count += 1;

                                    Coord current = get<0>(registry.vehicleRegistry[vehicleID][j]);
                                    Coord next = get<0>(registry.vehicleRegistry[vehicleID][j+1]);


                                    if(j < size-1){

                                       std::vector<double> xTrain = {linearDistance(current, next), (get<1>(registry.vehicleRegistry[vehicleID][j]) + get<1>(registry.vehicleRegistry[vehicleID][j+1])) / 2 };
                                        X.push_back(xTrain);
                                        Y.push_back(get<2>(registry.vehicleRegistry[vehicleID][j]) - get<2>(registry.vehicleRegistry[vehicleID][j+1]));

                                        /*
                                        std::cout << "{"
                                        << linearDistance(current, next)
                                        << ", "
                                        << (get<1>(registry.vehicleRegistry[vehicleID][j]) + get<1>(registry.vehicleRegistry[vehicleID][j+1])) / 2 << "}, "
                                        << get<2>(registry.vehicleRegistry[vehicleID][j]) - get<2>(registry.vehicleRegistry[vehicleID][j+1])
                                        << endl ;
                                        */
                                    }
                                }


                                LinearRegression mlr(X, Y, NODEBUG);
                                mlr.fit();

                                double averageSpeed = totalSpeed/count;
                                double entryDistance = linearDistance(vehicleCoord, entry);
                                double dwellDistance = linearDistance(entry, exit);
                                int timeToReach = mlr.predict({ entryDistance, averageSpeed  });
                                int dwellTime = mlr.predict({dwellDistance, averageSpeed});

                                /*
                                std::cout << "Distance to entry: " << entryDistance << endl;
                                std::cout<< "Average Speed: " << averageSpeed << endl ;
                                std::cout << "Distance / Speed: " << entryDistance/averageSpeed << endl;
                                std::cout << "MLR Predicted time to reach: " << timeToReach << endl ;
                                std::cout << "MLR Predicted Dwell Distance: " << dwellDistance << endl ;
                                std::cout << "MLR Predicted DT: " << dwellTime << endl ;
                                std::cout << "Distance to Entry: " << distEntry << endl;
                                std::cout << "Distance to Exit: " << linearDistance(vehicleCoord, exit) << endl;
                                */

                                // Sending a WSM Message to the nextRSU
                                TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
                                populateWSM(wsm);
                                wsm->setSenderType(1);
                                wsm->setSenderAddress(vehicleID);
                                wsm->setMessageTime((int)simTime().dbl());
                                wsm->setTargetAddress(rsu.first);
                                wsm->setSenderSpeedRLDCO(vehicleSpeed);
                                wsm->setSenderVelocity(velocity);
                                wsm->setAverageSpeed(averageSpeed);
                                wsm->setSenderPositionRLDCO(vehicleCoord);
                                wsm->setEntryCoord(entry);
                                wsm->setExitCoord(exit);
                                wsm->setTimeToReach(timeToReach);
                                wsm->setDwellDistance(dwellDistance);
                                wsm->setDwellTime(dwellTime);
                                wsm->setHopCountRLDCO(0);
                                sendDown(wsm);

                                //std::cout << "----------------------------------" << endl;

                            }

                            else{
                                /*
                                cout << "Vehicle too far" << endl;
                                */
                            }


                        }// if moving towards.


                    } // if intersects. not empty.

                } // if out of range


            } // For each known rsu

        } // if size > 2

    } // at specific simtime

}



void TraCIDemo11p::onWSA(DemoServiceAdvertisment* wsa)
{

}

void TraCIDemo11p::onWSM(BaseFrame1609_4* frame)
{


    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);

    int hopCount = wsm->getHopCountRLDCO();

    // Ignoring self wsm
    if(wsm->getSenderAddress() != mobility->getId()){

        // If wsm from another car
        if(  wsm->getSenderType() == 1){
            //std::cout << "A vehicle received this message from a another vehicle" << std::endl;
            //std::cout << "Sender Type: " << wsm->getSenderType() << endl;

            Coord vehicleCoord = wsm->getSenderPositionRLDCO();
            int vehicleID = wsm->getSenderAddress();
            float vehicleSpeed = wsm->getSenderSpeedRLDCO();

            /*
            std::cout << "Sender Type: " << wsm->getSenderType()  << endl;
            std::cout << "Sender Address: " << vehicleID  << endl;
            std::cout << "Sender Speed: " << vehicleSpeed << endl;
            std::cout << "Sender Position: " << vehicleCoord << endl;
            std::cout << "Current Hop Count: " << hopCount << endl;
            */

            if(wsm->getHopCountRLDCO() < 3){
                wsm->setHopCountRLDCO(hopCount+1);
                sendDelayedDown(wsm->dup(), uniform(1,3));
                //std::cout << "Flooding." << endl;
            }

            else{
                //std::cout << "Three hops completed. Discarding WSM" << endl;
            }

            //std::cout  << "-----------------------------" << endl;

        }

        else{ // If WSM from a RSU

            //std::cout << "A vehicle received this message from a RSU" << std::endl;

            Coord rsuCoord = wsm->getSenderPositionRLDCO();
            int rsuID = wsm->getSenderAddress();

            // Adding to RSU Registry if not already present
            if ( registry.rsuRegistry.find(rsuID) == registry.rsuRegistry.end() ) {
                registry.rsuRegistry[rsuID] = rsuCoord ;
            }


            if(wsm->getHopCountRLDCO() < 3){
                wsm->setHopCountRLDCO(hopCount+1);
                sendDelayedDown(wsm->dup(), uniform(1,3)); // waits delay seconds before sending ? this avoids packet collision. uniform creates a random delay
            }

            else{
                //std::cout << "All hops completed. Discarding WSM" << endl;
            }

        }

    }

    else{
        //std::cout<< "Self WSM Message" << endl ;
        //std::cout  << "-----------------------------" << endl;
    }

} // onWSM

void TraCIDemo11p:: onBSM(DemoSafetyMessage* bsm)
{

}

void TraCIDemo11p::handleSelfMsg(cMessage* msg)
{
    //std::cout << "Is this the problem?" << endl;
    DemoBaseApplLayer::handleSelfMsg(msg);

}

void TraCIDemo11p::finish()
{
    DemoBaseApplLayer::finish();

    //std::cout << simTime().dbl() << endl ;

    if(simTime().dbl() == 3000){
        std::cout << "And now my watch has ended: " << mobility->getId() << endl;
        std::cout << simTime().dbl() << endl;
        std::cout << "............................" << endl << endl;


        for(int i=0; i< 5; i++){
            std::cout << "Coordinate : " << get<0>(registry.vehicleRegistry[262][i])<< endl;
            std::cout << "Speed : " << get<1>(registry.vehicleRegistry[262][i])<< endl;
            std::cout << "Simtime : " << get<2>(registry.vehicleRegistry[262][i])<< endl;
            std::cout << "----------------------------------" << endl;
        }


        LinearRegression mlr({
            {13828.2,20908.8, 10.94},
            {13851,20946.5, 14.77 },
            {13926.5,20925.2, 14.09 },
            {13982.3,20894.2, 6.92},
            {14039.6,20865, 14.49}
        },
            {1030, 1025, 1020, 1015, 1010},

            NODEBUG);
        mlr.fit();
        std::cout << mlr.predict({ 14500, 21700, 12.50 }) << endl ;
        std::cout << "----------------------------------" << endl;


        for (auto const& x : registry.rsuRegistry)
        {
            std::cout << x.first << ':' << x.second << std::endl ;
        }

    }






    // statistics recording goes here
}


