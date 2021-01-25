//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
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


#include "TraCIDemoRSU11p.h"

#include <veins/modules/application/traci/Registry.h>
#include "veins/base/utils/Coord.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"


using namespace veins;
using namespace std;


Define_Module(veins::TraCIDemoRSU11p);

Registry edge;


//log.open("./results/node_results.txt");



double distance(Coord& a,  Coord& b) {
    Coord dist(a - b);
    return dist.length();
}


void TraCIDemoRSU11p::onWSM(BaseFrame1609_4* frame)
{

    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);


    // If wsm is from vehicle
    if(wsm->getSenderType() == 1){

        // WSM targeted to this RSU
        if(wsm->getTargetAddress() == myId){

            // Finding the RSU Co-ordinate.
            BaseMobility *baseMob;
            baseMob = FindModule<BaseMobility*>::findSubModule(getParentModule());
            Coord rsuCoord = baseMob->getPositionAt(simTime().dbl());

            int vehicleID = wsm->getSenderAddress();
            int messageTime = wsm->getMessageTime();
            int lastKnown = get<0>(edge.msgRegistry[myId][vehicleID]);
            Coord vehicleCoord = wsm->getSenderPositionRLDCO();

            if(lastKnown != messageTime && lastKnown < messageTime && distance(vehicleCoord, rsuCoord) < 2400 ){

                //Retrieving WSM information
                float vehicleSpeed = wsm->getSenderSpeedRLDCO();
                int hopCount = wsm->getHopCountRLDCO();
                double dwellDistance = wsm->getDwellDistance();
                int dwellTime = wsm->getDwellTime();
                int timeToReach = wsm->getTimeToReach();
                int dwellStart = messageTime + timeToReach;
                int dwellEnd = dwellStart + dwellTime;


                // if vehicle is out of range
                if(distance(vehicleCoord, rsuCoord) > 800){


                    //edge.msgRegistry[myId].insert(edge.msgRegistry[myId].begin(), {vehicleID, messageTime});
                    std::tuple<int, double, int, int> vehicleData (messageTime, dwellDistance, dwellStart, dwellEnd );

                    // If I want to keep historic data. Using a vector
                    //edge.msgRegistry[myId][vehicleID].insert(edge.msgRegistry[myId][vehicleID].begin(), vehicleData);

                    edge.msgRegistry[myId][vehicleID] = vehicleData;

                    //std::cout << "RSU Received this WSM Message from a vehicle out of range." << endl;
                    //std::cout << "Last Known Time: " << lastKnown << endl;
                    //std::cout << "RSU Position: " << rsuCoord << endl ;
                    //std::cout << "RSU ID: " << myId << endl;
                    //std::cout << "Message Target ID: " << wsm->getTargetAddress() << endl;
                    //std::cout << "Sender Type: " << wsm->getSenderType()  << endl;
                    //std::cout << "New Message Time (SimTime): " <<  messageTime << endl;
                    //std::cout << "Time to Reach: " << timeToReach << endl;
                    //std::cout << "Dwell Time: " << dwellTime << endl ;
                   // std::cout << "Dwell Start " << dwellStart << endl;
                   // std::cout << "Dwell End: " << dwellEnd << endl;
                    //std::cout << "Dwell Distance: " << dwellDistance << endl ;
                   // std::cout << "Vehicle ID: " << vehicleID  << endl;
                    //std::cout << "Inserted: " << get<0>(edge.msgRegistry[myId][vehicleID]) << endl;
                    //std::cout << "Current Speed: " << vehicleSpeed << endl;
                    //std::cout << "Average Speed: " << wsm->getAverageSpeed() << endl;
                    //std::cout << "Velocity : " << wsm->getSenderVelocity() << endl;
                    //std::cout << "Sender Position: " << wsm->getEntryCoord() << endl;
                    //std::cout << "Entry Co-ordinate: " << wsm->getEntryCoord() << endl;
                   // std::cout << "Exit Coordinate: " << wsm->getExitCoord() << endl;
                    //std::cout << "Hop Count: " << hopCount << endl;
                    //std::cout << "Distance from Vehicle: " << distance(vehicleCoord, rsuCoord) << endl;


                    /*
                    for (auto const& rsu : edge.msgRegistry)
                    {
                        if(rsu.first == myId){

                            for (auto const& vehicle : rsu.second){
                                std::cout << "....." << endl;
                                std::cout<< "Vehicle ID: " <<  vehicle.first << endl ;
                                std::cout << "Entry Time: " << get<2>(vehicle.second) << endl ;
                                std::cout << "Exit Time: " << get<3>(vehicle.second) << endl ;

                            }

                        }

                    }
                    */


                    //std::cout  << "----------------------------" << endl;

                }

                else{
                    /*
                    std::cout << "RSU Received this WSM Message from a vehicle within range." << endl;
                    std::cout << "RSU Position: " << rsuCoord << endl ;
                    std::cout << "RSU ID: " << myId << endl;
                    std::cout << "Message Target ID: " << wsm->getTargetAddress() << endl;
                    std::cout << "Sender Type: " << wsm->getSenderType()  << endl;
                    std::cout << "Sender Address: " << vehicleID  << endl;
                    std::cout << "Sender Speed: " << vehicleSpeed << endl;
                    std::cout << "Sender Position: " << vehicleCoord << endl; // (0,0,0)
                    std::cout << "Hop Count: " << hopCount << endl;
                    std::cout << "Distance from Vehicle: " << distance(vehicleCoord, rsuCoord) << endl;
                    std::cout  << "-------------------------" << endl;
                    */
                }

            } // message doesnt exists in database

            else{
                //std::cout << "Received same WSM again. Discarding" <<endl;
                //std::cout  << "-------------------------" << endl;
            }

        } // targeted to this rsu

    }

}


// New Changes
void TraCIDemoRSU11p::initialize(int stage) {

    DemoBaseApplLayer::initialize(stage);

     //std::cout << stage;


    if (stage == 0) {
        //std::cout << stage;
        //Initializing members and pointers of your application goes here
        //request_event = new cMessage("request_interval"); // request_resources is the message type
        //request_tolerance_event = new cMessage("request_tolerance"); // request_tollerance is the message type to finish the request process.

        broadcast_event = new cMessage("boradcast_event");
        scheduleAt(simTime() + 5, broadcast_event);

        request_event = new cMessage("request_event");
        scheduleAt(simTime() + 30, request_event);

        vehicle_event = new cMessage("vehicle_event");
        scheduleAt(simTime() + 2, vehicle_event);

        stage++;

    }



}

//New Changes

void TraCIDemoRSU11p::onWSA(DemoServiceAdvertisment* wsa)
{
    // if this RSU receives a WSA for service 42, it will tune to the chan
    if (wsa->getPsid() == 42) {
        mac->changeServiceChannel(static_cast<Channel>(wsa->getTargetChannel()));
    }
}




void TraCIDemoRSU11p::handleSelfMsg(cMessage* msg)
{


    if(msg == broadcast_event){

        BaseMobility *baseMob;
        baseMob = FindModule<BaseMobility*>::findSubModule(getParentModule());

        TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
        populateWSM(wsm);
        wsm->setSenderType(0);
        wsm->setSenderAddress(myId);
        wsm->setSenderPositionRLDCO(baseMob->getPositionAt(simTime().dbl()));
        wsm->setHopCountRLDCO(0);
        sendDown(wsm);

        if(simTime().dbl() < 100)
        scheduleAt(simTime() + 5, msg);

    }

    //else if(msg == request_tolerance_event)  {
        // Schedule the next interval to request, if necessary.
     //   scheduleAt(simTime() + request_interval_size + request_tolerance_size, request_tolerance_event);

    //}


    else if(msg== request_event){ // Event is an application request

        double appStart = simTime().dbl() + 65 ;
        int appEnd = appStart + 15;


       // std::cout << "App request at: " << simTime() << "s. At RSU: " << myId << endl ;
        //std::cout << "App start time: " << appStart << ". App end time: " << appEnd << endl ;

        int vehicleCountStart = 0;
        int vehicleCountThrough = 0;

        for (auto const& vehicle : edge.msgRegistry[myId]){
            if(appStart >= get<2>(vehicle.second) && appStart <= get<3>(vehicle.second) ){
                vehicleCountStart++;
            }

            if(appStart >= get<2>(vehicle.second) && appEnd <= get<3>(vehicle.second) ){
                vehicleCountThrough++;
            }

        }

        //std::cout << "Vehicles available at the time of start: " << vehicleCountStart << endl ;
        //std::cout << "Vehicles available throughout execution: " << vehicleCountThrough << endl ;
        //std::cout << "--------------------------" << endl ;

        // Logging predictions
        predictLog.open("results/predictions.csv",  ios::out | ios::app);
        predictLog << myId << ", " << appStart << "," << appEnd << ", " << vehicleCountStart << ","<< vehicleCountThrough << "\n";
        predictLog.close();


        scheduleAt(simTime() + 20, request_event);
    }


    else if(msg == vehicle_event){ // Check for vehicles in range

        TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
        populateWSM(wsm);
        wsm->setSenderType(0);
        wsm->setInRange(true);
        wsm->setSenderAddress(myId);
        sendDown(wsm);
        scheduleAt(simTime() + 2, vehicle_event);

    }


}


void TraCIDemoRSU11p:: onBSM(DemoSafetyMessage* bsm)
{








}

