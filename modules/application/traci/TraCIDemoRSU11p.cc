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
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

#include<ctime>

using namespace veins;
using namespace std;


Define_Module(veins::TraCIDemoRSU11p);

Registry edge;
int rsuRange = 1000;
int rangeThreshold = 3;
std::map<int, double> alpha;
int inRangeMsgRcv = 0;
int wsmReceived = 0;


double distance(Coord& a,  Coord& b) {
    Coord dist(a - b);
    return dist.length();
}

void updateAlpha(int rsuID, int simulationTime){

    std::cout << "Update alpha called for rsu:  "  << rsuID << endl;
    std::cout << "Previous Alpha: " << alpha[rsuID] << endl;

    double actualVolume = edge.inRangeVehicle[rsuID][simulationTime].size();
    double predictedVolume = edge.predictedVolume[rsuID][simulationTime];

    if(predictedVolume !=0 && actualVolume !=0){
        alpha[rsuID] = actualVolume/ predictedVolume;
    }

    std::cout<< "Actual Volume: " << actualVolume <<endl;
    std::cout<< "Predicted Volume: " << predictedVolume << endl ;
    std::cout << "Division: " << actualVolume/ predictedVolume << endl;
    std::cout<< "Updated Alpha: " << alpha[rsuID]<< endl;
    std::cout<< "------------------" << endl;

}


void TraCIDemoRSU11p::onWSM(BaseFrame1609_4* frame)
{

    TraCIDemo11pMessage* wsm = check_and_cast<TraCIDemo11pMessage*>(frame);


    // If wsm is from vehicle and targeted to this RSU
    if(wsm->getSenderType() == 1 && wsm->getTargetAddress() == myId){


        int vehicleID = wsm->getSenderAddress();
        int messageTime = wsm->getMessageTime();

        // Logging vehicle information that are already in range for analysis.
        if(wsm->getInRange() == true){
            //std::cout << "Vehicle in Range: " << messageTime << ". at: " << myId << ". Vehicle ID: " << vehicleID << endl;
            inRangeMsgRcv++;
            wsmReceived++;
           // std::cout << "Total Message Received: " << inRangeMsgRcv  << endl ;
            //std::cout << "---------------------------------"<< endl;
            edge.inRangeVehicle[myId][messageTime].insert(edge.inRangeVehicle[myId][messageTime].begin(), vehicleID);

        }

        else{

            // Finding the RSU Co-ordinate.
            BaseMobility *baseMob;
            baseMob = FindModule<BaseMobility*>::findSubModule(getParentModule());
            Coord rsuCoord = baseMob->getPositionAt(simTime().dbl());

            Coord vehicleCoord = wsm->getSenderPositionRLDCO();
            int lastKnown = get<0>(edge.msgRegistry[myId][vehicleID]);

            // is this a new message
            if(lastKnown != messageTime && lastKnown < messageTime ){

                wsmReceived++;

                //Retrieving WSM information
                float vehicleSpeed = wsm->getSenderSpeedRLDCO();
                int hopCount = wsm->getHopCountRLDCO();
                double dwellDistance = wsm->getDwellDistance();
                int dwellTime = wsm->getDwellTime();
                int timeToReach = wsm->getTimeToReach();
                int dwellStart = messageTime + timeToReach;
                int dwellEnd = dwellStart + dwellTime;

                std::tuple<int, double, int, int> vehicleData (messageTime, dwellDistance, dwellStart, dwellEnd );
                edge.msgRegistry[myId][vehicleID] = vehicleData;

            } // message doesnt exists in database

        } // targeted to this rsu

    }

}


// New Changes
void TraCIDemoRSU11p::initialize(int stage) {

    DemoBaseApplLayer::initialize(stage);


    if (stage == 0) {

        broadcast_event = new cMessage("boradcast_event");
        scheduleAt(simTime() + 5, broadcast_event);

        request_event = new cMessage("request_event");
        scheduleAt(simTime() + 1, request_event);

        dwellTime_event = new cMessage("dwellTime_event");
        scheduleAt(simTime() + 1, dwellTime_event);

        vehicle_event = new cMessage("vehicle_event");
        scheduleAt(simTime() + 1, vehicle_event);

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
        sendDelayedDown(wsm, uniform(1,3));

        if(simTime().dbl() < 100)
        scheduleAt(simTime() + 5, msg);

    }



    if(msg== request_event){ // Event is an application request

        int simulationTime = (int) simTime().dbl();

        if( simulationTime > 149 &&  simulationTime % 50 == 0 && simulationTime != 1000){

            if(simulationTime == 150){
                alpha[11] = 1.0;
                alpha[16] = 1.0;
                alpha[21] = 1.0;
            }

            //else{
            //    updateAlpha(myId, simulationTime-50);
            //}

            double appStart = simulationTime + 25;
            int executionTime = 5;
            int appEnd = appStart + executionTime;

            int vehicleCountThrough = 0;
            int volume = 0;

            for (auto const& vehicle : edge.msgRegistry[myId]){
                if(simulationTime >= get<2>(vehicle.second) && simulationTime <= get<3>(vehicle.second) ){
                    volume++;
                }

                if(appStart >= get<2>(vehicle.second) && appEnd <= get<3>(vehicle.second) ){
                    vehicleCountThrough++;
                }

            }

            // Logging predictions
            predictLog.open("results/predictions_random_25_05.csv",  ios::out | ios::app);
            predictLog << myId << ", " << appStart << "," << appEnd << ","<< vehicleCountThrough * alpha[myId] << "\n";
            predictLog.close();


            // Logging volume in rsu database
            edge.predictedVolume[myId][simulationTime] = volume;

        }

        scheduleAt(simTime() + 1 , request_event);
    }

    if(msg == dwellTime_event){

        int simulationTime = (int) simTime().dbl();

        if( simulationTime % 51 == 0 &&  simulationTime != 1000 && simulationTime > 200){

            int nVc = 10;
            int dwellTime;

            for(dwellTime = 0; dwellTime < 1000 - simulationTime; dwellTime++ ){

                int vehicleCount = 0;
                int volume = 0;

                for (auto const& vehicle : edge.msgRegistry[myId]){
                    if( (simulationTime + dwellTime) >= get<2>(vehicle.second) &&  (simulationTime + dwellTime) <= get<3>(vehicle.second) ){
                        vehicleCount++;
                    }

                }

                if(vehicleCount < nVc)
                    break;
            }

            //std::cout << "Dwell time estimation at: " << simulationTime << "s. At RSU: " << myId << ". Dwell Time: " << dwellTime<<  endl ;
            //std::cout << "--------------------------" << endl ;

            // Logging predictions
            predictLog.open("results/dwellTime_random_25_05.csv",  ios::out | ios::app);
            predictLog << myId << ", " << simulationTime << "," << dwellTime* alpha[myId] << "\n";
            predictLog.close();

            // Logging to RSU database

            //edge.dtPrediction[myId][simulationTime] = dwellTime;


        }

        scheduleAt(simTime() + 1 , dwellTime_event);



    }


    if(msg == vehicle_event){ // Check for vehicles in range

        /*
        TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
        populateWSM(wsm);
        wsm->setSenderType(0);
        wsm->setInRange(true);
        wsm->setSenderAddress(myId);
        sendDown(wsm);
        scheduleAt(simTime() + 1, vehicle_event);
        */

    }

}


void TraCIDemoRSU11p:: onBSM(DemoSafetyMessage* bsm)
{

}

void TraCIDemoRSU11p::finish()
{
    DemoBaseApplLayer::finish();
    if(simTime().dbl() == 1000){
        std::cout << "In Range Received: " << inRangeMsgRcv << endl;
        std::cout << "WSM Received: " << wsmReceived << endl;
    }

}

