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
#include <veins/modules/application/traci/CommonVars.h>
#include "veins/base/utils/Coord.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"



#include<ctime>

using namespace veins;
using namespace std;


Define_Module(veins::TraCIDemoRSU11p);

Registry edge;
int rsuRange = 400;
int rangeThreshold = 4;
std::map<int, double> alpha;
int inRangeMsgRcv = 0;
int wsmReceived = 0;
int numberVehicles = 1000;
int appID = 0;

int rsuList[6] = {14,19,24,29,34,39};


std::map<int, std::map<int , int>> bandWidth {
    {14, {{19,rand() % 20 + 0},{24,rand() % 20 + 0},{29,rand() % 20 + 0},{34,rand() % 20 + 0},{39,rand() % 20 + 0}} },
    {19, {{14,rand() % 20 + 0},{24,rand() % 20 + 0},{29,rand() % 20 + 0},{34,rand() % 20 + 0},{39,rand() % 20 + 0}} },
    {24, {{19,rand() % 20 + 0}, {14,rand() % 20 + 0},{29,rand() % 20 + 0},{34,rand() % 20 + 0},{39,rand() % 20 + 0}} },
    {29, {{19,rand() % 20 + 0},{24,rand() % 20 + 0},{14,rand() % 20 + 0},{34,rand() % 20 + 0},{39,rand() % 20 + 0}} },
    {34, {{19,rand() % 20 + 0},{24,rand() % 20 + 0},{29,rand() % 20 + 0},{14,rand() % 20 + 0},{39,rand() % 20 + 0}} },
    {39, {{19,rand() % 20 + 0},{24,rand() % 20 + 0},{29,rand() % 20 + 0},{34,rand() % 20 + 0},{14,rand() % 20 + 0}} },
};

double rMatrix [6] [6] = {
        {100, 100, 100, 100, 100, 100},
        {100, 100, 100, 100, 100, 100},
        {100, 100, 100, 100, 100, 100},
        {100, 100, 100, 100, 100, 100},
        {100, 100, 100, 100, 100, 100},
        {100, 100, 100, 100, 100, 100},
};


//double rMatrix [6] [6] = {
//        {-1, -1, -1, -1, 100, 100 },
//        {100, 100, -1, -1, 100, 100},
//        {100, 100, -1, 100, 100, 100 },
//        {100, 100, -1, -1, 100, 100 },
//        {100, -1, -1, -1, 100, 100 },
//        {100, 100, -1, -1, 100, 100},
//
//};

double qMatrix [6] [6] = {
        {0,0,0,0,0,0},
        {0,0,0,0,0,0},
        {0,0,0,0,0,0},
        {0,0,0,0,0,0},
        {0,0,0,0,0,0},
        {0,0,0,0,0,0},
};



double gamma = 0.75;
double qAlpha = .95;


// processing, storage, bandwidth
double influence [3] = {0.5, 0.30, 0.20};



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


    if(wsm->getInRange() == true){
        int vehicleID = wsm->getSenderAddress();
        int messageTime = wsm->getMessageTime();
        //std::cout << "Vehicle in Range: " << messageTime << ". at: " << myId << ". Vehicle ID: " << vehicleID << endl;
        inRangeMsgRcv++;
//        wsmReceived++;
//        std::cout << "Total Message Received: " << inRangeMsgRcv  << endl ;
//        std::cout << "---------------------------------"<< endl;
        edge.inRangeVehicle[myId][messageTime].insert(edge.inRangeVehicle[myId][messageTime].begin(), vehicleID);

    }


    // If wsm is from vehicle and targeted to this RSU
    if(wsm->getSenderType() == 1 && wsm->getTargetAddress() == myId){


        int vehicleID = wsm->getSenderAddress();
        int messageTime = wsm->getMessageTime();


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
            int availableResource = wsm->getAvailableResource();
            int availableStorage = wsm->getAvailableStorage();

//            std::cout << availableStorage << endl;
//            std::cout << "---------------"<< endl;
//
//            Coord dist(vehicleCoord - rsuCoord);
//            if(dist.length() > rsuRange){
//                std::cout<< dist.length() << endl;
//                std::cout<< "RSU Received this message" << endl;
//                std::cout<< "Dwell Time: " << dwellTime << endl;
//                std::cout<< "Time to Reach: " << timeToReach<< endl;
//                std::cout<< "Dwell Start: " << dwellStart << endl;
//                std::cout<< "Dwell End: " << dwellEnd << endl;
//                std::cout << "---------------" << endl;
//            }

            std::tuple<int, double, int, int, int, int> vehicleData (messageTime, dwellDistance, dwellStart, dwellEnd, availableResource, availableStorage );
            edge.msgRegistry[myId][vehicleID] = vehicleData;

        } // message doesnt exists in database

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

        traffic_flow_event = new cMessage("traffic_flow_event");
        scheduleAt(simTime()+5, traffic_flow_event);

        rl_event = new cMessage("rl_event");
        scheduleAt(simTime()+2, rl_event);

        app_status = new cMessage("app_status");
        scheduleAt(simTime()+2, app_status);

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


int findIndex(int rsuID){
    int index;
    for(int i=0; i<6; i++){
        if(rsuList[i]== rsuID){
            index = i;
            break;
        }
    }
    return index;
}

std::vector<int> available_actions(int state){

    std::vector<int> av_act;

    for (int i = 0; i < 6; i++){
       if(rMatrix[state][i] >= 0){
            av_act.push_back(i);
        }
    }

    return av_act;
}


void updateQMatrix(int current_state, int action){
    double max_value = 0;
    std::vector<int> available_acts = available_actions(action);

    for(int j=0; j< available_acts.size(); j++){
        if(qMatrix[action][available_acts.at(j)] >= max_value)
        max_value = qMatrix[action][available_acts.at(j)];
    }

    qMatrix[current_state][action] = ((1-qAlpha)*qMatrix[current_state][action]) +  (qAlpha * (rMatrix[current_state][action] + (gamma* max_value)));
}


int chooseFogContRL(int appID, int simulationTime, int originRSU, int resourceReq, int storageReq, int appStart, int appEnd){

    std::vector<std::tuple<int, int, int, int, double>> probFog;
    int score;
    int highestScore=0;
    int chosenFog;

    for(int i=0; i<6; i++){
        int resourcePoolCont = 0 ;
        int storagePoolCont = 0;
        if(rsuList[i]!=originRSU ){
            for (auto const& vehicle : edge.msgRegistry[rsuList[i]]){
                if(appStart >= get<2>(vehicle.second) && appEnd <= get<3>(vehicle.second) ){
                    resourcePoolCont = resourcePoolCont + get<4>(vehicle.second);
                    storagePoolCont = storagePoolCont + get<5>(vehicle.second);
                }
            }

            if(resourcePoolCont >= resourceReq && storagePoolCont >= storageReq){
                score = (resourcePoolCont * influence[0]) + ( storagePoolCont * influence[1] ) + ( bandWidth[originRSU][rsuList[i]]  * influence[2]);
                if(score >= highestScore) {
                    highestScore= score;
                    //chosenFog = rsuList[i];
                }
                std::tuple<int, int, int, int, double> probDetails (rsuList[i], resourcePoolCont, storagePoolCont, bandWidth[originRSU][rsuList[i]], score);
                probFog.push_back(probDetails);
            }

        } // if not origin
    }

    if(probFog.size()!=0){ // ALL the RL Magic Here
        std::cout << "RL Enabled Probable Fogs: " << endl ;
        double max_value = 0;
        int state = findIndex(originRSU);
        int max_index;
        int action;
        for(auto itr : probFog){
            std::cout << get<0>(itr) << ", Resource: " << get<1>(itr) << ", Storage: " << get<2>(itr) << ", Bandwidth: " << get<3>(itr) << ", Score: " << get<4>(itr)
                    << endl;
            action = findIndex(get<0>(itr));

            if(qMatrix[state][action] >= max_value){
                max_value = qMatrix[state][action];
                max_index = action;
            }

        }

        std::cout << "-------------------------" << endl;
        return rsuList[action];
    }

    else{

        return 0;
    }

}


int chooseFogCont(int appID, int simulationTime, int originRSU, int resourceReq, int storageReq, int appStart, int appEnd){

    //std::vector<int> probFog;
    //std::cout << "Choose Fog Continuous Called." << endl ;
    std::vector<std::tuple<int, int, int, int, double>> probFog;
    int score;
    int highestScore=0;
    int chosenFog;

    for(int i=0; i<6; i++){
        int resourcePoolCont = 0 ;
        int storagePoolCont = 0;
        if(rsuList[i]!=originRSU ){
            for (auto const& vehicle : edge.msgRegistry[rsuList[i]]){
                if(appStart >= get<2>(vehicle.second) && appEnd <= get<3>(vehicle.second) ){
                    resourcePoolCont = resourcePoolCont + get<4>(vehicle.second);
                    storagePoolCont = storagePoolCont + get<5>(vehicle.second);
                }
            }

            if(resourcePoolCont >= resourceReq && storagePoolCont >= storageReq){
                score = (resourcePoolCont * influence[0]) + ( storagePoolCont * influence[1] ) + ( bandWidth[originRSU][rsuList[i]]  * influence[2]);
                if(score >= highestScore) {
                    highestScore= score;
                    chosenFog = rsuList[i];
                }
                std::tuple<int, int, int, int, double> probDetails (rsuList[i], resourcePoolCont, storagePoolCont, bandWidth[originRSU][rsuList[i]], score);
                probFog.push_back(probDetails);
            }

        } // if not origin
    }

    if(probFog.size()!=0){
//        for(auto itr : probFog){
//
//        }
        return chosenFog;
    }

    else{
        return 0;
    }

}

int chooseFogDist(int appID, int simulationTime, int originRSU, int resourceReq, int storageReq, int appStart, int appEnd){

    std::vector<std::tuple<int, int, int, int, double>> probFog;
    //std::cout << "Choose Fog Disrupted Called." << endl ;

    int score;
    int highestScore=0;
    int chosenFog;

    for(int i=0; i<6; i++){

        int lowest = 9999;
        int lowestResourceDist = 9999;
        int lowestStorageDist = 9999;

        if(rsuList[i]!=originRSU ){

            for (int j = appStart ; j <= appEnd ; j++){
                int currentTotal = 0;
                int resourcePoolDist=0;
                int storagePoolDist = 0;
                for (auto const& vehicle : edge.msgRegistry[rsuList[i]]){
                    if(j >= get<2>(vehicle.second) || j <= get<3>(vehicle.second) ){
                        currentTotal++;
                        resourcePoolDist = resourcePoolDist + get<4>(vehicle.second);
                        storagePoolDist = storagePoolDist + get<5>(vehicle.second);
                    }
                }

                if(currentTotal <= lowest){
                   lowest = currentTotal;
                   lowestResourceDist = resourcePoolDist;
                   lowestStorageDist = storagePoolDist;
                }
            }

            if(lowestResourceDist >= resourceReq && lowestStorageDist >= storageReq){
                score = (lowestResourceDist * influence[0]) + ( lowestStorageDist * influence[1] ) + ( bandWidth[originRSU][rsuList[i]]  * influence[2]);
                if(score >= highestScore){
                    highestScore = score;
                    chosenFog = rsuList[i];
                }
                std::tuple<int, int, int, int, double> probDetails (rsuList[i], lowestResourceDist, lowestStorageDist, bandWidth[originRSU][rsuList[i]], score );
                probFog.push_back(probDetails);
            }
        } // if not origin

    }

    if(probFog.size()!=0){
        //std::cout << "Probable Fogs: " << endl ;
//        for(auto itr : probFog){
//           // std::cout << get<0>(itr) << ", Resource: " << get<1>(itr) << ", Storage: " << get<2>(itr) << ", Bandwidth: " << get<3>(itr) << ", Score: " << get<4>(itr)
//                    //<< endl;
//        }
        return chosenFog;
    }

    else{
        return 0;
    }

}


int appStatusRL(int id){

    if(get<0>(edge.appRegistryRL[id]) == 0){ // Continuous
        int resourcePoolCont=0;
        int storagePoolCont=0;
        std::vector<int> vehiclesAvailable;

        for (auto const& vehicle : vehicleLocation){

            if(get<2>(vehicle) == get<5>(edge.appRegistryRL[id])){
                if(get<1>(vehicle) >= get<2>(edge.appRegistryRL[id])  && get<1>(vehicle) <= get<3>(edge.appRegistryRL[id])){
                    vehiclesAvailable.push_back(get<0>(vehicle));
                }

            }
        }

        sort( vehiclesAvailable.begin(),  vehiclesAvailable.end() );
        vehiclesAvailable.erase( unique(  vehiclesAvailable.begin(),  vehiclesAvailable.end() ),  vehiclesAvailable.end() );


        for (auto const& vehicle : vehiclesAvailable){
            resourcePoolCont += get<0>(vehicleResource[vehicle]);
            storagePoolCont += get<1>(vehicleResource[vehicle]);
        }

        if(resourcePoolCont >= get<6>(edge.appRegistryRL[id]) && storagePoolCont >= get<7>(edge.appRegistryRL[id])){
            return 7;
        }
        else{
            return 5;
        }

    }

    else{ // Disruptive
        return 7;
    }
}


int appStatus(int id){

    if(get<0>(edge.appRegistry[id]) == 0){
        //std::cout<< "App needs continuous resource" << endl;
        int resourcePoolCont=0;
        int storagePoolCont=0;
        std::vector<int> vehiclesAvailable;

        for (auto const& vehicle : vehicleLocation){

            if(get<2>(vehicle) == get<5>(edge.appRegistry[id])){

                //std::cout << get<2>(vehicle) << " " << get<5>(edge.appRegistry[id]) <<" " << get<1>(vehicle) << " "
                                                            //<<get<2>(edge.appRegistry[id]) << " " << get<3>(edge.appRegistry[id])
                                                            //<< endl;
                if(get<1>(vehicle) >= get<2>(edge.appRegistry[id])  && get<1>(vehicle) <= get<3>(edge.appRegistry[id])){
                    vehiclesAvailable.push_back(get<0>(vehicle));
                }

            }
        }

        sort( vehiclesAvailable.begin(),  vehiclesAvailable.end() );
        vehiclesAvailable.erase( unique(  vehiclesAvailable.begin(),  vehiclesAvailable.end() ),  vehiclesAvailable.end() );

        //std::cout << "Actual Vehicles Available: " << vehiclesAvailable.size() << endl;

        for (auto const& vehicle : vehiclesAvailable){
            resourcePoolCont += get<0>(vehicleResource[vehicle]);
            storagePoolCont += get<1>(vehicleResource[vehicle]);
        }

        //std::cout<< "Resource: " << resourcePoolCont << ", Storage: "<< storagePoolCont << endl;

        if(resourcePoolCont >= get<6>(edge.appRegistry[id]) && storagePoolCont >= get<7>(edge.appRegistry[id])){
            //std::cout<< "Assigned and Served." << endl;
            return 7;
        }
        else{
            //std::cout<< "Assigned and Failed." << endl;
            return 5;
        }

    }
    else{
        //std::cout<< "App needs Disrupted resource" << endl;
        return 7;
    }
}



void TraCIDemoRSU11p::handleSelfMsg(cMessage* msg)
{

    int simulationTime = (int) simTime().dbl();

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


        if( simulationTime > 149 &&  simulationTime % 50 == 0 && simulationTime != 1000){


            int resourcePoolCont=0;
            int storagePoolCont = 0;
            int status = 0; // 0 - Initiated. 1 - Assigned, 2 - Denied  5 -> Failed. 7 -> Success
            int servedRSU=99;
            double appStart = simulationTime + 0; // Previously was 25
            int executionTime = 5;
            int appEnd = appStart + executionTime;
            int chosenFog;

            int resourceReq = rand() % 10 + 10;
            int storageReq = rand() % 10 + 10;

            //int appType = rand() % 2; // 0 - Continuous connectivity, 1 - Disrupted connectivity
            int appType = 0 ;

            predictLog.open("results/phase2/ahp/applications.csv",  ios::out | ios::app);
                       predictLog << appID << ", " << appType << ", " << simulationTime <<", "
                               <<  appStart << ", " <<  appEnd << ", " <<  myId << ", "
                               <<  chosenFog << ", " <<  resourceReq << ", " <<  storageReq
                               << ", " <<  0 << ", " <<  status << "\n" ;
                       predictLog.close();

            //Application --> resource, deadline
//            Don't remove this section. 0
//            std::map<int, std::tuple<int,int>> applications;
//            int appCount = rand() % 10 + 0;
//
//            for(int i=0; i< appCount ; i++ ){
//               std::tuple<int, int> appData (rand() % 10 + 10, rand() % 10 + 10 );
//               applications[i] = appData;
//            }
//             If we want to implement multiple application requests and choosing algorithm.


            int vehicleCountThrough = 0;
            //int volume = 0;

            for (auto const& vehicle : edge.msgRegistry[myId]){
                if(appStart >= get<2>(vehicle.second) && appEnd <= get<3>(vehicle.second) ){
                    vehicleCountThrough++;
                    resourcePoolCont = resourcePoolCont + get<4>(vehicle.second);
                    storagePoolCont = storagePoolCont + get<5>(vehicle.second);
                }
            }

            int lowest = 9999;
            int disrupted = 0;
            int lowestResourceDist = 9999;
            int lowestStorageDist = 9999;

            for (int j = appStart ; j <= appEnd ; j++){
                int currentTotal = 0;
                int resourcePoolDist=0;
                int storagePoolDist=0;
                for (auto const& vehicle : edge.msgRegistry[myId]){
                    if(j >= get<2>(vehicle.second) || j <= get<3>(vehicle.second) ){
                        currentTotal++;
                        resourcePoolDist = resourcePoolDist + get<4>(vehicle.second);
                        storagePoolDist = storagePoolDist + get<5>(vehicle.second);
                    }
                }

                if(currentTotal <= lowest){
                   lowest=currentTotal;
                   lowestResourceDist = resourcePoolDist;
                   lowestStorageDist = storagePoolDist;
                }
            }

//            std::cout << "App Id: " << appID <<endl;
//            std::cout << "App Request on RSU: " << myId <<endl;
//            std::cout << "Application request at: " << simulationTime <<endl;
//            std::cout << "App type: " << appType <<endl;
//            std::cout << "Computation: " << resourceReq << " Storage: " << storageReq <<endl;
//            std::cout << "Continuous Conn: " << vehicleCountThrough <<endl;
//            std::cout << "Continuous Conn Computation: " << resourcePoolCont << " Storage: "<< storagePoolCont <<endl;
//            std::cout << "Disrupted Conn: " << lowest <<endl ;
//            std::cout << "Disrupted Conn Resource: " << lowestResourceDist << " Storage: " << lowestStorageDist << endl ;



            /*************************
             * Simple AHP Based Decision
             ***************************/

            if((appType== 0 && resourceReq <= resourcePoolCont && storageReq<= storagePoolCont) || (appType== 1 && resourceReq <= lowestResourceDist && storageReq <= lowestStorageDist)){ // Continuous/Disrupted and Served
               chosenFog = myId;
               status = 1; // Assigned
            }

            else if(appType==0){
                chosenFog = chooseFogCont(appID,  simulationTime, myId, resourceReq, storageReq, appStart, appEnd);
                if(chosenFog==0){
                    status = 2;
                }
                else{
                    status = 1;
                }
            }

            else if(appType==1){
                chosenFog = chooseFogDist(appID, simulationTime, myId, resourceReq, storageReq, appStart, appEnd);
                if(chosenFog==0){
                    status = 2;
                }
                else{
                    status = 1;
                }
            }

            // Inserting into application registry. // ID, type, time, rsu, servedRSU, resource, storage, deadline, status
            std::tuple<int, int, int, int, int, int, int, int, int, int> appData (appType, simulationTime, appStart, appEnd,  myId, chosenFog,  resourceReq, storageReq, 0, status );
            edge.appRegistry[appID] = appData;

            //Logging Applications for AHP
//            predictLog.open("results/phase2/ahp/applications.csv",  ios::out | ios::app);
//            predictLog << appID << ", " << appType << ", " << simulationTime <<", "
//                    <<  appStart << ", " <<  appEnd << ", " <<  myId << ", "
//                    <<  chosenFog << ", " <<  resourceReq << ", " <<  storageReq
//                    << ", " <<  0 << ", " <<  status << "\n" ;
//            predictLog.close();
            /*************************
            * Simple AHP Based Decision Ends
            ***************************/



            /***********
             *  RL Implemented Decision
             ****************/

            if((appType== 0 && resourceReq <= resourcePoolCont && storageReq<= storagePoolCont) || (appType== 1 && resourceReq <= lowestResourceDist && storageReq <= lowestStorageDist)){ // Continuous/Disrupted and Served
               chosenFog = myId;
               status = 1; // Assigned
            }

            else if(appType==0){
                chosenFog = chooseFogContRL(appID,  simulationTime, myId, resourceReq, storageReq, appStart, appEnd);
                if(chosenFog==0){
                    status = 2;
                }
                else{
                    status = 1;
                }
            }

            else if(appType==1){
                chosenFog = chooseFogDist(appID, simulationTime, myId, resourceReq, storageReq, appStart, appEnd);
                if(chosenFog==0){
                    status = 2;
                }
                else{
                    status = 1;
                }
            }

            // Inserting into application registry. // ID, type, time, rsu, servedRSU, resource, storage, deadline, status
            std::tuple<int, int, int, int, int, int, int, int, int, int> appDataRL (appType, simulationTime, appStart, appEnd,  myId, chosenFog,  resourceReq, storageReq, 0, status );
            edge.appRegistryRL[appID] = appDataRL;

//            //Logging Applications for RL
//            predictLog.open("results/phase2/rl/applications.csv",  ios::out | ios::app);
//            predictLog << appID << ", " << appType << ", " << simulationTime <<", "
//                    <<  appStart << ", " <<  appEnd << ", " <<  myId << ", "
//                    <<  chosenFog << ", " <<  resourceReq << ", " <<  storageReq
//                    << ", " <<  0 << ", " <<  status << "\n" ;
//            predictLog.close();

             /***********
             *  RL Implemented Decision Ends
             ****************/



            appID++;

//            std::cout << "Application Status: " << status <<endl;
//            std::cout << "-----------------------" <<endl ;
//
//
//             Logging predictions
//            predictLog.open("results/dis2/predictions_random_25_05_linear_1000_400m_hop4_disrupted.csv",  ios::out | ios::app);
//            predictLog << myId << ", " << appStart << "," << appEnd << ","<< lowest/6 << "\n";
//            predictLog.close();
//
//
//             Logging volume in rsu database
//            edge.predictedVolume[myId][simulationTime] = volume;

        }

        scheduleAt(simTime() + 1 , request_event);

    }

    if(msg == dwellTime_event){


        if( simulationTime % 51 == 0 &&  simulationTime != 1000 && simulationTime > 200){

            int nVc = 3;
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

//            std::cout << "Dwell time estimation at: " << simulationTime << "s. At RSU: " << myId << ". Dwell Time: " << dwellTime<<  endl ;
//            std::cout << "--------------------------" << endl ;
//
//             Logging predictions
//            predictLog.open("results/dis2/dwellTime_random_25_05_linear_1000_400m_hop4_disrupted.csv",  ios::out | ios::app);
//            predictLog << myId << ", " << simulationTime << "," << dwellTime*3 << "\n";
//            predictLog.close();
//
//             Logging to RSU database
//
//            edge.dtPrediction[myId][simulationTime] = dwellTime;
//
//            int vehicleCount = traci->get

        }

        scheduleAt(simTime() + 1 , dwellTime_event);
    }


    if(msg == rl_event){ // All the RL magic here. Updating R matrix and Q matrix.

        if( (simulationTime > 149 && myId == 14) &&  (simulationTime == 156 || simulationTime == 206 || simulationTime == 256 || simulationTime == 306
                || simulationTime == 356 || simulationTime == 406 || simulationTime == 456 || simulationTime == 506 || simulationTime == 556 || simulationTime == 606
                || simulationTime == 656 || simulationTime == 706 || simulationTime == 756 || simulationTime == 806 || simulationTime == 856  || simulationTime == 906
                || simulationTime == 956)){

            // All the RL Stuff
//            std::cout<< "################################" << endl;
//            std::cout<< "This is an RL Training Event" << endl;

            int RLoriginFog;
            int RLchosenFog;
            int serveStatus;


            for(int i=appID-1; i >= appID-6; i--){

                // Check if app was assigned
                if(std::get<9>(edge.appRegistryRL[i])== 1){
                    RLoriginFog = std::get<4>(edge.appRegistryRL[i]);
                    RLchosenFog = std::get<5>(edge.appRegistryRL[i]);
//                    std::cout<< "Type: " << std::get<0>(edge.appRegistryRL[i]) << ", Status: 1, " << "Origin: " << RLoriginFog << ", Chosen Fog: " << RLchosenFog
//                            << ", Resource Req: " << std::get<6>(edge.appRegistryRL[i]) << ", Storage Req: " << std::get<7>(edge.appRegistryRL[i])
//                            << endl;

                    serveStatus = appStatusRL(i);
                    //appStatus(i);

                    if(serveStatus == 5){
                        int originIndex = findIndex(RLoriginFog);
                        int chosenIndex = findIndex(RLchosenFog);
                        //std::cout<< "Assigned and Failed. Decreasing reward for: " << RLoriginFog << ", " <<RLchosenFog << ", ["<< originIndex << chosenIndex << "]"  <<  endl;
                        //rMatrix[originIndex][chosenIndex] = rMatrix[originIndex][chosenIndex] - 20 ;
                        rMatrix[originIndex][chosenIndex] = -1;
                        updateQMatrix(originIndex, chosenIndex);
                    }

                    else if(serveStatus == 7){
                        int originIndex = findIndex(RLoriginFog);
                        int chosenIndex = findIndex(RLchosenFog);
                        //std::cout<< "Assigned and Served. Increasing reward for: " << RLoriginFog << ", " <<RLchosenFog << ", ["<< originIndex << chosenIndex << "]"  <<  endl;
                        //rMatrix[originIndex][chosenIndex] = rMatrix[originIndex][chosenIndex] + 20 ;
                        rMatrix[originIndex][chosenIndex] = 100;
                        updateQMatrix(originIndex, chosenIndex);
                    }


                    //Logging Applications for RL
                    //predictLog.open("results/phase2/applications_rl_"+std::to_string(numberVehicles)+".csv",  ios::out | ios::app);
                   // predictLog << i << ", " << std::get<0>(edge.appRegistryRL[i]) << ", " << std::get<1>(edge.appRegistryRL[i]) <<", "
                    //       <<  std::get<2>(edge.appRegistryRL[i]) << ", " <<  std::get<3>(edge.appRegistryRL[i]) << ", " <<  std::get<4>(edge.appRegistryRL[i]) << ", "
                    //       <<  std::get<5>(edge.appRegistryRL[i]) << ", " <<  std::get<6>(edge.appRegistryRL[i]) << ", " <<  std::get<7>(edge.appRegistryRL[i])
                   //        << ", " <<  0 << ", " <<  serveStatus << "\n" ;
                   // predictLog.close();

                } // App was assigned

                else{ // app was denied

//                    predictLog.open("results/phase2/applications_rl_"+std::to_string(numberVehicles)+".csv",  ios::out | ios::app);
//                    predictLog << i << ", " << std::get<0>(edge.appRegistryRL[i]) << ", " << std::get<1>(edge.appRegistryRL[i]) <<", "
//                           <<  std::get<2>(edge.appRegistryRL[i]) << ", " <<  std::get<3>(edge.appRegistryRL[i]) << ", " <<  std::get<4>(edge.appRegistryRL[i]) << ", "
//                           <<  std::get<5>(edge.appRegistryRL[i]) << ", " <<  std::get<6>(edge.appRegistryRL[i]) << ", " <<  std::get<7>(edge.appRegistryRL[i])
//                           << ", " <<  0 << ", " <<  2 << "\n" ;
//                    predictLog.close();

                } // App was denied

            }


            //std::cout << vehicleLocation.size() << endl;

            //std::cout<< "###############################" << endl;

        }

        scheduleAt(simTime() + 2 , rl_event);
    }



    if(msg == app_status){ // Prediodical app status check and logging

        if( (simulationTime > 149 && myId == 14) &&  (simulationTime == 158 || simulationTime == 208 || simulationTime == 258 || simulationTime == 308
                || simulationTime == 358 || simulationTime == 408 || simulationTime == 458 || simulationTime == 508 || simulationTime == 558 || simulationTime == 608
                || simulationTime == 658 || simulationTime == 708 || simulationTime == 758 || simulationTime == 808 || simulationTime == 858  || simulationTime == 908
                || simulationTime == 958)){

            // All the RL Stuff
//            std::cout<< "################################" << endl;
//            std::cout<< "This is an app logging event" << endl;

            int originFog;
            int chosenFog;
            int serveStatus;


            for(int i=appID-1; i >= appID-6; i--){

                // Check if app was assigned
                if(std::get<9>(edge.appRegistry[i])== 1){
                    originFog = std::get<4>(edge.appRegistry[i]);
                    chosenFog = std::get<5>(edge.appRegistry[i]);
//                    std::cout<< "Type: " << std::get<0>(edge.appRegistry[i]) << ", Status: 1, " << "Origin: " << originFog << ", Chosen Fog: " << chosenFog
//                            << ", Resource Req: " << std::get<6>(edge.appRegistry[i]) << ", Storage Req: " << std::get<7>(edge.appRegistry[i])
//                            << endl;

                    serveStatus = appStatus(i);


                    //Logging Applications for RL
//                    predictLog.open("results/phase2/applications_ahp_"+std::to_string(numberVehicles)+".csv",  ios::out | ios::app);
//                    predictLog << i << ", " << std::get<0>(edge.appRegistry[i]) << ", " << std::get<1>(edge.appRegistry[i]) <<", "
//                           <<  std::get<2>(edge.appRegistry[i]) << ", " <<  std::get<3>(edge.appRegistry[i]) << ", " <<  std::get<4>(edge.appRegistry[i]) << ", "
//                           <<  std::get<5>(edge.appRegistry[i]) << ", " <<  std::get<6>(edge.appRegistry[i]) << ", " <<  std::get<7>(edge.appRegistry[i])
//                           << ", " <<  0 << ", " <<  serveStatus << "\n" ;
//                    predictLog.close();

                } // App was assigned

                else{ // app was denied

//                    predictLog.open("results/phase2/applications_ahp_"+std::to_string(numberVehicles)+".csv",  ios::out | ios::app);
//                    predictLog << i << ", " << std::get<0>(edge.appRegistry[i]) << ", " << std::get<1>(edge.appRegistry[i]) <<", "
//                           <<  std::get<2>(edge.appRegistry[i]) << ", " <<  std::get<3>(edge.appRegistry[i]) << ", " <<  std::get<4>(edge.appRegistry[i]) << ", "
//                           <<  std::get<5>(edge.appRegistry[i]) << ", " <<  std::get<6>(edge.appRegistry[i]) << ", " <<  std::get<7>(edge.appRegistry[i])
//                           << ", " <<  0 << ", " <<  2 << "\n" ;
//                    predictLog.close();

                } // App was denied

            }


            //std::cout << vehicleLocation.size() << endl;

            //std::cout<< "###############################" << endl;

        }

        scheduleAt(simTime() + 2 , app_status);
    }




    if(msg == traffic_flow_event && myId == 24){ //Estimating traffic flow

        int simulationTime = (int) simTime().dbl();

        /*
        std::cout<< "Traffic Flow Event at: " << myId << ". Simulation Time: " << simulationTime <<endl;
        double actualVolume = edge.inRangeVehicle[myId][simulationTime].size();

        int* a = &edge.inRangeVehicle[myId][simulationTime-4][0];

        //for(int i=0; i < edge.inRangeVehicle[myId][simulationTime-4].size(); i++)
         //   std::cout << a[i] << ' ';
       // std::cout<< endl;

        sort(edge.inRangeVehicle[myId][simulationTime-4].begin(), edge.inRangeVehicle[myId][simulationTime-4].end());
        sort(edge.inRangeVehicle[myId][simulationTime-3].begin(), edge.inRangeVehicle[myId][simulationTime-3].end());
        sort(edge.inRangeVehicle[myId][simulationTime-2].begin(), edge.inRangeVehicle[myId][simulationTime-2].end());
        sort(edge.inRangeVehicle[myId][simulationTime-1].begin(), edge.inRangeVehicle[myId][simulationTime-1].end());

        for(int i=0; i < edge.inRangeVehicle[myId][simulationTime-4].size(); i++)
                    std::cout << edge.inRangeVehicle[myId][simulationTime-4].at(i) << ' ';

        std::cout<< endl;

        for(int i=0; i < edge.inRangeVehicle[myId][simulationTime-3].size(); i++)
           std::cout << edge.inRangeVehicle[myId][simulationTime-3].at(i) << ' ';

        std::cout<< endl;

        for(int i=0; i < edge.inRangeVehicle[myId][simulationTime-2].size(); i++)
           std::cout << edge.inRangeVehicle[myId][simulationTime-2].at(i) << ' ';

        std::cout<< endl;

        for(int i=0; i < edge.inRangeVehicle[myId][simulationTime-1].size(); i++)
           std::cout << edge.inRangeVehicle[myId][simulationTime-1].at(i) << ' ';

        std::cout<< endl;

        std::cout<< edge.inRangeVehicle[myId][simulationTime-1].size() << endl;
        std::cout<< edge.inRangeVehicle[myId][simulationTime-2].size() << endl;
        std::cout<< "--------------" << endl;

        scheduleAt(simTime() + 10, traffic_flow_event);
        */
    }

}


void TraCIDemoRSU11p:: onBSM(DemoSafetyMessage* bsm)
{

}

void TraCIDemoRSU11p::finish()
{
    DemoBaseApplLayer::finish();
    if(simTime().dbl() == 1000 && myId == 24){
        //std::cout << "In Range Received: " << inRangeMsgRcv << endl;
        std::cout << "WSM Received: " << wsmReceived << endl;

        for(int i = 0; i< 6; i++){
            std::cout<< "{";
            for(int j= 0; j< 6; j++){
                std::cout<< rMatrix[i][j] << ", ";
            }
            std::cout<< "},";
            std::cout<< endl;
        }

        for(int i = 0; i< 6; i++){

            for(int j= 0; j< 6; j++){
                std::cout<< qMatrix[i][j] << ", ";
            }
            std::cout<< endl;
        }

    }

    /*for(auto itr : edge.appRegistry){
        cout << std::get<0>(itr) << ", " << std::get<1>(itr)<< ", "
                << std::get<2>(itr)<< ", " << std::get<3>(itr) << ", "
                << std::get<4>(itr)<< ", " << std::get<5>(itr)<< ", "
                << std::get<6>(itr)<<  ", " <<std::get<7>(itr) <<  ", " <<std::get<8>(itr) <<  endl ;

        cout << "------------------------" << endl;

    }*/


}

