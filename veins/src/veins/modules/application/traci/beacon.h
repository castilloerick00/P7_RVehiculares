// Archivo completo: veins/modules/application/traci/beacon.h

#pragma once

#include "veins/veins.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/base/utils/SimpleAddress.h"
#include <iomanip>
#include <string>

using namespace std;
using namespace veins;

class BeaconList {
private:
    typedef struct beacon {
        string typeNode;
        int idMsg;
        int idVehicle;
        string sumoId;
        double time = 0;
        Coord SenderCoord;
        double dsr = 0;
        double dsd = 0;
        double drd = 0;
        double Nv;
        double wdist = 0;
        double abe = 0;
        LAddress::L2Type NH_Address = 0;
        double NH_GS = 1;
        double NH_Dst_to_Dest = 0;
        cMessage *beaconStartMsg;
        double speed;
        double heading;
        double distanceToAnchor;
        int laneNumber;
        beacon* next;
    }* beaconPtr;

    beaconPtr head;
    beaconPtr curr;
    beaconPtr temp;

public:
    BeaconList();
    void AddBeacon(string b_typeNode, int b_idMsg, int b_idVehicle, const std::string& b_sumoId, double b_time, Coord b_SenderCoord, double b_Nv, double NH_Dst_to_Dest, double b_speed, double b_heading, double b_distance, int b_laneNum);
    void UpdateBeacon(string b_typeNode, int b_idMsg, int b_idVehicle, const std::string& b_sumoId, double b_time, Coord b_SenderCoord, double b_Nv, double NH_Dst_to_Dest, double b_speed, double b_heading, double b_distance, int b_laneNum);
    void DeleteBeacon(int b_idVehicle);
    void PrintBeacons(const std::string& ownerSumoId, string IN_OUT);
    void SortBeacons();
    void PurgeBeacons(double b_ttl);
    int CounterBeacons();
    bool SearchBeaconTreeID(int treeID);
    bool SearchBeaconNodeID(int NodeID);
    int SearchBeaconRSU();
    int SearchClstNODE(Coord DstNodeCoord);
    int TopTable();
    double TopTableNv();
    double TopTabledisToDestination();
    double topTableDtoS();
    Coord SearchBeaconCoord(int NodeID);
    double TopTable_U_dist();
    Coord TopTableNghCoord();
    bool Search_RSU_NNT();
    double TopTableGS();

    // --- NUEVAS FUNCIONES ---
    LAddress::L2Type FindBestNextHop(Coord currentNodePos, Coord destinationPos);
    LAddress::L2Type SearchBySumoId(const std::string& sumoId); // <-- AÑADIDA
};
