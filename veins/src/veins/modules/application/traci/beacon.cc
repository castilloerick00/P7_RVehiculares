// Archivo completo: veins/modules/application/traci/beacon.cc

#include "veins/modules/application/traci/beacon.h"
#include <omnetpp.h>
#include <iomanip>
#include <cmath>

using namespace std;
using namespace veins;

BeaconList::BeaconList() {
    head = NULL;
    curr = NULL;
    temp = NULL;
}

// ... (todas las demás funciones se mantienen igual, no las repito por brevedad)
void BeaconList::AddBeacon(std::string b_typeNode, int b_idMsg, int b_idVehicle, const std::string& b_sumoId, double b_time, Coord b_SenderCoord, double b_Nv, double NH_Dst_to_Dest, double b_speed, double b_heading, double b_distance, int b_laneNum) {
    beaconPtr n = new beacon;
    n->next = NULL;
    n->typeNode = b_typeNode;
    n->idMsg = b_idMsg;
    n->idVehicle = b_idVehicle;
    n->sumoId = b_sumoId;
    n->time = b_time;
    n->SenderCoord = b_SenderCoord;
    n->Nv = b_Nv;
    n->NH_Dst_to_Dest = NH_Dst_to_Dest;
    n->speed = b_speed;
    n->heading = b_heading;
    n->distanceToAnchor = b_distance;
    n->laneNumber = b_laneNum;
    EV << "[AddBeacon] Node OMNeT ID " << b_idVehicle << " (SUMO ID: " << b_sumoId << ")" << endl;
    if (head != NULL) {
        curr = head;
        while (curr->next != NULL) { curr = curr->next; }
        curr->next = n;
    } else {
        head = n;
    }
}

void BeaconList::UpdateBeacon(std::string b_typeNode, int b_idMsg, int b_idVehicle, const std::string& b_sumoId, double b_time, Coord b_SenderCoord, double b_Nv, double NH_Dst_to_Dest, double b_speed, double b_heading, double b_distance, int b_laneNum) {
    beaconPtr n = head;
    while (n != NULL) {
        if (n->idVehicle == b_idVehicle) {
            n->typeNode = b_typeNode;
            n->idMsg = b_idMsg;
            n->sumoId = b_sumoId;
            n->time = b_time;
            n->SenderCoord = b_SenderCoord;
            n->Nv = b_Nv;
            n->NH_Dst_to_Dest = NH_Dst_to_Dest;
            n->speed = b_speed;
            n->heading = b_heading;
            n->distanceToAnchor = b_distance;
            n->laneNumber = b_laneNum;
            EV << "[UpdateBeacon] Node OMNeT ID " << b_idVehicle << " (SUMO ID: " << b_sumoId << ")" << endl;
            return;
        }
        n = n->next;
    }
}

void BeaconList::PrintBeacons(const std::string& ownerSumoId, string IN_OUT) {
    curr = head;
    EV << "------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------" << endl;
    EV << setw(35) << IN_OUT << "  -->  Vehicle Neighbor Table -> Owner SUMO ID =" << ownerSumoId << " (Time: " << simTime() << "s)" << endl;
    EV << "------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------" << endl;
    EV << "NodeType" << setw(10) << "SUMO_ID" << setw(10) << "OMNeT_ID" << setw(10) << "TreeMsg" << setw(12) << "NghCount" << setw(14) << "ArrivalTime" << setw(14) << "Speed" << setw(16) << "HdgToAnc(deg)" << setw(18) << "DistToAnchor" << setw(12) << "LaneNum" << endl;
    while (curr != NULL) {
        EV << setw(5) << curr->typeNode
           << setw(10) << curr->sumoId
           << setw(10) << curr->idVehicle
           << setw(10) << curr->idMsg
           << setw(12) << curr->Nv
           << setw(14) << std::fixed << std::setprecision(3) << curr->time
           << setw(14) << std::fixed << std::setprecision(2) << curr->speed
           << setw(16) << std::fixed << std::setprecision(2) << curr->heading
           << setw(18) << std::fixed << std::setprecision(2) << curr->distanceToAnchor
           << setw(12) << curr->laneNumber << endl;
        curr = curr->next;
    }
    EV << "------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------" << endl;
}

void BeaconList::DeleteBeacon(int delb_idVehicle) {
    beaconPtr delPtr = NULL;
    temp = head;
    curr = head;
    while (curr != NULL && curr->idVehicle != delb_idVehicle) {
        temp = curr;
        curr = curr->next;
    }
    if (curr == NULL) {
        return;
    }
    else {
        delPtr = curr;
        if (delPtr == head) {
            head = head->next;
        }
        else {
            temp->next = curr->next;
        }
        delete delPtr;
    }
}

void BeaconList::PurgeBeacons(double b_ttl) {
    if (head == NULL) return;
    double TimeExpired = simTime().dbl() - b_ttl;
    while (head != NULL && head->time < TimeExpired) {
        temp = head;
        head = head->next;
        delete temp;
    }
    curr = head;
    if (curr == NULL) return;
    while (curr->next != NULL) {
        if (curr->next->time < TimeExpired) {
            temp = curr->next;
            curr->next = temp->next;
            delete temp;
        }
        else {
            curr = curr->next;
        }
    }
}
int BeaconList::TopTable() { /*...*/ return 0; }
double BeaconList::TopTableNv() { /*...*/ return 0; }
Coord BeaconList::TopTableNghCoord() { /*...*/ return Coord(); }
double BeaconList::TopTable_U_dist() { /*...*/ return 0; }
double BeaconList::TopTabledisToDestination() { /*...*/ return 0; }
double BeaconList::topTableDtoS() { /*...*/ return 0; }
int BeaconList::CounterBeacons() {
    curr = head;
    int counter = 0;
    while (curr != NULL) {
        curr = curr->next;
        counter++;
    }
    return counter;
}
bool BeaconList::SearchBeaconNodeID(int NodeID) {
    curr = head;
    while(curr) {
        if(curr->idVehicle == NodeID) return true;
        curr = curr->next;
    }
    return false;
}
bool BeaconList::SearchBeaconTreeID(int treeID) { /*...*/ return false; }
Coord BeaconList::SearchBeaconCoord(int NodeID) { /*...*/ return Coord(); }
int BeaconList::SearchClstNODE(Coord DstNodeCoord) { /*...*/ return 0; }
int BeaconList::SearchBeaconRSU() { /*...*/ return 0; }
bool BeaconList::Search_RSU_NNT() { /*...*/ return false; }
void BeaconList::SortBeacons() { /*...*/ }
double BeaconList::TopTableGS() { /*...*/ return 0; }

// --- NUEVA FUNCIÓN AÑADIDA ---
LAddress::L2Type BeaconList::SearchBySumoId(const std::string& sumoId) {
    beaconPtr p = head;
    while (p != nullptr) {
        if (p->sumoId == sumoId) {
            return p->idVehicle; // Devuelve el OMNeT ID si lo encuentra
        }
        p = p->next;
    }
    return LAddress::L2BROADCAST(); // Devuelve broadcast (-1) si no lo encuentra
}


LAddress::L2Type BeaconList::FindBestNextHop(Coord currentNodePos, Coord destinationPos) {
    PurgeBeacons(3);
    if (head == nullptr) {
        EV_WARN << "[BeaconList] Cannot select next hop, neighbor table is empty." << endl;
        return LAddress::L2BROADCAST();
    }

    double maxDistanceToAnchor = 0;
    double maxNeighborCount = 0;
    beaconPtr p = head;
    while (p != nullptr) {
        if (p->distanceToAnchor > maxDistanceToAnchor) {
            maxDistanceToAnchor = p->distanceToAnchor;
        }
        if (p->Nv > maxNeighborCount) {
            maxNeighborCount = p->Nv;
        }
        p = p->next;
    }

    if (maxDistanceToAnchor == 0) maxDistanceToAnchor = 1;
    if (maxNeighborCount == 0) maxNeighborCount = 1;

    double maxPeso = -1.0;
    LAddress::L2Type bestNextHop = LAddress::L2BROADCAST();
    std::string bestNextHopSumoId = "N/A";

    p = head;
    EV_INFO << "--- [BeaconList] Next Hop Selection ---" << endl;
    while (p != nullptr) {
        double dn = p->distanceToAnchor / maxDistanceToAnchor;

        double heading_deg = p->heading;
        double heading_rad = heading_deg * M_PI / 180.0;
        double cos_val = cos(heading_rad);
        double an = (cos_val + 1.0) / 2.0;

        double cn = static_cast<double>(p->laneNumber) / 4.0;
        if (cn < 0.25) cn = 0.25;
        if (cn > 1.0) cn = 1.0;

        double vn = p->Nv / maxNeighborCount;

        double peso = (1.0 - dn) * 0.2 + an * 0.5 + cn * 0.2 + vn * 0.1;
        EV_INFO << "  - Candidate: " << p->sumoId << " | dn(Anchor):" << dn << " an:" << an << " cn:" << cn << " vn:" << vn << " -> peso: " << peso << endl;

        if (peso > maxPeso) {
            maxPeso = peso;
            bestNextHop = p->idVehicle;
            bestNextHopSumoId = p->sumoId;
        }
        p = p->next;
    }
    EV_INFO << "--> [BeaconList] Best next hop selected: " << bestNextHopSumoId << " (OMNeT ID: " << bestNextHop << ") with weight " << maxPeso << endl;

    return bestNextHop;
}
