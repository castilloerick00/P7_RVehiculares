// Archivo completo: veins/modules/application/traci/TraCIDemo11p.h

#pragma once

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/application/traci/beacon.h"
#include "veins/modules/messages/DataPacket_m.h"

namespace veins {

class VEINS_API TraCIDemo11p : public DemoBaseApplLayer {
public:
    void initialize(int stage) override;
    ~TraCIDemo11p() override = default;

protected:
    simtime_t lastDroveAt;
    bool sentMessage;
    bool forwardMsg;
    int currentSubscribedServiceId;
    bool dataMessageSent;

    BeaconList ListBeacon;
    LAddress::L2Type Table_Ngh_NodeId;
    std::string Table_Ngh_SumoId;
    std::string Table_Ngh_NodeType;
    int Table_Ngh_Msg_TreeID;
    Coord Table_Ngh_Coord;
    double Table_Beacon_ArrivalTime;
    double Table_Ngh_Heading;
    int Table_Ngh_LaneNumber;
    double Table_Ngh_NeighborCount;
    double NH_Dst_to_Dest;

protected:
    void onWSM(BaseFrame1609_4* wsm) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;
    void onBSM(DemoSafetyMessage* bsm) override;
    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
    void UpdateNNT(DemoSafetyMessage* bsm);
    void PreparaBeaconValues(DemoSafetyMessage* bsm);

    void handleDataPacket(DataPacket* dp);
    void forwardDataPacket(DataPacket* dp);
};

} // namespace veins
