// Archivo completo: veins/modules/application/ieee80211p/DemoBaseApplLayer.h

#pragma once

#include <map>

#include "veins/base/modules/BaseApplLayer.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/modules/messages/DemoServiceAdvertisement_m.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include "veins/modules/mac/ieee80211p/DemoBaseApplLayerToMac1609_4Interface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
// Se elimina la inclusión de beacon.h de la clase base

namespace veins {

using veins::AnnotationManager;
using veins::AnnotationManagerAccess;
using veins::TraCICommandInterface;
using veins::TraCIMobility;
using veins::TraCIMobilityAccess;

class VEINS_API DemoBaseApplLayer : public BaseApplLayer {
public:
    ~DemoBaseApplLayer() override;
    void initialize(int stage) override;
    void finish() override;
    void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) override;

    enum DemoApplMessageKinds {
        SEND_BEACON_EVT,
        SEND_WSA_EVT
    };

protected:
    // Las variables estáticas para el anclaje son compartidas por todas las instancias
    static Coord anchorPointPosition;
    static bool anchorPointCaptured;

    void handleLowerMsg(cMessage* msg) override;
    void handleSelfMsg(cMessage* msg) override;
    virtual void populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId = LAddress::L2BROADCAST(), int serial = 0);
    virtual void onWSM(BaseFrame1609_4* wsm) {};
    virtual void onBSM(DemoSafetyMessage* bsm) {};
    virtual void onWSA(DemoServiceAdvertisment* wsa) {};
    virtual void handlePositionUpdate(cObject* obj);
    virtual void handleParkingUpdate(cObject* obj);
    virtual void startService(Channel channel, int serviceId, std::string serviceDescription);
    virtual void stopService();
    virtual simtime_t computeAsynchronousSendingTime(simtime_t interval, ChannelType chantype);
    virtual void sendDown(cMessage* msg);
    virtual void sendDelayedDown(cMessage* msg, simtime_t delay);
    virtual void checkAndTrackPacket(cMessage* msg);

protected:
    TraCIMobility* mobility;
    TraCICommandInterface* traci;
    TraCICommandInterface::Vehicle* traciVehicle;
    AnnotationManager* annotations;
    DemoBaseApplLayerToMac1609_4Interface* mac;
    bool isParked;
    bool pos_updated;
    uint32_t beaconLengthBits;
    uint32_t beaconUserPriority;
    simtime_t beaconInterval;
    bool sendBeacons;
    uint32_t dataLengthBits;
    uint32_t dataUserPriority;
    bool dataOnSch;
    int currentOfferedServiceId;
    std::string currentServiceDescription;
    Channel currentServiceChannel;
    simtime_t wsaInterval;
    Coord ini_position;
    Coord curPosition;
    Coord curSpeed;
    LAddress::L2Type myId = 0;
    int mySCH;
    uint32_t generatedWSMs;
    uint32_t generatedWSAs;
    uint32_t generatedBSMs;
    uint32_t receivedWSMs;
    uint32_t receivedWSAs;
    uint32_t receivedBSMs;
    std::string nodeType;
    cMessage* sendBeaconEvt;
    cMessage* sendWSAEvt;
};

} // namespace veins
