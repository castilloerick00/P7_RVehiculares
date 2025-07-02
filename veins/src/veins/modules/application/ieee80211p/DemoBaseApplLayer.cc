// Archivo completo: veins/modules/application/ieee80211p/DemoBaseApplLayer.cc

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include <cmath>
#include <string>
#include <stdexcept>

using namespace veins;

Coord DemoBaseApplLayer::anchorPointPosition;
bool DemoBaseApplLayer::anchorPointCaptured = false;

DemoBaseApplLayer::~DemoBaseApplLayer() {
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    findHost()->unsubscribe(BaseMobility::mobilityStateChangedSignal, this);
    findHost()->unsubscribe(TraCIMobility::parkingStateChangedSignal, this);
}

void DemoBaseApplLayer::initialize(int stage) {
    BaseApplLayer::initialize(stage);
    if (stage == 0) {
        if (FindModule<TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
        }
        else {
            traci = nullptr;
            mobility = nullptr;
            traciVehicle = nullptr;
        }
        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);
        mac = FindModule<DemoBaseApplLayerToMac1609_4Interface*>::findSubModule(getParentModule());
        ASSERT(mac);
        headerLength = par("headerLength");
        sendBeacons = par("sendBeacons").boolValue();
        beaconLengthBits = par("beaconLengthBits");
        beaconUserPriority = par("beaconUserPriority");
        beaconInterval = par("beaconInterval");
        dataLengthBits = par("dataLengthBits");
        dataOnSch = par("dataOnSch").boolValue();
        dataUserPriority = par("dataUserPriority");
        wsaInterval = par("wsaInterval").doubleValue();
        currentOfferedServiceId = -1;
        isParked = false;
        pos_updated = true;
        findHost()->subscribe(BaseMobility::mobilityStateChangedSignal, this);
        findHost()->subscribe(TraCIMobility::parkingStateChangedSignal, this);
        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt = new cMessage("wsa evt", SEND_WSA_EVT);
        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        receivedBSMs = 0;
        receivedWSAs = 0;
        receivedWSMs = 0;
        nodeType = getParentModule()->getName();
    }
    else if (stage == 1) {
        myId = mac->getMACAddress();
        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
        }
        simtime_t firstBeacon = simTime();
        if (par("avoidBeaconSynchronization").boolValue() == true) {
            simtime_t randomOffset = dblrand() * beaconInterval;
            firstBeacon = simTime() + randomOffset;
            if (mac->isChannelSwitchingActive() == true) {
                if (beaconInterval.raw() % (mac->getSwitchingInterval().raw() * 2)) {
                }
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, ChannelType::control);
            }
            if (sendBeacons) {
                scheduleAt(firstBeacon, sendBeaconEvt);
            }
        }
    }
}

void DemoBaseApplLayer::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial) {
    wsm->setRecipientAddress(rcvId);
    wsm->setBitLength(headerLength);
    wsm->setCurrentNodeAddress(myId);
    wsm->setNodeType(nodeType.c_str());
    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        double speedValue = traciVehicle ? traciVehicle->getSpeed() : 0.0;
        bsm->setSpeed(speedValue);
        bsm->setTreeId(myId + generatedBSMs);
        bsm->setPsid(-1);
        bsm->setChannelNumber(static_cast<int>(Channel::cch));
        bsm->addBitLength(beaconLengthBits);
        bsm->setIni_position(ini_position);
        wsm->setUserPriority(beaconUserPriority);
        if (mobility) {
            bsm->setSenderSumoId(mobility->getExternalId().c_str());
        }
        if (traciVehicle) {
            double headingDeg = traciVehicle->getAngle();
            bsm->setSenderHeading(headingDeg);
            std::string laneIdStr = traciVehicle->getLaneId();
            int laneNumber = 0;
            size_t pos = laneIdStr.find_last_of('_');
            if (pos != std::string::npos) {
                try {
                    laneNumber = std::stoi(laneIdStr.substr(pos + 1)) + 1;
                }
                catch (const std::exception& e) {
                    EV_WARN << "Could not parse lane index from lane ID: " << laneIdStr << endl;
                    laneNumber = 0;
                }
            }
            bsm->setLaneNumber(laneNumber);
        }
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(static_cast<int>(Channel::cch));
        wsa->setTargetChannel(static_cast<int>(currentServiceChannel));
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
    }
    else {
        if (dataOnSch) wsm->setChannelNumber(static_cast<int>(Channel::sch1));
        else wsm->setChannelNumber(static_cast<int>(Channel::cch));
        wsm->addBitLength(dataLengthBits);
        wsm->setUserPriority(dataUserPriority);
    }
}

void DemoBaseApplLayer::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) {
    Enter_Method_Silent();
    if (signalID == BaseMobility::mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == TraCIMobility::parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void DemoBaseApplLayer::handlePositionUpdate(cObject* obj) {
    ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mobility->getPositionAt(simTime());
    curSpeed = mobility->getCurrentSpeed();
    if (traciVehicle && curSpeed.length() == 0) {
        curSpeed = Coord(traciVehicle->getSpeed(), 0, 0);
    }
    if (pos_updated && myId == 10) {
        pos_updated = false;
        ini_position = curPosition;
    }
}

void DemoBaseApplLayer::handleParkingUpdate(cObject* obj) {
    isParked = mobility->getParkingState();
}

void DemoBaseApplLayer::handleLowerMsg(cMessage* msg) {
    BaseFrame1609_4* wsm = dynamic_cast<BaseFrame1609_4*>(msg);
    ASSERT(wsm);
    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        receivedBSMs++;
        onBSM(bsm);
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        receivedWSAs++;
        onWSA(wsa);
    }
    else {
        receivedWSMs++;
        onWSM(wsm);
    }
    delete (msg);
}

void DemoBaseApplLayer::handleSelfMsg(cMessage* msg) {
    switch (msg->getKind()) {
    case SEND_BEACON_EVT: {
        DemoSafetyMessage* bsm = new DemoSafetyMessage();
        populateWSM(bsm);
        sendDown(bsm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        break;
    }
    case SEND_WSA_EVT: {
        DemoServiceAdvertisment* wsa = new DemoServiceAdvertisment();
        populateWSM(wsa);
        sendDown(wsa);
        scheduleAt(simTime() + wsaInterval, sendWSAEvt);
        break;
    }
    default: {
        if (msg) {}
        break;
    }
    }
}

simtime_t DemoBaseApplLayer::computeAsynchronousSendingTime(simtime_t interval, ChannelType chan) {
    simtime_t randomOffset = dblrand() * interval;
    simtime_t firstEvent;
    simtime_t switchingInterval = mac->getSwitchingInterval();
    simtime_t nextCCH;
    if (mac->isCurrentChannelCCH()) {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval * 2;
    }
    else {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval;
    }
    firstEvent = nextCCH + randomOffset;
    if (firstEvent.raw() % (2 * switchingInterval.raw()) > switchingInterval.raw()) {
        if (chan == ChannelType::control) firstEvent -= switchingInterval;
    }
    else {
        if (chan == ChannelType::service) firstEvent += switchingInterval;
    }
    return firstEvent;
}

void DemoBaseApplLayer::finish() {
    recordScalar("generatedWSMs", generatedWSMs);
    recordScalar("receivedWSMs", receivedWSMs);
    recordScalar("generatedBSMs", generatedBSMs);
    recordScalar("receivedBSMs", receivedBSMs);
    recordScalar("generatedWSAs", generatedWSAs);
    recordScalar("receivedWSAs", receivedWSAs);
}

void DemoBaseApplLayer::startService(Channel channel, int serviceId, std::string serviceDescription) {
    if (sendWSAEvt->isScheduled()) {
        throw cRuntimeError("Starting service although another service was already started");
    }
    mac->changeServiceChannel(channel);
    currentOfferedServiceId = serviceId;
    currentServiceChannel = channel;
    currentServiceDescription = serviceDescription;
    simtime_t wsaTime = computeAsynchronousSendingTime(wsaInterval, ChannelType::control);
    scheduleAt(wsaTime, sendWSAEvt);
}

void DemoBaseApplLayer::stopService() {
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

void DemoBaseApplLayer::sendDown(cMessage* msg) {
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void DemoBaseApplLayer::sendDelayedDown(cMessage* msg, simtime_t delay) {
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

void DemoBaseApplLayer::checkAndTrackPacket(cMessage* msg) {
    if (dynamic_cast<DemoSafetyMessage*>(msg)) {
        generatedBSMs++;
    }
    else if (dynamic_cast<DemoServiceAdvertisment*>(msg)) {
        generatedWSAs++;
    }
    else if (dynamic_cast<BaseFrame1609_4*>(msg)) {
        generatedWSMs++;
    }
}
