// Archivo completo: veins/modules/application/traci/TraCIDemo11p.cc

#include "veins/modules/application/traci/TraCIDemo11p.h"
#include "veins/modules/application/traci/TraCIDemo11pMessage_m.h"
#include "veins/modules/messages/DataPacket_m.h"
#include <cmath>
#include <string>
#include <stdexcept>

using namespace veins;

Define_Module(veins::TraCIDemo11p);

void TraCIDemo11p::initialize(int stage) {
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
        forwardMsg = false;
        NH_Dst_to_Dest = 0;
        dataMessageSent = false;
    }
}

void TraCIDemo11p::onWSA(DemoServiceAdvertisment* wsa) {
    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(static_cast<Channel>(wsa->getTargetChannel()));
        currentSubscribedServiceId = wsa->getPsid();
        if (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService(static_cast<Channel>(wsa->getTargetChannel()), wsa->getPsid(), "Mirrored Traffic Service");
        }
    }
}

void TraCIDemo11p::onBSM(DemoSafetyMessage* bsm) {
    if (simTime() >= 150.0) {
        UpdateNNT(bsm);
    }
}

void TraCIDemo11p::onWSM(BaseFrame1609_4* wsm) {
    if (DataPacket* dp = dynamic_cast<DataPacket*>(wsm)) {
        handleDataPacket(dp);
    }
    else if (TraCIDemo11pMessage* t_wsm = check_and_cast<TraCIDemo11pMessage*>(wsm)) {
        findHost()->getDisplayString().setTagArg("i", 1, "green");
        if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(t_wsm->getDemoData(), 9999);
        if (!sentMessage) {
            sentMessage = true;
            t_wsm->setSerial(3);
            scheduleAt(simTime() + 2 + uniform(0.01, 0.2), t_wsm->dup());
        }
    }
}

void TraCIDemo11p::handleSelfMsg(cMessage* msg) {
    if (DataPacket* dp = dynamic_cast<DataPacket*>(msg)) {
        forwardDataPacket(dp);
        return;
    }

    if (msg->getKind() == DemoBaseApplLayer::SEND_BEACON_EVT) {
        DemoSafetyMessage* bsm = new DemoSafetyMessage();
        populateWSM(bsm);
        bsm->setNeighborCount(ListBeacon.CounterBeacons());
        sendDown(bsm);
        scheduleAt(simTime() + beaconInterval, msg);
    }
    else if (TraCIDemo11pMessage* wsm = dynamic_cast<TraCIDemo11pMessage*>(msg)) {
        sendDown(wsm->dup());
        wsm->setSerial(wsm->getSerial() + 1);
        if (wsm->getSerial() >= 3) {
            stopService();
            delete (wsm);
        } else {
            scheduleAt(simTime() + 1, wsm);
        }
    } else {
        DemoBaseApplLayer::handleSelfMsg(msg);
    }
}

void TraCIDemo11p::handlePositionUpdate(cObject* obj) {
    DemoBaseApplLayer::handlePositionUpdate(obj);
    if (!anchorPointCaptured && simTime() >= 145.0) {
        if (mobility && mobility->getExternalId() == "2") {
            anchorPointPosition = curPosition;
            anchorPointCaptured = true;
            EV_WARN << "****** ANCHOR POINT SET ******" << endl;
            EV_WARN << "Node SUMO ID: " << mobility->getExternalId() << " (OMNeT ID: " << myId << ") captured its position at t=" << simTime() << "s." << endl;
            EV_WARN << "Coordinate: (" << anchorPointPosition.x << ", " << anchorPointPosition.y << ", " << anchorPointPosition.z << ")" << endl;
            EV_WARN << "******************************" << endl;
        }
    }

    if (!dataMessageSent && mobility && mobility->getExternalId() == "14" && simTime() >= 151.0) {
        if (!anchorPointCaptured) {
            EV_WARN << "Node 14 wants to send a message, but anchor point (destination pos) is not captured yet!" << endl;
            return;
        }

        EV_INFO << ">>> Node 14 (Source) is initiating a routed message to Node 2 at t=" << simTime() << "s. <<<" << endl;

        DataPacket* dp = new DataPacket();
        dp->setFinalDestinationSumoId("2");
        dp->setFinalDestinationPosition(anchorPointPosition);
        dp->setHopLimit(15);

        forwardDataPacket(dp);

        dataMessageSent = true;
    }

    if (mobility->getSpeed() < 1) {
        if (simTime() - lastDroveAt >= 10 && sentMessage == false) {
            findHost()->getDisplayString().setTagArg("i", 1, "red");
            sentMessage = true;
            TraCIDemo11pMessage* wsm = new TraCIDemo11pMessage();
            populateWSM(wsm);
            wsm->setDemoData(mobility->getRoadId().c_str());
            if (dataOnSch) {
                startService(Channel::sch2, 42, "Traffic Information Service");
                scheduleAt(computeAsynchronousSendingTime(1, ChannelType::service), wsm);
            } else {
                sendDown(wsm);
            }
        }
    } else {
        lastDroveAt = simTime();
    }
}

void TraCIDemo11p::PreparaBeaconValues(DemoSafetyMessage* bsm) {
    Table_Ngh_NodeId = bsm->getCurrentNodeAddress();
    Table_Ngh_SumoId = bsm->getSenderSumoId();
    Table_Ngh_NodeType = bsm->getNodeType();
    Table_Ngh_Coord = bsm->getSenderPos();
    Table_Beacon_ArrivalTime = bsm->getArrivalTime().dbl();
    Table_Ngh_Msg_TreeID = bsm->getTreeId();
    Table_Ngh_Heading = bsm->getSenderHeading();
    Table_Ngh_LaneNumber = bsm->getLaneNumber();
    Table_Ngh_NeighborCount = bsm->getNeighborCount();
}

void TraCIDemo11p::UpdateNNT(DemoSafetyMessage* bsm) {
    PreparaBeaconValues(bsm);

    double transformedHeading = -999;
    if (anchorPointCaptured) {
        double dx_ref = anchorPointPosition.x - Table_Ngh_Coord.x;
        double dy_ref = anchorPointPosition.y - Table_Ngh_Coord.y;
        double referenceAngleRad = atan2(dy_ref, dx_ref);
        double referenceAngleDeg = referenceAngleRad * 180. / M_PI;
        transformedHeading = Table_Ngh_Heading - referenceAngleDeg - 90;
        while (transformedHeading <= -180) transformedHeading += 360;
        while (transformedHeading > 180) transformedHeading -= 360;
    }

    double distanceToFixedPoint = -1.0;
    if (anchorPointCaptured) {
        distanceToFixedPoint = Table_Ngh_Coord.distance(anchorPointPosition);
    }

    double b_speed = bsm->getSpeed();

    if (ListBeacon.SearchBeaconNodeID(Table_Ngh_NodeId)) {
        ListBeacon.UpdateBeacon(Table_Ngh_NodeType, Table_Ngh_Msg_TreeID, Table_Ngh_NodeId, Table_Ngh_SumoId, Table_Beacon_ArrivalTime, Table_Ngh_Coord, Table_Ngh_NeighborCount, NH_Dst_to_Dest, b_speed, transformedHeading, distanceToFixedPoint, Table_Ngh_LaneNumber);
    }
    else {
        ListBeacon.AddBeacon(Table_Ngh_NodeType, Table_Ngh_Msg_TreeID, Table_Ngh_NodeId, Table_Ngh_SumoId, Table_Beacon_ArrivalTime, Table_Ngh_Coord, Table_Ngh_NeighborCount, NH_Dst_to_Dest, b_speed, transformedHeading, distanceToFixedPoint, Table_Ngh_LaneNumber);
    }

    ListBeacon.PurgeBeacons(3);
    if (mobility) {
        ListBeacon.PrintBeacons(mobility->getExternalId(), "IN");
    }
}

void TraCIDemo11p::handleDataPacket(DataPacket* dp) {
    EV_INFO << "Node " << mobility->getExternalId() << " received a routed DataPacket." << endl;
    findHost()->getDisplayString().setTagArg("i", 1, "red");

    if (mobility->getExternalId() == dp->getFinalDestinationSumoId()) {
        EV_WARN << "!!!!!!!!!! DESTINATION " << dp->getFinalDestinationSumoId() << " REACHED !!!!!!!!!! " << endl;
        findHost()->getDisplayString().setTagArg("i", 1, "gold");
        return;
    }

    if (dp->getHopLimit() <= 0) {
        EV_WARN << "Hop limit reached. Dropping DataPacket at node " << mobility->getExternalId() << "." << endl;
        return;
    }

    dp->setHopLimit(dp->getHopLimit() - 1);

    scheduleAt(simTime() + uniform(0.01, 0.05), dp->dup());
}

void TraCIDemo11p::forwardDataPacket(DataPacket* dp) {
    findHost()->getDisplayString().setTagArg("i", 1, "green");

    LAddress::L2Type nextHop = LAddress::L2BROADCAST();
    std::string finalDestSumoId = dp->getFinalDestinationSumoId();

    // --- NUEVA LÓGICA DE PRIORIDAD ---
    // 1. Primero, buscar si el destino final ya es un vecino directo.
    nextHop = ListBeacon.SearchBySumoId(finalDestSumoId);

    // 2. Si NO es un vecino directo (la búsqueda no lo encontró), entonces ejecutar el algoritmo de pesos.
    if (nextHop == LAddress::L2BROADCAST()) {
        EV_INFO << "Destination " << finalDestSumoId << " not in neighbor table. Calculating best forwarder..." << endl;
        nextHop = ListBeacon.FindBestNextHop(curPosition, dp->getFinalDestinationPosition());
    }
    // 3. Si SÍ es un vecino directo, el 'nextHop' ya tiene el valor correcto y se salta el cálculo de pesos.
    else {
        EV_WARN << ">>> Destination " << finalDestSumoId << " is a direct neighbor! Sending directly. <<<" << endl;
    }

    // --- Lógica de envío (sin cambios) ---
    if (nextHop != LAddress::L2BROADCAST()) {
        EV_INFO << "Node " << mobility->getExternalId() << " is forwarding DataPacket to OMNeT ID " << nextHop << endl;

        populateWSM(dp);
        dp->setRecipientAddress(nextHop);

        sendDown(dp);
    }
    else {
        EV_WARN << "Node " << mobility->getExternalId() << ": No suitable next hop found. Dropping DataPacket." << endl;
        delete dp;
    }
}
