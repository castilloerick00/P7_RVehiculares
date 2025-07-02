//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-License: GPL-2.0-or-later
//

import veins.base.utils.SimpleAddress;

namespace veins;

packet BaseFrame1609_4 {
    int channelNumber;
    int userPriority = 7;
    int psid = 0;
    LAddress::L2Type recipientAddress = -1;
    LAddress::L2Type currentNodeAddress;
    LAddress::L2Type senderAddress = 0;
    LAddress::L2Type destinationAddress = 0;
    LAddress::L2Type NH_Address = -1;
    string nodeType = "";
    double NodeDensity = 0;
    double speed;  // Campo para la velocidad
}
