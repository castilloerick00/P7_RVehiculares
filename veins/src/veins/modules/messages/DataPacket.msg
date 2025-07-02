// Archivo completo: veins/modules/messages/DataPacket.msg

import veins.modules.messages.BaseFrame1609_4;
import veins.base.utils.Coord;

namespace veins;

packet DataPacket extends BaseFrame1609_4 {
    string finalDestinationSumoId;
    Coord finalDestinationPosition;
    int hopLimit = 15;
}