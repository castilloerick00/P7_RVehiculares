import veins.base.utils.Coord;
import veins.modules.messages.BaseFrame1609_4;
import veins.base.utils.SimpleAddress;

namespace veins;

packet DemoSafetyMessage extends BaseFrame1609_4 {
    Coord senderPos;
    Coord senderSpeed;
    int treeId;
    double distancia = 0;
    Coord ini_position;
    string senderSumoId;
    double senderHeading;
    int laneNumber;
    int neighborCount;
}
