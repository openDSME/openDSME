
#ifndef SRC_OPENDSME_HELPER_GACKHELPER_H_
#define SRC_OPENDSME_HELPER_GACKHELPER_H_

#include "../mac_services/dataStructures/DSMEBitVector.h"

namespace dsme {

class GackHelper {
public:
    void init(uint8_t superframesPerGackGTS, uint8_t superframeOrder, uint8_t slotsPerCFP){
        maxSuperframeID = superframesPerGackGTS-1;
        this->slotsPerCFP = slotsPerCFP;
        DSME_ASSERT(superframeOrder >= 3 && superframeOrder <= 9);
        gackVector.initialize(superframesPerGackGTS * slotsPerCFP * lut_maxPacketsPerGTS[superframeOrder], false);
     }

    void reset(){
        gackVector.fill(false);
    }

    void registerReceivedMessage(uint8_t msgSequenceNumber, uint8_t superframeID, uint8_t GTSlotID){
        if(superframeID > maxSuperframeID){ //new GackGTS region, reset Bitmap
            reset();
            lastSequenceNumber = msgSequenceNumber;
            lastGTSlotID = GTSlotID;
            lastSuperframeID = superframeID;
            gackMapIterator = lastSuperframeID*slotsPerCFP*lut_maxPacketsPerGTS[superframeOrder] + lastGTSlotID*lut_maxPacketsPerGTS[superframeOrder];
        }

        if(lastGTSlotID < GTSlotID){   // new GTS, reset gackMapIterator
            lastGTSlotID = GTSlotID;
            gackMapIterator = lastSuperframeID*slotsPerCFP*lut_maxPacketsPerGTS[superframeOrder] + lastGTSlotID*lut_maxPacketsPerGTS[superframeOrder];
            lastSequenceNumber = msgSequenceNumber;
        }

        for(int i = lastSequenceNumber; i < msgSequenceNumber; i++){ //fill the missing messages with Negative ACKs. Assumes, that messages come in order!
            gackVector.set(gackMapIterator++, false);
        }
        gackVector.set(gackMapIterator++, true);
        lastSequenceNumber = msgSequenceNumber;
    }
    BitVector<(2*7*12)>& getGackVector(){
        return gackVector;
    }
private:

    const uint8_t lut_maxPacketsPerGTS[10] = {0,0,0,1,3,6,12,26,52,105}; //calculated by formula 4.1 in thesis by Diercks
    uint8_t lastSuperframeID = 0;
    uint16_t lastSequenceNumber = 0;
    uint8_t lastGTSlotID = 0;
    uint16_t gackMapIterator;
    uint8_t maxSuperframeID;
    uint8_t superframeOrder;
    uint8_t slotsPerCFP;
    BitVector<(2*7*12)> gackVector; //superframeID * GTSlotID * MsgSlotID
    //SuperframeOrder 9, 4 superframes per GackGTS, 7 CFP Slots, TODO: replace with compile time size
};
}

#endif /* SRC_OPENDSME_HELPER_GACKHELPER_H_ */

