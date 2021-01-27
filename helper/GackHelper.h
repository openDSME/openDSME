
#ifndef SRC_OPENDSME_HELPER_GACKHELPER_H_
#define SRC_OPENDSME_HELPER_GACKHELPER_H_

#include "../mac_services/dataStructures/DSMEBitVector.h"

namespace dsme {

class GackHelper {
public:
    void init(uint8_t superframesPerGackGTS, uint8_t superframeOrder, uint8_t slotsPerCFP){
        this->maxSuperframeID = superframesPerGackGTS-1;
        this->slotsPerCFP = slotsPerCFP;
        this->superframeOrder = superframeOrder;
        DSME_ASSERT(superframeOrder >= 3 && superframeOrder <= 9);
        gackVector.initialize(superframesPerGackGTS * slotsPerCFP * lut_maxPacketsPerGTS[superframeOrder], false);
     }

    void reset(){
        gackVector.fill(false);
        lastSequenceNumber = 0;
        lastGTSlotID = 0;
        lastSuperframeID = 0;
        gackMapIterator = 0;
        currentSlotPacketID = 0;
    }

    void handleNewSuperframe(uint8_t superframeID, uint8_t multiSuperframeID){
        ASSERT(superframeID <= maxSuperframeID);
        if(superframeID % (maxSuperframeID+1) == 0){ //reset once nrSuperframesPerGackGTS is reached
            reset();
        }
    }

    void registerReceivedMessage(uint8_t msgSequenceNumber, uint8_t superframeID, uint8_t GTSlotID){
        if(superframeID > maxSuperframeID){ //new GackGTS region, reset Bitmap
            reset();
            lastSequenceNumber = msgSequenceNumber;
            lastGTSlotID = GTSlotID;
            lastSuperframeID = superframeID;
            gackMapIterator = lastSuperframeID*slotsPerCFP*lut_maxPacketsPerGTS[superframeOrder] + lastGTSlotID*lut_maxPacketsPerGTS[superframeOrder];
            currentSlotPacketID = 0;
        }

        if(lastGTSlotID < GTSlotID){   // new GTS, reset gackMapIterator
            lastGTSlotID = GTSlotID;
            gackMapIterator = lastSuperframeID*slotsPerCFP*lut_maxPacketsPerGTS[superframeOrder] + lastGTSlotID*lut_maxPacketsPerGTS[superframeOrder];
            lastSequenceNumber = msgSequenceNumber;
            currentSlotPacketID = 0;
        }

        for(uint16_t i = lastSequenceNumber+1; i < msgSequenceNumber; i++){ //fill the missing messages with Negative ACKs. Assumes, that messages come in order!
            gackVector.set(gackMapIterator++, false);
            currentSlotPacketID++;
        }
        gackVector.set(gackMapIterator++, true);
        currentSlotPacketID++;
        ASSERT(currentSlotPacketID < lut_maxPacketsPerGTS[superframeOrder]);
        lastSequenceNumber = msgSequenceNumber;
    }
    BitVector<(2*7*26)>& getGackVector(){
        return gackVector;
    }
private:

    const uint8_t lut_maxPacketsPerGTS[10] = {0,0,0,1,3,6,12,26,52,105}; //calculated by formula 4.1 in thesis by Diercks
    uint8_t lastSuperframeID = 0;
    uint16_t lastSequenceNumber = 0;
    uint8_t lastGTSlotID = 0;
    uint16_t gackMapIterator = 0;
    uint8_t maxSuperframeID;
    uint8_t superframeOrder;
    uint8_t slotsPerCFP;
    uint8_t currentSlotPacketID = 0;
    BitVector<(2*7*26)> gackVector; //superframeID * GTSlotID * MsgSlotID
    //SuperframeOrder 9, 4 superframes per GackGTS, 7 CFP Slots, TODO: replace with compile time size
};
}

#endif /* SRC_OPENDSME_HELPER_GACKHELPER_H_ */

