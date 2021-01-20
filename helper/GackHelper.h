
#ifndef SRC_OPENDSME_HELPER_GACKHELPER_H_
#define SRC_OPENDSME_HELPER_GACKHELPER_H_

#include "../mac_services/dataStructures/DSMEBitVector.h"

namespace dsme {

class GackHelper {
public:
    void init(uint8_t superframesPerGackGTS, uint8_t superframeOrder, uint8_t slotsPerCFP){
        maxSuperframeID = superframesPerGackGTS-1;
        DSME_ASSERT(superframeOrder >= 3 && superframeOrder <= 9);
        gackMap.initialize(superframesPerGackGTS * slotsPerCFP * lut_maxPacketsPerGTS[superframeOrder], false);
     }

    void reset(){
        gackMap.fill(false);
    }

    void registerReceivedMessage(uint8_t msgSequenceNumber, uint8_t superframeID, uint8_t GTSlotID){
        if(superframeID > maxSuperframeID){ //new GackGTS region, reset Bitmap
            reset();
            lastSequenceNumber = msgSequenceNumber;
            lastGTSlotID = GTSlotID;
            lastSuperframeID = superframeID;
            gackMapIterator = lastSuperframeID*lastGTSlotID;
        }

        if(lastGTSlotID < GTSlotID){   // new GTS, reset gackMapIterator
            lastGTSlotID = GTSlotID;
            gackMapIterator = lastSuperframeID*lastGTSlotID;
            lastSequenceNumber = msgSequenceNumber;
        }

        for(int i = lastSequenceNumber; i < msgSequenceNumber; i++){ //fill the missing messages with Negative ACKs. Assumes, that messages come in order!
            gackMap.set(gackMapIterator++, false);
        }
        gackMap.set(gackMapIterator++, true);
        lastSequenceNumber = msgSequenceNumber;
    }

    void packetTransmitted(uint8_t GTS){
        if(transmittedPacketsGTS[GTS] < lut_maxPacketsPerGTS[superframeOrder]){ //gilt nur für max packet size
            transmittedPacketsGTS[GTS]++;
        }
    }

    uint8_t count(){
        int sum = 0;
        for(int i = 0; i < 7; i++){
            sum += transmittedPacketsGTS[i];
        }
        return sum;
    }

    void resetTransmittedPacketsGTS(){
        for(int i = 0; i < 7; i++){ //packetSF
            transmittedPacketsGTS[i] = 0;
        }
    }

    uint8_t transmittedPacketsGTS[7] = {0};
private:

    const uint8_t lut_maxPacketsPerGTS[10] = {0,0,0,1,3,6,12,26,52,105}; //calculated by formula 4.1 in thesis by Diercks
    uint8_t lastSuperframeID = 0;
    uint16_t lastSequenceNumber = 0;
    uint8_t lastGTSlotID = 0;
    uint16_t gackMapIterator;
    uint8_t maxSuperframeID;
    uint8_t superframeOrder;
    BitVector<(105*4*7)> gackMap; //superframeID * GTSlotID * MsgSlotID
    //SuperframeOrder 9, 4 superframes per GackGTS, 7 CFP Slots, TODO: replace with compile time size
};
}

#endif /* SRC_OPENDSME_HELPER_GACKHELPER_H_ */

