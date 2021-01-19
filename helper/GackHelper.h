
#ifndef SRC_OPENDSME_HELPER_GACKHELPER_H_
#define SRC_OPENDSME_HELPER_GACKHELPER_H_


namespace dsme {

class GackHelper {
public:
    void registerReceivedMessage(){
        //
        if(gackMap.length() != gackMapSize){
            gackMap.initialize(gackMapSize, false);
        }

        if(newSuperframe == true){
            newSuperframe = false;
            lastSeqNum = pendingMessage->getHeader().getSequenceNumber() - 1;
            gackMapIterator = 0;
            lastGTSID = 0;
            gackMap.initialize(gackMapSize, false);
        }

        if(lastGTSID < dsme.getMessageDispatcher().currentACTElement.node()->content.getGTSlotID()){
            lastGTSID = dsme.getMessageDispatcher().currentACTElement.node()->content.getGTSlotID();
            gackMapIterator = 0;
            lastSeqNum = pendingMessage->getHeader().getSequenceNumber() - 1;
        }

        if(lastSeqNum + 1 == pendingMessage->getHeader().getSequenceNumber()){
            gackMap.set(lastGTSID*(gackMapSize/SlotsCFP)+gackMapIterator, true);
            gackMapIterator++;
        } else if(lastSeqNum + 1 < pendingMessage->getHeader().getSequenceNumber()){
            for(int i = lastSeqNum + 1; i < pendingMessage->getHeader().getSequenceNumber(); i++){
                gackMap.set(lastGTSID*(gackMapSize/SlotsCFP)+gackMapIterator, false);
                gackMapIterator++;
            }
            gackMap.set(lastGTSID*(gackMapSize/SlotsCFP)+gackMapIterator, true);
        }
        lastSeqNum = pendingMessage->getHeader().getSequenceNumber();
    }

    void init(uint8_t sFOrder){
        superFrameOrder = sFOrder;
     }
    uint16_t getGackMapSize(){
        return slotsCFP * getMaxPacketsGTS();
    }
    void packetTransmitted(uint8_t GTS){
        if(transmittedPacketsGTS[GTS] < getMaxPacketsGTS()){ //gilt nur für max packet size
            transmittedPacketsGTS[GTS]++;
        }
    }
    uint8_t getMaxPacketsGTS(){
        return getIndex(superFrameOrder);
    }

    uint8_t count(){
        int sum = 0;
        for(int i = 0; i < 7; i++){
            sum += transmittedPacketsGTS[i];
        }
        return sum;
    }

    maxPacketsGTS getIndex(uint8_t superframeOrder)
    {
        if(superframeOrder <= 3 || superframeOrder >= 9) return maxPacketsGTS::undefined;

        switch(superframeOrder)
       {
           case 3:
               return maxPacketsGTS::SO3;
           case 4:
               return maxPacketsGTS::SO4;
           case 5:
               return maxPacketsGTS::SO5;
           case 6:
               return maxPacketsGTS::SO6;
           case 7:
               return maxPacketsGTS::SO7;
           case 8:
               return maxPacketsGTS::SO8;
           case 9:
               return maxPacketsGTS::SO9;
           default:
               return maxPacketsGTS::undefined;
       }
    }

    void resetTransmittedPacketsGTS(){
        for(int i = 0; i < 7; i++){ //packetSF
            transmittedPacketsGTS[i] = 0;
        }
    }

    uint8_t transmittedPacketsGTS[7] = {0};
    uint8_t superFrameOrder;
private:
    // for max packet size when GACK is used
    enum maxPacketsGTS {undefined = 0, SO3 = 1, SO4 = 3, SO5 = 6, SO6 = 12, SO7 = 26, SO8 = 52, SO9 = 105};
    uint8_t slotsCFP{7};
};
}

#endif /* SRC_OPENDSME_HELPER_GACKHELPER_H_ */

