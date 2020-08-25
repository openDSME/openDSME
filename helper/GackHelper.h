/*
 * GackHelper.h
 *
 *  Created on: 20.08.2020
 *      Author: cjd8627
 */

#ifndef SRC_OPENDSME_HELPER_GACKHELPER_H_
#define SRC_OPENDSME_HELPER_GACKHELPER_H_

#include "Integers.h"

namespace dsme {

    class GackHelper {
    private:
        enum maxPacketsGTS {undefined = 0, SO3 = 1, SO4 = 2, SO5 = 5, SO6 = 11, SO7 = 23, SO8 = 46, SO9 = 93};
        uint8_t slotsCFP{7};
    public:
        uint8_t transmittedPacketsGTS[7] = {0};

        uint8_t superFrameOrder;

        void init(uint8_t sFOrder){
            superFrameOrder = sFOrder;
         }
        uint16_t getGackMapSize(){
            return slotsCFP * getMaxPacketsGTS();
        }

        void packetTransmitted(uint8_t GTS){
            if(transmittedPacketsGTS[GTS] < getMaxPacketsGTS()){
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
    };
}

#endif /* SRC_OPENDSME_HELPER_GACKHELPER_H_ */
