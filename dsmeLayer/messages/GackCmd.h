/*
 * GackCmd.h
 *
 *  Created on: 29.07.2020
 *      Author: cjd8627
 */

#ifndef SRC_OPENDSME_DSMELAYER_MESSAGES_GACKCMD_H_
#define SRC_OPENDSME_DSMELAYER_MESSAGES_GACKCMD_H_

#include "../../mac_services/dataStructures/DSMEMessageElement.h"
#include "../../mac_services/dataStructures/DSMEBitVector.h"

namespace dsme {
class GackCmd : public DSMEMessageElement {
    private:
        uint8_t gackMapSize;
        BitVector<GACK_MAX_SIZE> gackMap;
    public:
        GackCmd(uint8_t size){
            gackMap.setLength(size);
            gackMapSize = size;
        }
        GackCmd(BitVector<GACK_MAX_SIZE> BitVector) : gackMap(BitVector){
        }

        virtual uint8_t getSerializationLength(){
            uint8_t size = 0;
            size += gackMap.getSerializationLength();
            return size;
        }

        virtual void serialize(Serializer& serializer){
            serializer << gackMap;
        }

        BitVector<GACK_MAX_SIZE> getGackMap(){
            return gackMap;
        }
    };
}
#endif /* SRC_OPENDSME_DSMELAYER_MESSAGES_GACKCMD_H_ */
