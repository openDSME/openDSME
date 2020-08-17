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
#include "../../mac_services/pib/MAC_PIB.h"

namespace dsme {
class GackCmd : public DSMEMessageElement {
    private:
        static const uint8_t GackMapSize = 7*20;
        BitVector<GackMapSize> GackMap;
    public:
        GackCmd(){
            GackMap.setLength(GackMapSize);
        }
        GackCmd(BitVector<GackMapSize> BitVector) : GackMap(BitVector){
        }

        virtual uint8_t getSerializationLength(){
            uint8_t size = 0;
            size += GackMap.getSerializationLength();
            return size;
        }

        virtual void serialize(Serializer& serializer){
            serializer << GackMap;
        }

        BitVector<GackMapSize> getGackMap(){
            return GackMap;
        }
    };
}
#endif /* SRC_OPENDSME_DSMELAYER_MESSAGES_GACKCMD_H_ */
