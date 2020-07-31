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
        BitVector<7*50> gAckMap;
    public:
        GackCmd(){}
        GackCmd(BitVector<7*50> BitVector):gAckMap(BitVector){
        }

        virtual uint8_t getSerializationLength(){
            uint8_t size = 0;
            size += gAckMap.getSerializationLength();
            return size;
        }

        virtual void serialize(Serializer& serializer){
            serializer << gAckMap;
        }
    };
}
#endif /* SRC_OPENDSME_DSMELAYER_MESSAGES_GACKCMD_H_ */
