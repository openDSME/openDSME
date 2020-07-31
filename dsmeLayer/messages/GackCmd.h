/*
 * GackCmd.h
 *
 *  Created on: 29.07.2020
 *      Author: cjd8627
 */

#ifndef SRC_OPENDSME_DSMELAYER_MESSAGES_GACKCMD_H_
#define SRC_OPENDSME_DSMELAYER_MESSAGES_GACKCMD_H_

#include "../../mac_services/dataStructures/DSMEMessageElement.h"


class GackCmd : public DSMEMessageElement{
private:
    BitVector<7*10> gAckMap;
public:
    GackCmd();
    virtual ~GackCmd();

    uint8_t getSerializationLength(){

    }

    void serialize(Serializer& serializer){

    }
};

#endif /* SRC_OPENDSME_DSMELAYER_MESSAGES_GACKCMD_H_ */
