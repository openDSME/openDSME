/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * introduced in the IEEE 802.15.4e-2012 standard
 *
 * Authors: Florian Meier <florian.meier@tuhh.de>
 *          Maximilian Koestler <maximilian.koestler@tuhh.de>
 *          Sandrina Backhauss <sandrina.backhauss@tuhh.de>
 *
 * Based on
 *          DSME Implementation for the INET Framework
 *          Tobias Luebkert <tobias.luebkert@tuhh.de>
 *
 * Copyright (c) 2015, Institute of Telematics, Hamburg University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef IELIST_H_
#define IELIST_H_

#include "./Integers.h"
#include "InformationElement.h"
#include "DSMELinkedList.h"

namespace dsme {

class IEList {
public:
    IEList(){

    }
    ~IEList(){

    }

    bool contains(InformationElement::tIEID ieID){
        for(uint8_t id = 0; id < 255;id++){
            IEListNode *element = ieNodeList.getElementAt(id);
            if(element->ieID == ieID){
                return true;
            }
        }
        return false;
    }

    InformationElement* getIEByID(InformationElement::tIEID ieID){
        for(uint8_t id = 0; id < 255;id++){
            IEListNode *element = ieNodeList.getElementAt(id);
            if(element->ieID == ieID){
                return element->ieElement;
            }
        }
        return nullptr;
    }

    void insert(InformationElement *element){
        IEListNode *node = new IEListNode;
        //DSME_ASSERT(node != nullptr);
        node->ieElement = element;
        node->ieID = element->getIEID();
        ieNodeList.insertLast(node);
    }

    uint8_t getSize()
    {
        return ieNodeList.getSize();
    }

    uint8_t getSerializationLength() {
        uint8_t len = 0;
        for(uint8_t id = 0; id < 255;id++){
            IEListNode *element = ieNodeList.getElementAt(id);
            if(element == nullptr){ //reached end
                break;
            }
            len+=element->getSerializationLength();
        }
        len+=1; //stopIEID
        return len;
    }

    void serialize(Serializer& serializer) {
        if(serializer.getType() == SERIALIZATION){
            for(uint8_t id = 0; id < 255;id++){
                IEListNode *element = ieNodeList.getElementAt(id);
                if(element == nullptr){ //reached end
                    break;
                }
                element->serialize(serializer);
            }
            serializer.push((uint8_t)InformationElement::ID_stopIE);    //serializer << (uint8_t)InformationElement::ID_stopIE;    //set stopIE
        }
        else{   //DESERIALIZATION
            //create a new IEListNode object and let it serialize itself
            InformationElement::tIEID tmpID;
            tmpID = (InformationElement::tIEID) serializer.pop();    //serializer << (uint8_t)tmpID;
            if(tmpID == InformationElement::ID_stopIE){
                return; //this was the last IE in the list
            }
            IEListNode *element = new IEListNode;
            //DSME_ASSERT(element != nullptr);
            element->ieID = tmpID;
            element->serialize(serializer);
        }
    }

private:
    struct IEListNode{
        InformationElement::tIEID ieID;
        InformationElement *ieElement;

        uint8_t getSerializationLength() {
            uint8_t len = 0;
            len += 1;   //ieID
            len += ieElement->getSerializationLength();
            return len;
        }
        void serialize(Serializer& serializer) {
            if(serializer.getType() == SERIALIZATION){
                serializer.push(ieID);  //serializer << (uint8_t)this->ieID;
                ieElement->serialize(serializer);
            }
            else{   //DESERIALIZATION
                switch (ieID) { //maybe this can be automated? getObjectByID?
                    case InformationElement::ID_lastMessage:
                        ieElement = (lastMessageIE*) new lastMessageIE;
                        break;
                    case InformationElement::ID_gackEnabled:
                        ieElement = (gackEnabledIE*) new gackEnabledIE;
                        break;
                    case InformationElement::ID_gackResponse:
                        ieElement = (gackResponseIE*) new gackResponseIE;
                        break;
                    default:
                        //DSME_ASSERT(false);
                        break;
                }
                //DSME_ASSERT(ieElement != nullptr);
                ieElement->serialize(serializer);
            }
        }
    };
    DSMELinkedList<IEListNode*> ieNodeList;
};

} /* namespace dsme */

#endif /* IELIST_H_ */
