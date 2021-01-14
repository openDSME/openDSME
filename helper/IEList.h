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
        ieListID = ieListIDCtr;
        ieListIDCtr++;
    }
    IEList& operator=(IEList const& other) {    //copy ieList
        uint8_t len = other.getSerializationLength();
        uint8_t *buffer = new uint8_t[len];
        uint8_t *movingPtr = buffer;
        if(buffer != nullptr){
            for(uint8_t i=0;i<len;i++){
                buffer[i]=0;
            }
            other.serializeTo(movingPtr);
            movingPtr = buffer; //reset pointer
            this->deserializeFrom((const uint8_t*&)movingPtr, len);
            delete[] buffer;
            buffer = nullptr;
            movingPtr = nullptr;
        }
        return *this;
    }

    ~IEList(){
        while(ieNodeList.getSize()>0){
            IEListNode &element = ieNodeList.getLast();
            delete element.ieElement;
            element.ieElement = nullptr;
            ieNodeList.deleteLast();
        }
        ieListIDCtr--;
    }

    bool contains(InformationElement::tIEID ieID){
        for(uint8_t id = 0; id < ieNodeList.getSize();id++){
            IEListNode &element = ieNodeList.getElementAt(id);
            if(element.ieID == ieID){
                return true;
            }
        }
        return false;
    }

    InformationElement* getIEByID(InformationElement::tIEID ieID){
        for(uint8_t id = 0; id < ieNodeList.getSize();id++){
            IEListNode &element = ieNodeList.getElementAt(id);
            if(element.ieID == ieID){
                return element.ieElement;
            }
        }
        return nullptr;
    }

    void insert(InformationElement *element){
        IEListNode node;
        node.ieElement = element;
        node.ieID = element->getIEID();
        ieNodeList.insertLast(node);
    }

    uint8_t getSize()
    {
        return ieNodeList.getSize();
    }

    uint8_t getSerializationLength() const{
        uint8_t len = 0;
        for(uint8_t id = 0; id < ieNodeList.getSize();id++){
            IEListNode &element = ieNodeList.getElementAt(id);
            len+=element.getSerializationLength();
        }
        len+=1; //stopIEID
        return len;
    }

    void serializeTo(uint8_t*& buffer) const{
        for(uint8_t id = 0; id < ieNodeList.getSize();id++){
            IEListNode &element = ieNodeList.getElementAt(id);
            element.serializeTo(buffer);
        }
        *(buffer++) = (uint8_t)InformationElement::ID_stopIE;   //set stopIE
    }

    void deserializeFrom(const uint8_t*& buffer, uint8_t payloadLength){
        for(uint8_t id = 0; id < 255;id++){
            //create a new IEListNode object and let it deserialize itself
            InformationElement::tIEID tmpID;
            tmpID = (InformationElement::tIEID) *(buffer++);
            if(tmpID == InformationElement::ID_stopIE){
                return; //this was the last IE in the list
            }
            IEListNode element;
            element.ieID = tmpID;
            element.deserializeFrom(buffer, payloadLength);
            ieNodeList.insertLast(element);
        }
    }

private:
    static uint16_t ieListIDCtr;
    uint16_t ieListID;
    struct IEListNode{
        InformationElement::tIEID ieID;
        InformationElement *ieElement;

        IEListNode() = default;
        /*
        IEListNode(IEListNode const& other) {
            this->ieID = other.ieID;
            if(other.ieID == InformationElement::ID_lastMessage) {
                lastMessageIE *ie = new lastMessageIE();
                *ie = *reinterpret_cast<lastMessageIE*>(other.ieElement);
                this->ieElement = ie;
            } else if(other.ieID == InformationElement::ID_gackEnabled) {
                gackEnabledIE *ie = new gackEnabledIE();
                *ie = *reinterpret_cast<gackEnabledIE*>(other.ieElement);
                this->ieElement = ie;
            } else if(other.ieID == InformationElement::ID_gackResponse) {
                gackResponseIE *ie = new gackResponseIE();
                *ie = *reinterpret_cast<gackResponseIE*>(other.ieElement);
                this->ieElement = ie;
            }
        }
        */

        uint8_t getSerializationLength() {
            uint8_t len = 0;
            len += 1;   //ieID
            len += ieElement->getSerializationLength();
            return len;
        }

        void serializeTo(uint8_t*& buffer) {
            *(buffer++) = (uint8_t)this->ieID;
            ieElement->serializeTo(buffer);
        }

        void deserializeFrom(const uint8_t*& buffer, uint8_t payloadLength){
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
                    ieElement = nullptr;
                    break;
            }
            if(ieElement == nullptr){
                //assert
            }
            ieElement->deserializeFrom(buffer, payloadLength);
        }
    };
    DSMELinkedList<IEListNode> ieNodeList;
};

} /* namespace dsme */

#endif /* IELIST_H_ */
