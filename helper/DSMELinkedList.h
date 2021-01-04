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

#ifndef DSMELINKEDLIST_H_
#define DSMELINKEDLIST_H_

#include "./Integers.h"

namespace dsme {

template <typename C>
class DSMELinkedList {
public:
    DSMELinkedList(){
        head = nullptr;
        tail = nullptr;
        size = 0;
    }

    DSMELinkedList(DSMELinkedList const& other) {
        size = 0;
        for(uint8_t i=0; i<other.getSize(); i++) {
            this->insertLast(other.getElementAt(i));
        }
    }

    ~DSMELinkedList(){
        while(size>0){
            deleteLast();
        }
    }

    void insertFirst(C element)
    {
        struct listNode* newItem;
        newItem=new listNode;
        size++;
        if(head==nullptr)
        {
            head=newItem;
            newItem->prev=nullptr;
            newItem->element=element;
            newItem->next=nullptr;
            tail=newItem;
        }
        else
        {
            newItem->next=head;
            newItem->element=element;
            newItem->prev=nullptr;
            head->prev=newItem;
            head=newItem;
        }
    }

    void insertLast(C element)
    {
        struct listNode* newItem;
        newItem=new listNode;
        newItem->element=element;
        size++;
        if(head==nullptr)
        {
            head=newItem;
            newItem->prev=nullptr;
            newItem->next=nullptr;
            tail=newItem;
        }
        else
        {
            newItem->prev=tail;
            tail->next=newItem;
            newItem->next=nullptr;
            tail=newItem;
        }
    }

    void deleteFirst()
    {
        if(head==nullptr)
        {
            return;
        }
        if(head==tail)///one element in the list
        {
            struct listNode* cur;
            cur=head;
            head=nullptr;
            tail=nullptr;
            delete cur;
            cur = nullptr;
            size--;
            return;
        }
        else
        {
            struct listNode* cur;
            cur=head;
            head=head->next;
            head->prev=nullptr;
            delete cur;
            cur = nullptr;
            size--;
        }
    }

    void deleteLast()
    {
        if(head==nullptr) return;
        if(head==tail)
        {
            struct listNode* cur;
            cur=head;
            head=nullptr;
            tail=nullptr;
            delete cur;
            cur = nullptr;
            size--;
            return;
        }
        else
        {
            struct listNode* cur;
            cur=tail;
            tail=tail->prev;
            tail->next=nullptr;
            delete cur;
            cur = nullptr;
            size--;
        }
    }

    C& getElementAt(uint8_t id) const
    {
        struct listNode* temp = head;
        while(id--)
        {
            temp=temp->next;
        }
        return temp->element;
    }

    C& getLast()
    {
        return tail->element;
    }

    bool contains(C element) const
    {
        struct listNode* temp;
        temp=head;
        while(temp!=nullptr)
        {
            if(temp->element==element)
            {
                return true;
                break;
            }
            temp=temp->next;
        }
        return false;
    }

    void deleteItemIfExists(C element)
    {
        struct listNode* temp;
        temp=head;
        if(head==tail)
        {
            if(head->element!=element)
            {
                return;
            }
            head=nullptr;
            tail=nullptr;
            size--;
            delete temp;
            return;
        }
        if(head->element==element)
        {
            head=head->next;
            head->prev=nullptr;
            size--;
            delete temp;
            return;
        }
        else if(tail->element==element)
        {
            temp=tail;
            tail=tail->prev;
            tail->next=nullptr;
            size--;
            delete temp;
            return;
        }
        while(temp->element!=element)
        {
            temp=temp->next;
            if(temp==nullptr)
            {
                return;
            }
        }
        temp->next->prev=temp->prev;
        temp->prev->next=temp->next;
        size--;
        delete temp;
    }

    uint8_t getSize() const
    {
        return size;
    }

private:
    struct listNode{
        C element;
        listNode *next;
        listNode *prev;
    };
    listNode *head, *tail;
    uint8_t size;
};

} /* namespace dsme */

#endif /* DSMELINKEDLIST_H_ */
