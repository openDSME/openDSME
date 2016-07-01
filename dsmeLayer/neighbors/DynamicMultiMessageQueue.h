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

#ifndef DYNAMICMULTIMESSAGEQUEUE_H_
#define DYNAMICMULTIMESSAGEQUEUE_H_

/* INCLUDES ******************************************************************/

#include <stdint.h>

#include "MessageQueueEntry.h"
#include "NeighborListEntry.h"

namespace dsme {

/* TYPES *********************************************************************/

typedef uint8_t queue_size_t;

/* CLASSES *******************************************************************/

/**
 * A queue for a fixed maximum number of messages for different neighbors
 * @template-param T type of nodes to store
 * @template-param S size of allocated chunks
 * @template-param M max number of chunks to allocate
 */
template<typename T, uint8_t S, uint8_t M>
class DynamicMultiMessageQueue {
private:
    /**
     * Represents one memory chunk to store a fixed number of messages
     * @template-param T type of nodes to store
     * @template-param S size of allocated chunks
     */
    struct Chunk {
        Chunk() :
                next(nullptr) {
            /* link all free slots inside the new chunks */
            for (uint8_t i = 0; i < S - 1; i++) {
                data[i].next = &(data[i + 1]);
            }
        }
        /* Chunks are organised in a linked list */
        Chunk* next;

        /* Every chunk stores 'S' messages of type 'T' */
        MessageQueueEntry<T> data[S];
    };

public:
    DynamicMultiMessageQueue();
    virtual ~DynamicMultiMessageQueue();

    /**
     * Adds a new message to the queue of a neighbor
     * -> time: O(1)
     * @param neighbor the neighbor the message belongs to
     * @param msg pointer to the message, ownership STAYS with caller
     */
    void push_back(NeighborListEntry<T>& neighbor, T* msg);

    /**
     * Gets and removes the first (oldest) element of the queue of a neighbor, nullptr if not existent
     * -> time: O(1)
     * @param neighbor the neighbor the message belongs to
     */
    T* pop_front(NeighborListEntry<T>& neighbor);

    /**
     * Gets the first (oldest) element of the queue of a neighbor, nullptr if not existent
     * -> time: O(1)
     * @param neighbor the neighbor the message belongs to
     */
    T* front(const NeighborListEntry<T>& neighbor);

    /**
     * Deletes all [but first] messages from the queue of a neighbor
     * -> time: O(neighbor->queueSize)
     * @param neighbor the neighbor the messages belong to
     * @param if true, first message is preserved
     */
    void flush(NeighborListEntry<T>& neighbor, bool keepFront);

    bool isFull() {
        return full;
    }

private:
    Chunk* chunkList;
    Chunk* lastChunk;
    uint8_t chunkCount;

    /* flag, set if queue is full */
    bool full;

    MessageQueueEntry<T>* freeFront;
    MessageQueueEntry<T>* freeBack;

    inline void addToFree(MessageQueueEntry<T> *entry);
    void tryAllocateChunk();
};

/* FUNCTION DEFINITIONS ******************************************************/

template<typename T, uint8_t S, uint8_t M>
DynamicMultiMessageQueue<T, S, M>::DynamicMultiMessageQueue() :
        chunkList(new Chunk()), chunkCount(1), full(false) {
    this->lastChunk = this->chunkList;
    this->freeFront = &(this->lastChunk->data[0]);
    this->freeBack = &(this->lastChunk->data[S - 1]);
}

template<typename T, uint8_t S, uint8_t M>
DynamicMultiMessageQueue<T, S, M>::~DynamicMultiMessageQueue() {
    Chunk *next, *toDelete = this->chunkList;
    while (toDelete != nullptr) {
        next = toDelete->next;
        delete toDelete;
        toDelete = next;
    }
}

template<typename T, uint8_t S, uint8_t M>
void DynamicMultiMessageQueue<T, S, M>::push_back(NeighborListEntry<T>& neighbor, T* msg) {

    if (this->full) {
        /* '-> all slots are used and no new chunk could be allocated */
        return;
    }

    MessageQueueEntry<T>* entry = this->freeFront;

    if (this->freeFront == this->freeBack) {
        /* '-> this was the last free spot */
        this->tryAllocateChunk();
    } else {
        /* '-> still multiple empty spots left */
        this->freeFront = this->freeFront->next;
    }

    entry->value = msg;
    entry->next = nullptr;

    if (neighbor.messageBack != nullptr) {
        neighbor.messageBack->next = entry;
    }
    neighbor.messageBack = entry;

    if (neighbor.messageFront == nullptr) {
        neighbor.messageFront = neighbor.messageBack;
    }

    neighbor.queueSize++;
}

template<typename T, uint8_t S, uint8_t M>
T* DynamicMultiMessageQueue<T, S, M>::pop_front(NeighborListEntry<T>& neighbor) {
    if (neighbor.queueSize > 0) {
        /* '-> queue contains messages for this neighbor */

        MessageQueueEntry<T> *entry = neighbor.messageFront;
        T* msg = entry->value;

        neighbor.messageFront = entry->next;

        if(neighbor.messageFront == nullptr) {
            neighbor.messageBack = nullptr;
        }

        this->addToFree(entry);

        neighbor.queueSize--;
        this->full = false;
        return msg;
    } else {
        /* '-> no messages pending for this neighbor */
        return nullptr;
    }
}

template<typename T, uint8_t S, uint8_t M>
T* DynamicMultiMessageQueue<T, S, M>::front(const NeighborListEntry<T>& neighbor) {
    return (neighbor.messageFront != nullptr) ? neighbor.messageFront->value : nullptr;
}

template<typename T, uint8_t S, uint8_t M>
void DynamicMultiMessageQueue<T, S, M>::flush(NeighborListEntry<T>& neighbor, bool keepFront) {
    MessageQueueEntry<T> *entry = neighbor.messageFront, *temp;

    if (keepFront && entry != nullptr) {
        /* keep existing first entry */
        temp = entry;
        entry = entry->next;
        temp->next = nullptr;

        neighbor.queueSize = 1;
    } else {
        /* discard first entry or list already empty */
        neighbor.messageFront = nullptr;
        neighbor.queueSize = 0;
    }

    if (entry != nullptr) {
        /* '-> there are entries to be invalidated */
        this->full = false;
    } else {
        /* '-> nothing to do */
        return;
    }

    /*
     * taken out of the loop for efficiency
     */
    this->addToFree(entry);
    entry = entry->next;

    while (entry != nullptr) {
        entry->value = nullptr;
        this->freeBack->next = entry;
        this->freeBack = entry;
        entry = entry->next;
    }

    neighbor.messageBack = neighbor.messageFront;
    return;
}

template<typename T, uint8_t S, uint8_t M>
inline void DynamicMultiMessageQueue<T, S, M>::addToFree(MessageQueueEntry<T> *entry) {
    entry->value = nullptr;

    if (this->freeFront == nullptr || this->freeBack == nullptr) {
        assert(this->freeFront == nullptr);
        assert(this->freeBack == nullptr);
        this->freeFront = entry;
    } else {
        this->freeBack->next = entry;
    }
    this->freeBack = entry;
    return;
}

template<typename T, uint8_t S, uint8_t M>
void DynamicMultiMessageQueue<T, S, M>::tryAllocateChunk() {
    if (this->chunkCount < M) {
        /* '-> allocate new chunk */
        this->lastChunk->next = new Chunk();
        this->lastChunk = this->lastChunk->next;

        this->freeFront = &(this->lastChunk->data[0]);
        this->freeBack = &(this->lastChunk->data[S - 1]);

        this->chunkCount++;
        this->full = false;
    } else {
        /* '-> no space for new chunks */
        this->freeFront = nullptr;
        this->freeBack = nullptr;
        this->full = true;
    }
}

}

#endif /* DYNAMICMULTIMESSAGEQUEUE_H_ */
