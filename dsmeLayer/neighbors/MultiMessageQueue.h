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

#ifndef MULTIMESSAGEQUEUE_H_
#define MULTIMESSAGEQUEUE_H_

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
 * @template-param S size of allocated chunk
 */
template <typename T, uint8_t S>
class MultiMessageQueue {
private:
    /**
     * Represents one memory chunk to store a fixed number of messages
     * @template-param T type of nodes to store
     * @template-param S size of allocated chunk
     */
    struct Chunk {
        Chunk() {
            /* link all free slots inside the new chunk */
            for(uint8_t i = 0; i < S - 1; i++) {
                data[i].next = &(data[i + 1]);
            }
        }

        /* Every chunk stores 'S' messages of type 'T' */
        MessageQueueEntry<T> data[S];
    };

public:
    MultiMessageQueue();
    MultiMessageQueue(const MultiMessageQueue&) = delete;

    virtual ~MultiMessageQueue();

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

    bool isFull() const {
        return full;
    }

private:
    Chunk chunk;

    /* flag, set if queue is full */
    bool full;

    MessageQueueEntry<T>* freeFront;
    MessageQueueEntry<T>* freeBack;

    inline void addToFree(MessageQueueEntry<T>* entry);
};

/* FUNCTION DEFINITIONS ******************************************************/

template <typename T, uint8_t S>
MultiMessageQueue<T, S>::MultiMessageQueue() : full(false) {
    this->freeFront = &(this->chunk.data[0]);
    this->freeBack = &(this->chunk.data[S - 1]);
}

template <typename T, uint8_t S>
MultiMessageQueue<T, S>::~MultiMessageQueue() {
}

template <typename T, uint8_t S>
void MultiMessageQueue<T, S>::push_back(NeighborListEntry<T>& neighbor, T* msg) {
    if(this->full) {
        /* '-> all slots are used */
        DSME_ASSERT(false);
        return;
    }

    MessageQueueEntry<T>* entry = this->freeFront;

    if(this->freeFront == this->freeBack) {
        /* '-> this was the last free spot */
        this->freeFront = nullptr;
        this->freeBack = nullptr;
        this->full = true;
    } else {
        /* '-> still multiple empty spots left */
        this->freeFront = this->freeFront->next;
    }

    entry->value = msg;
    entry->next = nullptr;

    if(neighbor.messageBack != nullptr) {
        neighbor.messageBack->next = entry;
    }
    neighbor.messageBack = entry;

    if(neighbor.messageFront == nullptr) {
        neighbor.messageFront = neighbor.messageBack;
    }

    neighbor.queueSize++;
}

template <typename T, uint8_t S>
T* MultiMessageQueue<T, S>::pop_front(NeighborListEntry<T>& neighbor) {
    if(neighbor.queueSize > 0) {
        /* '-> queue contains messages for this neighbor */

        MessageQueueEntry<T>* entry = neighbor.messageFront;
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

template <typename T, uint8_t S>
T* MultiMessageQueue<T, S>::front(const NeighborListEntry<T>& neighbor) {
    return (neighbor.messageFront != nullptr) ? neighbor.messageFront->value : nullptr;
}

template <typename T, uint8_t S>
void MultiMessageQueue<T, S>::flush(NeighborListEntry<T>& neighbor, bool keepFront) {
    MessageQueueEntry<T>* entry = neighbor.messageFront;

    if(keepFront && entry != nullptr) {
        /* keep existing first entry */
        MessageQueueEntry<T>* temp = entry;
        entry = entry->next;
        temp->next = nullptr;

        neighbor.queueSize = 1;
    } else {
        /* discard first entry or list already empty */
        neighbor.messageFront = nullptr;
        neighbor.queueSize = 0;
    }

    if(entry != nullptr) {
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

    while(entry != nullptr) {
        entry->value = nullptr;
        this->freeBack->next = entry;
        this->freeBack = entry;
        entry = entry->next;
    }

    neighbor.messageBack = neighbor.messageFront;
    return;
}

template <typename T, uint8_t S>
inline void MultiMessageQueue<T, S>::addToFree(MessageQueueEntry<T>* entry) {
    DSME_ASSERT(entry != nullptr);
    entry->value = nullptr;

    if(this->freeFront == nullptr || this->freeBack == nullptr) {
        DSME_ASSERT(this->freeFront == nullptr);
        DSME_ASSERT(this->freeBack == nullptr);
        this->freeFront = entry;
    } else {
        this->freeBack->next = entry;
    }
    this->freeBack = entry;
    return;
}

} /* namespace dsme */

#endif /* MULTIMESSAGEQUEUE_H_ */
