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

#ifndef NEIGHBORQUEUE_H_
#define NEIGHBORQUEUE_H_

/* INCLUDES ******************************************************************/

#include <stdint.h>

#include "../../../dsme_settings.h"
#include "../../mac_services/dataStructures/RBTree.h"
#include "../../mac_services/dataStructures/RBTreeIterator.h"
#include "DynamicMultiMessageQueue.h"
#include "Neighbor.h"

namespace dsme {

/* TYPES *********************************************************************/

typedef uint8_t neighbor_size_t;
class DSMEMessage;

/* CLASSES *******************************************************************/

/*
 * @template-param N maximum number of neighbors
 */
template<uint8_t N>
class NeighborQueue {
public:
    typedef RBTree<NeighborListEntry<DSMEMessage>, IEEE802154MacAddress>::iterator iterator;

    iterator begin();

    const iterator end() const;

    /*
     * adds a Neighbor
     * @param neighbor which will be added
     */
    void addNeighbor(Neighbor& neighbor);

    /*
     * erase a Neighbor
     * @param address for the element which should be erased
     */
    void eraseNeighbor(iterator& neighbor);

    /*
     * gives the number of Neighbors
     *
     * @return number of Neighbors
     */
    neighbor_size_t getNumNeighbors() const;

    iterator findByAddress(const IEEE802154MacAddress& address);

    queue_size_t getPacketsInQueue(const iterator& neighbor) const;
    bool isQueueEmpty(iterator& neighbor);

    DSMEMessage* front(iterator& neighbor);

    DSMEMessage* popFront(iterator& neighbor);

    void pushBack(iterator& neighbor, DSMEMessage* msg);

    void flushQueues(bool keepFront);

    bool isQueueFull() const {
        return queue.isFull();
    }

private:
    DynamicMultiMessageQueue<DSMEMessage, TOTAL_GTS_QUEUE_SIZE, 1> queue;
    RBTree<NeighborListEntry<DSMEMessage>, IEEE802154MacAddress> neighbors;
};

/* FUNCTION DEFINITIONS ******************************************************/

template<uint8_t N>
typename NeighborQueue<N>::iterator NeighborQueue<N>::begin() {
    return neighbors.begin();
}

template<uint8_t N>
const typename NeighborQueue<N>::iterator NeighborQueue<N>::end() const {
    return neighbors.end();
}

template<uint8_t N>
void NeighborQueue<N>::addNeighbor(Neighbor& neighbor) {
    if (neighbors.size() < N) {
        neighbors.insert(NeighborListEntry<DSMEMessage>(neighbor), neighbor.address);
        return;
    } else {
        return;
    }
}

template<uint8_t N>
void NeighborQueue<N>::eraseNeighbor(iterator& neighbor) {
    if (neighbor != neighbors.end()) {
        queue.flush(*neighbor, false);
        neighbors.remove(neighbor);
    }
    return;
}

template<uint8_t N>
neighbor_size_t NeighborQueue<N>::getNumNeighbors() const {
    return neighbors.size();
}

template<uint8_t N>
typename NeighborQueue<N>::iterator NeighborQueue<N>::findByAddress(const IEEE802154MacAddress& address) {
    return neighbors.find(address);
}

template<uint8_t N>
queue_size_t NeighborQueue<N>::getPacketsInQueue(const iterator& neighbor) const{
    if(neighbor != end()) {
        return neighbor->queueSize;
    } else {
        return 0;
    }
}

template<uint8_t N>
bool NeighborQueue<N>::isQueueEmpty(iterator& neighbor) {
    return (neighbor->queueSize == 0);
}

template<uint8_t N>
DSMEMessage* NeighborQueue<N>::front(iterator& neighbor) {
    return queue.front(*neighbor);
}

template<uint8_t N>
DSMEMessage* NeighborQueue<N>::popFront(iterator& neighbor) {
    return queue.pop_front(*neighbor);
}

template<uint8_t N>
void NeighborQueue<N>::pushBack(iterator& neighbor, DSMEMessage* msg) {
    queue.push_back(*neighbor, msg);
    return;
}

template<uint8_t N>
void NeighborQueue<N>::flushQueues(bool keepFront) {
    for (iterator i = neighbors.begin(); i != neighbors.end(); ++i) {
        queue.flush(*i, keepFront);
    }
    return;
}

}

#endif

