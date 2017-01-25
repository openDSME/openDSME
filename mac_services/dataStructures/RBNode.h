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

#ifndef RBNODE_H_
#define RBNODE_H_

#include "RBTree.h"
#include "RBTreeIterator.h"

namespace dsme {

/* ENUM **********************************************************************/
enum color_t { RED, BLACK };

/* CLASSES *******************************************************************/
template <typename T, typename K>
class RBTree;

template <typename T, typename K>
class RBTreeIterator;

template <typename T, typename K>
struct RBNode {
    RBNode(const T& content, const K& key);
    virtual ~RBNode() = default;

    /*
     * Get the object stored in the node
     */
    T& getContent();
    const T& getContent() const;

    /*
     * Pointers to construct the tree
     */
    RBNode<T, K> *leftChild, *rightChild, *parent;

    /*
     * Key to identify object
     */
    K key;

    /*
     * Content stored in the node
     */
    T content;

    /*
     * Color of the node (Red-Black-Tree)
     */
    color_t color;
};
/* FUNCTION DEFINITIONS ******************************************************/
template <typename T, typename K>
RBNode<T, K>::RBNode(const T& content, const K& key) : leftChild(nullptr), rightChild(nullptr), parent(nullptr), key(key), content(content), color(RED) {
}

template <typename T, typename K>
T& RBNode<T, K>::getContent() {
    return content;
}

template <typename T, typename K>
const T& RBNode<T, K>::getContent() const {
    return content;
}
}
#endif /* RBNODE_H_ */
