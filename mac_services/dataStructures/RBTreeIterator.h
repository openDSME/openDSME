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

#ifndef BITVECTORITERATOR_H_
#define BITVECTORITERATOR_H_

#define RBTREE_ITERATOR_POSTORDER

#include "RBTree.h"
#include "RBNode.h"

namespace dsme {

template<typename T, typename K>
class RBTree;

template<typename T, typename K>
struct RBNode;

template<typename T, typename K>
class RBTreeIterator {
    friend class RBTree<T, K> ;
public:
    /*
     * Somehow this is required for successful compilation.
     * TODO: Find out why
     * DO NOT USE, does not create usefull iterators
     */
    RBTreeIterator();

    RBTreeIterator(const RBTree<T, K>* instance, RBNode<T, K>* initialNode);
    virtual ~RBTreeIterator();

    RBTreeIterator<T, K>& operator=(const RBTreeIterator<T, K>&);
    RBTreeIterator<T, K>& operator++();

    RBTreeIterator<T, K> operator++(int);

    T& operator*();
    T* operator->();
    const T* operator->() const;
    RBNode<T, K>* node();

    bool operator==(const RBTreeIterator<T, K>&) const;
    bool operator!=(const RBTreeIterator<T, K>&) const;

    static RBTreeIterator<T, K> begin(RBTree<T, K>* instance, RBNode<T, K>* rootNode);

private:
    const RBTree<T, K>* instance;
    RBNode<T, K>* currentNode;
};

template<typename T, typename K>
RBTreeIterator<T, K>::RBTreeIterator() :
    instance(nullptr),
    currentNode(nullptr) {
}

template<typename T, typename K>
RBTreeIterator<T, K>::RBTreeIterator(const RBTree<T, K>* instance, RBNode<T, K>* initialNode) :
    instance(instance),
    currentNode(initialNode) {
}

template<typename T, typename K>
RBTreeIterator<T, K>::~RBTreeIterator() {
}

template<typename T, typename K>
RBTreeIterator<T, K>& RBTreeIterator<T, K>::operator=(const RBTreeIterator<T, K>& other) {
    this->instance = other.instance;
    this->currentNode = other.currentNode;
    return *this;
}

#ifdef RBTREE_ITERATOR_POSTORDER
/**
 * iterate over Tree in postorder
 */
template<typename T, typename K>
RBTreeIterator<T, K>& RBTreeIterator<T, K>::operator++() {
    RBNode<T, K>* parent;
    /*end iterator does not increment */
    if (this->currentNode == nullptr) {
        return *this;
    }
    parent = this->currentNode->parent;
    /*
     * reaches root -> next is end()
     */
    if (parent == nullptr) {
        this->currentNode = nullptr;
        return *this;
    }

    /*
     * left child -> go to right child
     * right child -> go to parent
     */
    if ((this->currentNode == parent->leftChild) && (parent->rightChild != nullptr)) {
        this->currentNode = parent->rightChild;
    } else {
        this->currentNode = this->currentNode->parent;
        return *this;
    }
    while (true) {
        if (this->currentNode->leftChild != nullptr) {
            /* '-> has left child node */
            this->currentNode = this->currentNode->leftChild;
        } else if (this->currentNode->rightChild != nullptr) {
            /* '-> only right child node */
            this->currentNode = this->currentNode->rightChild;
        } else {
            return *this;
        }
    } //endwhile
}

#else

template<typename T, typename K>
RBTreeIterator<T, K>& RBTreeIterator<T, K>::operator++() {
    if (this->currentNode == nullptr) {
        return *this;
    }
    RBNode<T, K>* n;
    if (this->currentNode->leftChild != nullptr) {
        /* '-> has left child node -> visit always */
        this->currentNode = this->currentNode->leftChild;
    } else if (this->currentNode->rightChild != nullptr) {
        /* '-> only right child node -> visit always */
        this->currentNode = this->currentNode->rightChild;
    } else {
        n = this->currentNode;
        while (true) {
            /* '-> traverse back until node with unvisited right neighbor or root */
            if (n->parent == nullptr) {
                this->currentNode = nullptr;
                break;
            } else if (n == n->parent->leftChild
                       && n->parent->rightChild != nullptr) {
                this->currentNode = n->parent->rightChild;
                break;
            } else {
                n = n->parent;
            }
        }
    }
    return *this;
}

#endif

template<typename T, typename K>
RBTreeIterator<T, K> RBTreeIterator<T, K>::operator++(int) {
    RBTreeIterator<T, K> old = *this;
    ++(*this);
    return old;
}

template<typename T, typename K>
T& RBTreeIterator<T, K>::operator*() {
    return this->currentNode->getContent();
}

template<typename T, typename K>
T* RBTreeIterator<T, K>::operator->() {
    return &(this->currentNode->getContent());
}

template<typename T, typename K>
const T* RBTreeIterator<T, K>::operator->() const {
    return &(this->currentNode->getContent());
}

template<typename T, typename K>
RBNode<T, K>* RBTreeIterator<T, K>::node() {
    return this->currentNode;
}

template<typename T, typename K>
bool RBTreeIterator<T, K>::operator==(const RBTreeIterator<T, K>& other) const {
    return (this->instance == other.instance && this->currentNode == other.currentNode);
}

template<typename T, typename K>
bool RBTreeIterator<T, K>::operator!=(const RBTreeIterator<T, K>& other) const {
    return !((*this) == other);
}

#ifdef RBTREE_ITERATOR_POSTORDER
template<typename T, typename K>
RBTreeIterator<T, K> RBTreeIterator<T, K>::begin(RBTree<T, K>* instance, RBNode<T, K>* rootNode) {
    if (rootNode == nullptr) {
        return RBTreeIterator(instance, rootNode);
    }
    while (true) {
        if (rootNode->leftChild != nullptr) {
            rootNode = rootNode->leftChild;
        } else if (rootNode->rightChild != nullptr) {
            rootNode = rootNode->rightChild;
        } else {
            return RBTreeIterator(instance, rootNode);
        }
    }
}
#else
template<typename T, typename K>
RBTreeIterator<T, K> RBTreeIterator<T, K>::begin(RBTree<T, K>* instance,
        RBNode<T, K>* rootNode) {
    return RBTreeIterator(instance, rootNode);
}
#endif

}

#endif
