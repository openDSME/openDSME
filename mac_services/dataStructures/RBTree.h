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

#ifndef RBTREE_H_
#define RBTREE_H_

#include "RBNode.h"
#include "RBTreeIterator.h"

#include <stdint.h>

namespace dsme {

/* CLASSES *******************************************************************/

template<typename T, typename K>
struct RBNode;

/*
 * generic implementation for an RB-Tree
 * advantage: balanced binary search tree -> find() in maximal O(log n) steps
 */
template<typename T, typename K>
class RBTree {

public:
    typedef RBTreeIterator<T, K> iterator;
    typedef uint8_t tree_size_t;

    /*
     * initialize membervariables
     */
    RBTree();

    /*
     * It is only valid if postorder RBTreeIterator is used.
     *
     */
    virtual ~RBTree();

    /*
     * do not allow duplication of a tree
     */
    RBTree(const RBTree&) = delete;
    RBTree& operator=(const RBTree&) = delete;

    /*
     * stores new object at correct position
     * @Param obj: object to be inserted
     *        key: key to identify the object
     * @return true, if insert was successful
     */
    bool insert(T obj, K key);

    /*
     * removes object and reconstruct a new RBTree
     * @Param iterator that points to node that shall be removed
     */
    void remove(iterator& iter);

    /*
     * find object with key key
     * @Param key: key to identify the object
     */
    iterator find(K key);

    /*
     * return number of elements in the RBTree
     */
    tree_size_t size() const;

    /*
     * return iterator to root
     */
    iterator begin();

    /*
     * return null iterator
     */
    iterator end();
    const iterator end() const;

    RBNode<T, K>* getRoot();

private:
    /*
     * root node
     */
    RBNode<T, K>* root;

    /*
     * number of nodes in the RBTree
     */
    tree_size_t m_size;

    /*
     * rotate tree to the right, if not balanced
     * @Param node x is center of the rotation
     */
    void rotate_right(RBNode<T, K>* x);

    /*
     * rotate tree to the left, if not balanced
     * @Param node x is center of the rotation
     */
    void rotate_left(RBNode<T, K>* x);

    /*
     * encapsulate finding grandparent node
     * @Param current node
     */
    RBNode<T, K>* grandparent(RBNode<T, K>* x);

    /*
     * encapsulate finding grandparent node
     * @Param current node
     */
    RBNode<T, K>* uncle(RBNode<T, K>* x);

    /*
     * encapsulate finding sibling node
     * @Param current node
     */
    RBNode<T, K>* sibling(RBNode<T, K>* x);

    /*
     * find the smallest node in right subtree of x, that node has only one child
     * @Param current node, that shall be deleted
     */
    RBNode<T, K>* findSwapNode(RBNode<T, K>* x);

    /*
     * covers special cases for retaining RBTree structure
     * @Param node: node that should be deleted
     */
    void balanceTree(RBNode<T, K>* node);

};

/* FUNCTION DEFINITIONS ******************************************************/

template<typename T, typename K>
RBTree<T, K>::RBTree() :
    root(nullptr), m_size(0) {
}

template<typename T, typename K>
RBTree<T, K>::~RBTree() {
    iterator iter = this->begin();
    while (iter != this->end()) {
        delete (iter++).currentNode;
    }
}

template<typename T, typename K>
RBNode<T, K>* RBTree<T, K>::getRoot() {
    return root;
}

template<typename T, typename K>
typename RBTree<T, K>::iterator RBTree<T, K>::begin() {
    return RBTree<T, K>::iterator::begin(this, root);
}

template<typename T, typename K>
typename RBTree<T, K>::iterator RBTree<T, K>::end() {
    return RBTree<T, K>::iterator(this, nullptr);
}

template<typename T, typename K>
const typename RBTree<T, K>::iterator RBTree<T, K>::end() const {
    return RBTree<T, K>::iterator(this, nullptr);
}

template<typename T, typename K>
bool RBTree<T, K>::insert(T obj, K key) {
    RBNode<T, K>* node;
    if (m_size == (tree_size_t) - 1) {
        /* '-> tree is already full, depends on bit-width of 'tree_size_t' */
        return false;
    }
    if (m_size == 0) {
        node = new RBNode<T, K>(obj, key);
        //tree was empty -> inserted object becomes root
        node->parent = nullptr;
        node->color = BLACK; // Transformation_1: tree was empty before
        root = node;
    } else {
        //tree was not empty
        RBNode<T, K>* current = root;
        while (true) {
            if (key == current->key) {
                //double key means same node -> no insert necessary
                return false;
            } else if (key < current->key) {
                // key is smaller -> left path
                if (current->leftChild == nullptr) {
                    // has no left child -> current node sets his child -> position found
                    node = new RBNode<T, K>(obj, key);
                    node->parent = current;
                    current->leftChild = node;
                    break;
                }
                current = current->leftChild;
            } else if (key > current->key) {
                // key is bigger -> right path
                if (current->rightChild == nullptr) {
                    // has no right child -> current node sets his child -> position found
                    node = new RBNode<T, K>(obj, key);
                    node->parent = current;
                    current->rightChild = node;
                    break;
                }
                current = current->rightChild;
            } //end if
        } //end while
        /*
         * correct position for insertion found
         */

        while (true) {
            RBNode<T, K>* P, *G, *U;
            P = node->parent;
            G = grandparent(node); // grandparent from node
            U = uncle(node); // uncle from node

            if (P == nullptr || P->color == BLACK) { // Transformation_2 : parent node is BLACK
                break; //nothing to do
            } else if (U != nullptr && U->color == RED) { // Transformation_3: parent and uncle are RED
                P->color = BLACK;
                U->color = BLACK;
                G->color = RED;
                node = G; // new problem node
                continue; // Iteration
            }

            /*Transformation_4: P == RED , U != RED
             * node and P are different direction children ( one is a leftchild and the other a rightchild)
             */
            if (node == P->rightChild && (G == nullptr || P == G->leftChild)) { // Transformation_4R
                //node is right from parent, parent is left from grandparent
                rotate_left(P);
                //node becomes parent from P, both are now left children

                node = P; // new problem node
                P = node->parent; // new parent(red)

            } else if (node == P->leftChild && (G == nullptr || P == G->rightChild)) { // Transformation_4L
                //node is left from parent, parent is right from grandparent
                rotate_right(P);
                //node becomes parent from P, both are now right children

                node = P; // new problem node
                P = node->parent; // new parent(red)
            }

            /* Transformation_5: P == RED , U != RED,
             * node and P are both same direction child (both right or both left)
             */
            if (G != nullptr) {
                if (node == P->leftChild && P == G->leftChild) { // Transformation_5L
                    // node and P are left children
                    rotate_right(G);
                } else { // Transformation_5R
                    // node and P are right children
                    rotate_left(G);
                }

                G->color = RED;
            }
            P->color = BLACK;
        } //end while

    }
    m_size++;
    return true;
}

template<typename T, typename K>
void RBTree<T, K>::balanceTree(RBNode<T, K>* node) {
    /*
     * node : in first iteration -> deleted node (is black and has no children)
     *        other -> problem node
     * parent: is parent of node
     * sib: is sibling of node
     */

    /* the problem will at most move up to the root*/
    while (node->parent != nullptr) {
        /*
         * in each iteration sibling and parent have to be set to the actual value
         */
        RBNode<T, K>* sib = sibling(node);
        RBNode<T, K>* parent = node->parent;

        /*
         * case 1: sibling of problem node is RED
         */
        if (sib->color == RED) {
            if (parent->rightChild == sib) {
                rotate_left(parent);
            } else {
                rotate_right(parent);
            }
            sib->color = BLACK;
            parent->color = RED;
            sib = sibling(node);
        }
        /*
         * case 2: sibling of problem node is BLACK
         *         both children are BLACK or their are no children.
         */
        if (sib->color == BLACK && (sib->rightChild == nullptr || sib->rightChild->color == BLACK)
                && (sib->leftChild == nullptr || sib->leftChild->color == BLACK)) {
            node = parent;
            sib->color = RED;
            if (node->color == RED) {
                node->color = BLACK;
                return;
            } else {
                continue;
            }

        }
        /*
         * sibling is right child
         */
        if (parent->rightChild == sib) {
            /*
             * case 3: sibling of problem node is BLACK
             *         inner child is RED (left child)
             *         outer child is BLACK or does not exist (right child)
             */
            if (sib->color == BLACK && (sib->rightChild == nullptr || sib->rightChild->color == BLACK)
                    && (sib->leftChild != nullptr && sib->leftChild->color == RED)) {
                rotate_right(sib);
                sib->color = RED;
                sib = sibling(node);
                sib->color = BLACK;
            }
            /*
             * case 4: sibling of problem node is BLACK
             *         outer child is RED (right child)
             */
            if (sib->color == BLACK && (sib->rightChild != nullptr && sib->rightChild->color == RED)) {

                rotate_left(parent);

                grandparent(node)->color = parent->color;
                parent->color = BLACK;
                uncle(node)->color = BLACK;
                return;
            }
        } else {
            /*
             * sibling is left child
             */
            /*
             * case 3: sibling of problem node is BLACK
             *         inner child is RED (right child)
             *         outer child is BLACK or does not exist (left child)
             */
            if (sib->color == BLACK && (sib->rightChild != nullptr && sib->rightChild->color == RED)
                    && (sib->leftChild == nullptr || sib->leftChild->color == BLACK)) {
                rotate_left(sib);
                sib->color = RED;
                sib = sibling(node);
                sib->color = BLACK;
            }
            /*
             * case 4: sibling of problem node is BLACK
             *         outer child is RED (left child)
             */
            if (sib->color == BLACK && (sib->leftChild != nullptr && sib->leftChild->color == RED)) {
                rotate_right(parent);
                grandparent(node)->color = parent->color;
                parent->color = BLACK;
                uncle(node)->color = BLACK;
                return;
            }
        }
    }
}

template<typename T, typename K>
void RBTree<T, K>::remove(iterator& iter) {
    if (iter == end()) {
        return;
    }

    RBNode<T, K>* node = iter.node(), *rnode, *parent, *child;
    /*
     * preparation for one child
     */
    if (node->leftChild == nullptr) {
        rnode = node;
        child = rnode->rightChild;
    } else if (node->rightChild == nullptr) {
        rnode = node;
        child = rnode->leftChild;
    } else {
        /*
         * case 3 or case 6: node has two children
         */
        RBNode<T, K>* swapnode = findSwapNode(node);
        node->content = swapnode->content;
        node->key = swapnode->key;
        rnode = swapnode;
        child = rnode->rightChild; //single child can only be rightChild (swapnode cannot have a left child by definition)
    }

    /*
     * rnode points to the node that has to be removed
     */
    parent = rnode->parent;
    if (parent == nullptr) {
        /*
         * case 1 and case 2: root is single node or has one child
         */
        root = child; // child becomes new root
        if (child != nullptr) {
            child->parent = nullptr;
        }
        delete rnode;

    }
    /*
     * simple cases: rnode or child is RED and rnode is not root
     */
    else if (rnode->color == RED && parent != nullptr) {
        /*
         * case 4.1 and case 5.1: rnode is RED -> it can be removed easily.
         */

        if (rnode == parent->leftChild) {
            parent->leftChild = child;
        } else {
            parent->rightChild = child;
        }

        delete rnode;

    } else if (child == nullptr && rnode->color == BLACK && parent != nullptr) {
        /*
         * case 4.2: rnode is BLACK and child is nullptr
         */
        balanceTree(rnode);
        if (rnode == parent->leftChild) {
            parent->leftChild = nullptr;
        } else {
            parent->rightChild = nullptr;
        }

        delete rnode;
    } else if (child->color == RED && parent != nullptr && rnode->color == BLACK) {
        /*
         * case 5.2.1: rnode is BLACK and child is RED
         */
        child->color = BLACK;
        if (rnode == parent->leftChild) {
            parent->leftChild = child;
        } else {
            parent->rightChild = child;
        }
        child->parent = parent;
        delete rnode;
    } else {
        /*
         * case 5.2.2: rnode is BLACK and child is BLACK -> should not be possible to exist
         */
    }

    iter = end();
    m_size--;
}

template<typename T, typename K>
typename RBTree<T, K>::iterator RBTree<T, K>::find(K key) {
    RBNode<T, K>* current = root;

    while (current != nullptr) {
        if (current->key == key) {
            return RBTree<T, K>::iterator(this, current);
        } else if (key < current->key) {
            current = current->leftChild;
        } else {
            current = current->rightChild;
        }
    }
    return end();
}

template<typename T, typename K>
RBNode<T, K>* RBTree<T, K>::grandparent(RBNode<T, K>* x) {
    if (x == nullptr || x->parent == nullptr) {
        return nullptr;
    }
    return x->parent->parent;
}

template<typename T, typename K>
RBNode<T, K>* RBTree<T, K>::uncle(RBNode<T, K>* x) {
    if (grandparent(x) == nullptr) {
        return nullptr;
    }
    if (x->parent == grandparent(x)->leftChild) {
        return grandparent(x)->rightChild;
    } else {
        return grandparent(x)->leftChild;
    }
}

template<typename T, typename K>
RBNode<T, K>* RBTree<T, K>::sibling(RBNode<T, K>* x) {
    if (x->parent == nullptr) {
        return nullptr;
    }
    if (x == x->parent->leftChild) {
        return x->parent->rightChild;
    } else {
        return x->parent->leftChild;
    }
}

template<typename T, typename K>
RBNode<T, K>* RBTree<T, K>::findSwapNode(RBNode<T, K>* x) {
    /*
     * find node with smallest key in right subtree of x
     */

    RBNode<T, K>* node = x->rightChild;
    while (node->leftChild != nullptr) {
        node = node->leftChild;
    }
    return node;
}

template<typename T, typename K>
typename RBTree<T, K>::tree_size_t RBTree<T, K>::size() const {
    return m_size;
}

template<typename T, typename K>
void RBTree<T, K>::rotate_right(RBNode<T, K>* x) {
    RBNode<T, K>* leftchild, *rightgrandchild, *parent;

    leftchild = x->leftChild;
    rightgrandchild = leftchild->rightChild;
    parent = x->parent;

    if (parent == nullptr) {
        root = leftchild;
        leftchild->parent = nullptr;
        x->parent = leftchild;
    } else {
        if (parent->rightChild == x) {
            parent->rightChild = leftchild;
            leftchild->parent = parent;
        } else {
            parent->leftChild = leftchild;
            leftchild->parent = parent;
        }
    }
    leftchild->rightChild = x;
    x->parent = leftchild;

    x->leftChild = rightgrandchild;
    if (rightgrandchild != nullptr) {
        rightgrandchild->parent = x;
    }
}

template<typename T, typename K>
void RBTree<T, K>::rotate_left(RBNode<T, K>* x) {
    RBNode<T, K>* rightchild, *leftgrandchild, *parent;
    rightchild = x->rightChild;
    leftgrandchild = rightchild->leftChild;
    parent = x->parent;

    if (parent == nullptr) {
        root = rightchild;
        rightchild->parent = nullptr;
        x->parent = rightchild;
    } else {
        if (parent->leftChild == x) {
            parent->leftChild = rightchild;
            rightchild->parent = parent;
        } else {
            parent->rightChild = rightchild;
            rightchild->parent = parent;
        }
    }

    rightchild->leftChild = x;
    x->parent = rightchild;

    x->rightChild = leftgrandchild;
    if (leftgrandchild != nullptr) {
        leftgrandchild->parent = x;
    }
}

}
#endif /* RBTREE_H_ */
