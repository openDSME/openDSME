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

#ifndef QUICKNET__SPARSEMATRIX_H_
#define QUICKNET__SPARSEMATRIX_H_

#include "../../helper/Integers.h"
#include "Matrix.h"

namespace dsme {

namespace quicknet {

template <typename T>
class SparseMatrix : public Matrix{
public:
    SparseMatrix(uint8_t n, uint8_t m, const T* sparse_matrix, const uint8_t* index_rows, const uint8_t* index_columns, const float* dictionary) : Matrix(n, m, sparse_matrix), index_rows(index_rows), index_rolumns(index_columns), dictionary(dictionary) {
    }

    SparseMatrix(const SparseMatrix&) = delete;
    SparseMatrix& operator=(const SparseMatrix&) = delete;
   
    virtual void mult(const Vector<T> &input, Vector<T> &output) const override {
        DSME_ASSERT(input.length() == this->columns());
        DSME_ASSERT(output.length() == this->rows());
       
        uint8_t entry_index = 0;
        for(int i=0; i<this->rows(); i++) {
            output(i) = 0;
            for(int j=0; j<index_rows[i]; j++) {
                 uint8_t col = index_columns[entry_index++];
                 output(i) += input(col) * dictionary[(*this)(i, col)];
            }
        }
    }

private:
    const uint8_t* index_rows;
    const uint8_t* index_columns;
    const float* dictionary;
};

} /* namespace quicknet */

} /* namespace dsme */

#endif /* QUICKNET__MATRIX_H_ */
