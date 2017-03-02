/*
 * openDSME
 *
 * Implementation of the Deterministic & Synchronous Multi-channel Extension (DSME)
 * described in the IEEE 802.15.4-2015 standard
 *
 * Authors: Florian Kauer <florian.kauer@tuhh.de>
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

#include "./NeuralNetwork.h"

namespace dsme {

quicknet::Layer layers[3];

float l0_weights_array[36]{
    -0.632057, 0.298791,  0.409173,   0.075295,  0.310842,  0.609959,  -0.634336,  -0.0846814, -0.100855, -0.497546, 0.0201811, 0.296736,
    0.332275,  -0.487201, -0.0108817, 0.317351,  -0.30346,  -0.432407, -0.797769,  -0.54716,   0.0803563, 0.124381,  -0.698296, -0.194392,
    0.264011,  -0.260764, -0.423058,  -0.741561, -0.597822, 0.221788,  0.00325142, -0.36108,   -0.289276, -0.369159, -0.428123, 0.19941,
};
float l0_bias_array[4]{
    0.0935064, -0.243025, 0.0809402, -0.165077,
};
float l0_output_array[4];

float l1_weights_array[16]{
    0.646545, -0.199343, -0.0870448, 0.622228, -0.975576, 0.420801, -0.829461, 0.0912061,
    0.70085,  -0.447854, 0.325059,   0.181885, 0.640156,  0.206571, -0.710753, 0.691096,
};
float l1_bias_array[4]{
    0.11498, 0.0786337, 0.0801584, -0.0076172,
};
float l1_output_array[4];

float l2_weights_array[20]{
    0.532168,  0.29164,    -0.329229, 0.678553, -0.593356, -0.54416, -0.0164168, 0.284665, 0.668048, -0.173169,
    -0.168476, -0.0815055, 0.32422,   0.233954, 0.106146,  0.232341, 0.0855617,  0.257652, 0.142231, 0.567573,
};
float l2_bias_array[5]{
    -0.0961998, -0.108499, 0.114981, 0.0774459, 0.07548,
};
float l2_output_array[5];

NeuralNetwork::NeuralNetwork() : network{3, layers} {
    const quicknet::matrix_t l0_weights{4, 9, l0_weights_array};
    const quicknet::vector_t l0_bias{4, l0_bias_array};
    quicknet::vector_t l0_output{4, l0_output_array};

    layers[0] = quicknet::Layer{l0_weights, l0_bias, l0_output};
    const quicknet::matrix_t l1_weights{4, 4, l1_weights_array};
    const quicknet::vector_t l1_bias{4, l1_bias_array};
    quicknet::vector_t l1_output{4, l1_output_array};

    layers[1] = quicknet::Layer{l1_weights, l1_bias, l1_output};
    const quicknet::matrix_t l2_weights{5, 4, l2_weights_array};
    const quicknet::vector_t l2_bias{5, l2_bias_array};
    quicknet::vector_t l2_output{5, l2_output_array};

    layers[2] = quicknet::Layer{l2_weights, l2_bias, l2_output};
}

const quicknet::vector_t& NeuralNetwork::feedForward(const quicknet::vector_t& input) {
    return this->network.feedForward(input);
}

} /* namespace dsme */
