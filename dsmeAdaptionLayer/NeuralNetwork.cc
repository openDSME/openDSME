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

const float l0_weights_array[72] {
    -0.450517,0.223372,0.398406,0.146447,-0.315537,-0.044455,0.60127,-0.292425,0.335024,
    -0.317687,-0.338795,-0.140864,0.0339777,-0.0182826,-0.414133,0.157259,0.181194,-0.285436,
    0.320784,0.222715,-0.120864,0.215301,-0.0462402,0.0145642,0.186758,0.184367,-0.236478,
    -0.197822,0.347526,-0.006736,0.26074,-0.106376,0.0368646,0.419383,0.235721,-0.324143,
    0.935083,0.71171,-0.216251,0.0267075,0.509393,0.50644,0.218649,0.479253,-0.0400003,
    -0.32155,-0.142574,-0.128784,0.0578439,0.227126,0.200449,0.295902,0.277997,-0.528827,
    -0.315967,0.071536,-0.23378,0.0211989,0.292715,-0.136696,-0.00523956,-0.128007,-0.183468,
    -0.680562,-0.22755,-0.176197,-0.0601504,0.29819,-0.410372,-0.0878465,-0.167067,-0.00155065,
};
float l0_bias_array[8] {
    -0.106991,-0.435861,0.0335165,0.038877,0.00623968,0.210107,-0.363641,-0.0831152,
};
float l0_output_array[8];

const float l1_weights_array[40] {
    -0.49478,0.337746,-0.0225481,-0.148849,-0.0988261,-0.317815,0.0638509,-0.13692,
    -0.472525,0.133359,0.294997,-0.281505,-0.418212,-0.298649,0.193862,-0.16782,
    -0.485538,-0.173324,0.565774,-0.304641,-0.630479,-0.25851,-0.307558,0.559412,
    -0.586629,0.244488,-0.588756,0.137831,0.455256,-0.350996,-0.0364198,0.110441,
    -0.478167,0.22436,-0.474468,-0.0582494,0.224412,-0.236253,0.181508,-0.127365,
};
float l1_bias_array[5] {
    -0.172017,-0.114134,0.0407057,-0.0450892,0.0621143,
};
float l1_output_array[5];

const quicknet::matrix_t l0_weights{8,9,l0_weights_array};
const quicknet::vector_t l0_bias{8,l0_bias_array};
quicknet::vector_t l0_output{8,l0_output_array};

const quicknet::matrix_t l1_weights{5,8,l1_weights_array};
const quicknet::vector_t l1_bias{5,l1_bias_array};
quicknet::vector_t l1_output{5,l1_output_array};

quicknet::Layer layers[2] {
    {l0_weights, l0_bias, l0_output, quicknet::quick_tanh},
    {l1_weights, l1_bias, l1_output, quicknet::quick_softmax},
};

NeuralNetwork::NeuralNetwork() : network{2, layers} {
}

} /* namespace dsme */
