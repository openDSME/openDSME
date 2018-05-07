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

/******** Layer 0 ********/
static const float l0_weights_array[18] {
    23.71422005,43.46171188,-1.00180578,
    -6.62341976,12.16881752,-0.75082558,
    21.91276550,46.73196793,-1.89223683,
    -13.43641281,-27.20864677,1.47875011,
    24.51018715,42.33295822,-0.86048412,
    18.70379257,44.69678116,1.64947331,
};
static float l0_bias_array[6] {
    -0.75947106,0.51447004,-0.20190984,0.65793115,-0.74265754,-0.80352694
};
static float l0_output_array[6];

static const quicknet::matrix_t l0_weights{6,3,l0_weights_array};
static const quicknet::vector_t l0_bias{6,l0_bias_array};
static quicknet::vector_t l0_output{6,l0_output_array};

/******** Layer 1 ********/
static const float l1_weights_array[36] {
    3.39464617,-2.28247452,1.95771539,-3.34992075,3.62503362,3.71364498,
    -1.76891410,8.26901722,-0.66424388,61.31443024,-2.66976857,-1.04543591,
    -3.84625721,0.86596709,-4.38588858,4.43534994,-3.52879643,0.38089052,
    -5.45610809,1.15100110,-4.76177168,4.59681511,-6.27993202,-3.05812120,
    1.73447740,-1.18236852,2.33071446,-5.10598993,1.90438628,2.42812824,
    0.75718325,-8.65610313,0.56389427,0.92049515,0.90366000,1.49533117,
};
static float l1_bias_array[6] {
    -1.18388128,0.12728578,1.18040287,1.05579531,-2.65120244,0.09804524
};
static float l1_output_array[6];

static const quicknet::matrix_t l1_weights{6,6,l1_weights_array};
static const quicknet::vector_t l1_bias{6,l1_bias_array};
static quicknet::vector_t l1_output{6,l1_output_array};

/******** Layer 2 ********/
static const float l2_weights_array[90] {
    -2.27528954,0.24825554,0.78908598,3.53707004,-4.03183222,-2.78524017,
    -4.17542171,-3.88955140,-4.14152145,-2.51975083,-3.58299208,-3.58818245,
    1.26015747,-2.65824103,-1.34168243,2.78072143,-3.44436812,5.16382408,
    -0.91588742,1.92417538,0.46536046,-0.74839479,3.07085323,-0.11561805,
    -0.43642575,2.84873867,-1.69130683,0.74529189,2.30446982,-6.47426367,
    0.72583139,-1.88004708,1.20180547,-1.73153019,-1.11492610,4.74400711,
    2.83726931,-0.70544374,-1.22112548,1.44628417,0.00383955,-4.97048569,
    -1.42871618,0.00397724,2.53941751,-3.28082371,-0.64269245,6.02133036,
    -0.33895171,0.17330514,1.84662688,-13.97587967,-1.29445004,4.20684862,
    -0.40321630,1.47767448,-0.53633165,-1.96899188,0.59190160,-3.21406412,
    0.52106434,-0.19413270,0.17836581,0.93801159,-0.44804171,-2.96686935,
    0.11582860,-0.10389264,-1.82505906,-26.95653915,0.35318574,-3.32655025,
    1.06097877,-0.16255935,-4.80677700,-42.56092453,-10.31240368,-3.98332429,
    1.99004626,-1.54289675,1.03176117,3.70165396,-1.06923020,-2.46676373,
    -4.17017460,-4.24553680,-3.99468398,-2.46624756,-3.79183698,-3.42810369,
};
static float l2_bias_array[15] {
    -1.34867001,-4.32959700,1.18626189,-0.68444717,0.79860026,0.49032336,0.60856497,-0.72087979,-0.81589717,-1.34443295,0.94470894,-1.43287230,-0.66324842,0.51277763,-4.07175922
};
static float l2_output_array[15];

static const quicknet::matrix_t l2_weights{15,6,l2_weights_array};
static const quicknet::vector_t l2_bias{15,l2_bias_array};
static quicknet::vector_t l2_output{15,l2_output_array};

/******** Network ********/
static quicknet::Layer layers[3] {
    {l0_weights, l0_bias, l0_output, quicknet::quick_sigmoid},
    {l1_weights, l1_bias, l1_output, quicknet::quick_sigmoid},
    {l2_weights, l2_bias, l2_output, quicknet::quick_softmax},
};

NeuralNetwork::NeuralNetwork() : network{3, layers} {
}

} /* namespace dsme */
