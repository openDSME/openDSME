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
    10.67579842,14.41761112,3.66878486,
    -8.57790470,3.44429207,-3.90031433,
    8.75284767,15.30404758,-12.20483494,
    11.63218498,-3.36312747,4.98062086,
    -5.10256863,3.43544555,10.33296394,
    -8.59236813,12.94239807,9.77632332,
};
static float l0_bias_array[6] {
    -1.35567451,5.69110107,1.47404552,-3.37760758,-4.49701023,-4.43283749
};
static float l0_output_array[6];

static const quicknet::matrix_t l0_weights{6,3,l0_weights_array};
static const quicknet::vector_t l0_bias{6,l0_bias_array};
static quicknet::vector_t l0_output{6,l0_output_array};

/******** Layer 1 ********/
static const float l1_weights_array[36] {
    -0.17654309,8.01893330,10.49141979,-12.95424652,-12.58387089,-14.12204456,
    -1.15515184,-5.70966244,5.57141876,-1.16047192,-3.41033363,0.90216196,
    -1.81711912,10.79940987,-8.12032318,-5.63724422,-6.65560389,-0.50096434,
    2.41587520,5.07091188,3.92447686,2.53248525,2.63841438,2.62378407,
    6.01092100,-3.09925246,4.76126337,2.02658701,-2.54710007,0.05662666,
    3.41825581,-3.07007909,-2.83041954,9.00438118,5.87148285,7.21780825,
};
static float l1_bias_array[6] {
    4.40763474,0.59238428,0.64939916,5.50730753,-1.44691443,-0.97783774
};
static float l1_output_array[6];

static const quicknet::matrix_t l1_weights{6,6,l1_weights_array};
static const quicknet::vector_t l1_bias{6,l1_bias_array};
static quicknet::vector_t l1_output{6,l1_output_array};

/******** Layer 2 ********/
static const float l2_weights_array[90] {
    -2.52200317,-2.97171092,-3.16518569,-3.95151567,-3.39345074,-2.24032140,
    -3.14097118,-2.91351223,-2.93317866,-3.26441979,-3.18347526,-3.05826235,
    -0.01700409,8.14629650,2.98906207,-2.84948707,0.65529871,3.33597040,
    1.88070560,-1.71951747,-3.41240978,1.65982044,0.12373909,0.89318669,
    1.00983214,-1.33241284,-0.36179221,0.66349822,0.20547332,0.79255873,
    2.11156702,5.42778254,-0.65257096,0.25903031,-1.04275262,1.52343237,
    0.51017380,1.45265567,-0.97160596,0.54708546,-0.09896544,0.88322932,
    0.80904967,-0.32724255,-2.44006228,1.45905221,-1.67997873,0.05091684,
    1.04872906,-12.95839500,-0.18879506,-0.65998071,0.93258476,-0.00873115,
    -5.15279388,-0.59830260,3.67988825,1.80841267,-1.93297780,-6.58922577,
    -1.82381916,-9.94235420,2.58325815,-0.36447331,3.26854563,-3.35276914,
    -2.38544440,-2.98394108,-3.23110390,-3.77189040,-2.71454620,-3.01014137,
    -2.73589396,-3.30587220,-3.08886671,-3.31957078,-3.41454935,-2.70860195,
    -1.42137730,-7.93667173,3.97891855,-0.56252110,1.90796900,-2.96585011,
    -2.79875326,-2.98749661,-3.20551395,-3.88737226,-3.46720648,-2.26964211,
};
static float l2_bias_array[15] {
    -3.02682924,-3.08831835,-3.26206398,1.15282047,0.12751885,-0.17609327,0.93073159,1.93010402,-0.30392972,1.55034912,-0.61569983,-2.98889518,-3.17693377,-0.94497722,-2.99393773
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
