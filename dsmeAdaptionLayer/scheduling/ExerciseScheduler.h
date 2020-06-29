#ifndef EXERCISESCHEDULER_H_
#define EXERCISESCHEDULER_H_

#include "./GTSScheduling.h"

namespace dsme {

class DSMEAdaptionLayer;

class ExerciseScheduler : public GTSSchedulingImpl<GTSSchedulingData, GTSRxData> {
public:
    ExerciseScheduler(DSMEAdaptionLayer& dsmeAdaptionLayer);
    virtual ~ExerciseScheduler();

    /* Collect data for scheduling every MSF */
    virtual void multisuperframeEvent() override;

    /* Calculate the number of required GTS per node
     *
     * incomingPackets - number of incoming (received + generated) packets at the node per MSF
     * outgoingPackets - number of outgoing (transmitted + lost) packets at the node per MSF
     * currentTarget - current target for the number of allocated Tx-GTS
     * queueLevel - level of the queue (in packets)
     * allocatedTxGTS - number of allocated GTS for transmission
     * allocatedRxGTS - number of allocatd GTS for reception
     * numChildren - number of children
     * isLeaf - true if there are no children, false otherwise (WARNING: This changes during the network setup)
     * neighborhoodGTS - total number of GTS allocated by nodes in the neighborhood
     *
     * return: total number of GTS to allocate at this node (per MSF)
     *
     * IF YOUR NEED MORE DATA JUST ASK! :)
     * */
    int16_t getTargetGTS(uint16_t incomingPackets, uint16_t outgoingPackets, uint16_t allocatedTxGTS, uint16_t allocatedTxGTSTotal,  uint16_t allocatedRxGTSTotal, uint16_t neighborhoodGTS, int16_t currentTarget, uint16_t queueLevel, uint8_t numChildren, bool isLeaf);
};

}; /* NAMESPACE DSME */

#endif /* EXERCISESCHEDULER_H_ */
