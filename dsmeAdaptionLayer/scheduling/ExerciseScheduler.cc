#include "ExerciseScheduler.h"
#include "../../../dsme_platform.h"
#include "../DSMEAdaptionLayer.h"
#include "../../mac_services/pib/MAC_PIB.h"
#include "../../dsmeLayer/DSMELayer.h"
#include "../../dsmeLayer/messageDispatcher/MessageDispatcher.h"
#include "../../dsmeLayer/neighbors/NeighborQueue.h"

namespace dsme {

ExerciseScheduler::ExerciseScheduler(DSMEAdaptionLayer& dsmeAdaptionLayer) : GTSSchedulingImpl(dsmeAdaptionLayer) {
}

ExerciseScheduler::~ExerciseScheduler() {
}

void ExerciseScheduler::multisuperframeEvent() {
    uint16_t allocatedTxGTSTotal = 0;
    for(auto &data : this->txLinks) {
        allocatedTxGTSTotal += dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX);
    }
    uint16_t allocatedRxGTSTotal = 0;
    for(auto &data : this->rxLinks) {
        allocatedRxGTSTotal += dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::RX);
    }
    uint16_t neighborhoodGTS = 0;
    for(abs_slot_idx_t slot=0; slot<dsmeAdaptionLayer.getMAC_PIB().helper.getNumberGTSlotsPerMultisuperframe() * dsmeAdaptionLayer.getMAC_PIB().helper.getNumChannels(); slot++) {
        if(dsmeAdaptionLayer.getMAC_PIB().macDSMESAB.isOccupied(slot)) {
            neighborhoodGTS++;
        }
    }
    uint8_t numChildren = 0;
    for(auto &data : rxLinks) {
        if(data.messagesRxLastMultisuperframe != 0) {
            numChildren++;
        }
    }
    uint16_t queueLevel = 0;
    NeighborQueue<MAX_NEIGHBORS> &queue = dsmeAdaptionLayer.getDSME().getMessageDispatcher().getNeighborQueue();
    for(NeighborQueue<MAX_NEIGHBORS>::iterator it = queue.begin(); it != queue.end(); ++it) {
        queueLevel += it->queueSize;
    }

    for(auto &data : this->txLinks) {
        /* Collect data for outgoing link (there is only one parent */
        uint16_t incomingPackets = data.messagesInLastMultisuperframe;
        uint16_t outgoingPackets = data.messagesOutLastMultisuperframe;
        uint16_t allocatedTxGTS = dsmeAdaptionLayer.getMAC_PIB().macDSMEACT.getNumAllocatedGTS(data.address, Direction::TX);
        int16_t currentTarget = data.slotTarget;

        bool isLeaf = allocatedRxGTSTotal == 0;

        /* Update GTS target */
        data.slotTarget = getTargetGTS(incomingPackets, outgoingPackets, allocatedTxGTS, allocatedTxGTSTotal, allocatedRxGTSTotal, neighborhoodGTS, currentTarget, queueLevel, numChildren, isLeaf);
        data.messagesInLastMultisuperframe = 0;
        data.messagesOutLastMultisuperframe = 0;
    }
}

int16_t ExerciseScheduler::getTargetGTS(uint16_t incomingPackets, uint16_t outgoingPackets, uint16_t allocatedTxGTS, uint16_t allocatedTxGTSTotal, uint16_t allocatedRxGTSTotal, uint16_t neighborhoodGTS, int16_t currentTarget, uint16_t queueLevel, uint8_t numChildren, bool isLeaf) {
    LOG_INFO("incoming Packets: " << (int)incomingPackets << " outgoingPackets: " << (int)outgoingPackets << " allocatedTxGTS: " << (int)allocatedTxGTS << " allocatedTxGTSTotal: " << (int)allocatedTxGTSTotal << " allocatedRxGTSTotal: " << (int)allocatedRxGTSTotal << " neighborhoodGTS: " << (int)neighborhoodGTS << " currentTarget: " << (int)currentTarget << " queueLevel: " << queueLevel << " numChildren: " << (int)numChildren << " isLeaf: " << isLeaf);

    // TODO: remove the current implementation and implement your own solution here!

    if(incomingPackets > 2) {
        return 3;
    } else if(incomingPackets > 0) {
        return 1;
    } else {
        return 0;
    }
}

} /* NAMESPACE DSME */
