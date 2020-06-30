/*
 * IEQueue.h
 *
 *  Created on: 29.06.2020
 *      Author: cjd8627
 */

#ifndef SRC_OPENDSME_HELPER_IEQUEUE_H_
#define SRC_OPENDSME_HELPER_IEQUEUE_H_

namespace dsme {

    /* IEQueue - Stores Information Elements */
    template <uint16_t MAX_SIZE>
    class IEQueue {
    public:
        IEQueue() : queue{}, next_back(0), size(0) {
        }

        ~ IEQueue(){
            for(int i = 0; i < MAX_SIZE; i++){
                //delete queue[i]; //TODO: virtual destructor
            }
        }

        // assumes queue is not full
        template <typename T>
        void push(T element) {

            queue[next_back] = new T(element);
            if(next_back == 0) {
                next_back = MAX_SIZE - 1;
            } else {
                next_back--;
            }
            size++;
        }

        // assumes queue is not empty
        void pop() {
            size--;
        }

        // assumes queue is not empty
        InformationElement& front() {
            return queue[(next_back + size) % MAX_SIZE];
        }

        bool empty() const {
            return (size == 0);
        }

        bool full() const {
            return (size >= 10);
        }

        int getSize() const {
            return size;
        }

        bool getIEByID(uint8_t ieID, InformationElement*& iePointer){
            for(int i = 0; i < size; i++){
                //LOG_INFO(queue[0]->getIEID());
                if(queue[i]->getIEID() == ieID){
                    iePointer = queue[i];
                    return true;
                }
            }
            return false;
        }

        uint8_t* parse(){
            otherIE oIE;
            oIE.someFlag = true;
            this->push(oIE);

            uint8_t* buffer = new uint8_t[size];
            uint8_t* buf = buffer;

            for(InformationElement* iePointer: queue){
                switch(iePointer->getIEID()){
                    case 16:{
                        lastMessageIE* lMIE = dynamic_cast<lastMessageIE*>(iePointer);
                        *buf = lMIE->parse();
                        break;
                    }
                    case 11:{
                        otherIE* oIE = dynamic_cast<otherIE*>(iePointer);
                        *buf = oIE->parse();
                        break;
                    }
                }
                buf++;
            }
            return buffer;
        }

        void unparse(const uint8_t* pBuf, const uint8_t* buffer, uint8_t payloadLength){
            while(buffer-pBuf <= payloadLength){ //TODO dynamic size
                switch((*buffer)/10){
                    case 16:{
                        lastMessageIE lmIE;// wie lange existiert dieses Object
                        lmIE.isLastMessage = (*buffer)%10;
                        this->push(lmIE);
                        break;
                    }
                    case 11:{
                        otherIE oIE;
                        oIE.someFlag = (*buffer)%10;
                        this->push(oIE);
                        break;
                    }
                }
                buffer++;
            }
        }

    private:
        InformationElement* queue[MAX_SIZE];
        uint16_t next_back;
        uint16_t size;
    };
}





#endif /* SRC_OPENDSME_HELPER_IEQUEUE_H_ */
