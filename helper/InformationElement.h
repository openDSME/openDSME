/*
 * InformationElement.h
 *
 *  Created on: 29.06.2020
 *      Author: cjd8627
 */

#ifndef SRC_OPENDSME_HELPER_INFORMATIONELEMENT_H_
#define SRC_OPENDSME_HELPER_INFORMATIONELEMENT_H_

namespace dsme {
    class InformationElement {
        public:
        typedef enum{
            ID_lastMessage = 0x10,
            ID_gackEnabled = 0x0B,
            ID_gackResponse = 0x0C,
            ID_gack = 0x0D,
            ID_stopIE = 0xFF    //signal the end of the ieList
        }tIEID;
        virtual ~InformationElement(){};
        virtual uint8_t getSerializationLength() = 0;
        virtual void serializeTo(uint8_t*& buffer) = 0;
        virtual void deserializeFrom(const uint8_t*& buffer, uint8_t payloadLength) = 0;
        virtual InformationElement::tIEID getIEID() = 0;
        protected:
        InformationElement::tIEID IEID;
     };

    class lastMessageIE : public InformationElement{
        public:
        lastMessageIE(){
            IEID = ID_lastMessage;
        }
        ~lastMessageIE(){};

        uint8_t isLastMessage;

        InformationElement::tIEID getIEID(){
            return IEID;
        }

        uint8_t getSerializationLength() {
            return 1;
        }

        void serializeTo(uint8_t*& buffer) {
            *(buffer++) = isLastMessage;
        }
        void deserializeFrom(const uint8_t*& buffer, uint8_t payloadLength) {
            isLastMessage = *(buffer++);
        }
    };
    class gackIE : public InformationElement{
        public:
        gackIE(){
            IEID = ID_gack;
        }
        ~gackIE(){};

        InformationElement::tIEID getIEID(){
            return IEID;
        }

        uint8_t getSerializationLength() {
            return 0;
        }

        void serializeTo(uint8_t*& buffer) {
        }
        void deserializeFrom(const uint8_t*& buffer, uint8_t payloadLength) {
        }
    };

    class gackEnabledIE : public InformationElement{
        public:
        gackEnabledIE(){
            IEID = ID_gackEnabled;
        }
        ~gackEnabledIE(){};

        uint8_t gackEnabled;

        InformationElement::tIEID getIEID(){
            return IEID;
        }

        uint8_t getSerializationLength() {
            return 1;
        }

        void serializeTo(uint8_t*& buffer) {
            *(buffer++) = gackEnabled;
        }
        void deserializeFrom(const uint8_t*& buffer, uint8_t payloadLength) {
            gackEnabled = *(buffer++);
        }
    };
    class gackResponseIE : public InformationElement{
        public:
        gackResponseIE(){
            IEID = ID_gackResponse;
        }
        ~gackResponseIE(){};

        uint16_t superframeID;
        uint8_t slotID;
        uint8_t channelIndex;

        InformationElement::tIEID getIEID(){
            return IEID;
        }

        uint8_t getSerializationLength() {
            return 4;
        }

        void serializeTo(uint8_t*& buffer) {
            *(buffer++) = superframeID & 0xFF;  //LSB first?
            *(buffer++) = superframeID >> 8;
            *(buffer++) = slotID;
            *(buffer++) = channelIndex;
        }
        void deserializeFrom(const uint8_t*& buffer, uint8_t payloadLength) {
            superframeID = *(buffer) | (*(buffer + 1) << 8);
            buffer += 2;
            slotID = *(buffer++);
            channelIndex = *(buffer++);
        }
    };
}




#endif /* SRC_OPENDSME_HELPER_INFORMATIONELEMENT_H_ */
