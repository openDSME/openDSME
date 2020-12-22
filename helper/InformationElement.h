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
            ID_stopIE = 0xFF    //signal the end of the ieList
        }tIEID;
        virtual ~InformationElement(){};
        virtual uint8_t getSerializationLength() = 0;
        virtual void serialize(Serializer& serializer) = 0;
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

        void serialize(Serializer& serializer) {
            serializer << isLastMessage;
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

        void serialize(Serializer& serializer) {
            serializer << gackEnabled;
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

        void serialize(Serializer& serializer) {
            if(serializer.getType() == SERIALIZATION){
                serializer << superframeID;
                serializer << slotID;
                serializer << channelIndex;
            }
            else{   //DESERIALIZATION
                serializer << superframeID;
                serializer << slotID;
                serializer << channelIndex;
            }
        }
    };
}




#endif /* SRC_OPENDSME_HELPER_INFORMATIONELEMENT_H_ */
