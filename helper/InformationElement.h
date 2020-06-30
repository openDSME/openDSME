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
        protected:
        uint8_t IEID;
        public:
        virtual ~InformationElement() = default;
        virtual uint8_t parse() = 0;
        virtual uint8_t getIEID(){return 10;};

     };

    class lastMessageIE : public InformationElement{
        public:
        lastMessageIE(){
            IEID = 0x10;
        }
        ~lastMessageIE(){};

        bool isLastMessage;

        uint8_t getIEID(){
            return IEID;
        }

        uint8_t parse(){
            return IEID*0xA + isLastMessage;
        }
    };

    class otherIE : public InformationElement{
        public:
        otherIE(){
            IEID = 0xB;
        }
        ~otherIE(){};

        bool someFlag;

        uint8_t getIEID(){
            return IEID;
        }

        uint8_t parse(){
            return IEID*0xA + someFlag;
        }
    };
}




#endif /* SRC_OPENDSME_HELPER_INFORMATIONELEMENT_H_ */
