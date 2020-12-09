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
        typedef enum{
            ID_lastMessage = 0x10,
            ID_gackEnabled = 0x0B
        }tIEID;
        virtual ~InformationElement(){};
        virtual uint8_t parse() = 0;
        virtual uint8_t getIEID() = 0;

     };

    class lastMessageIE : public InformationElement{
        public:
        lastMessageIE(){
            IEID = ID_lastMessage;
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

    class gackEnabledIE : public InformationElement{
        public:
        gackEnabledIE(){
            IEID = ID_gackEnabled;
        }
        ~gackEnabledIE(){};

        bool gackEnabled;

        uint8_t getIEID(){
            return IEID;
        }

        uint8_t parse(){
            return IEID*0xA + gackEnabled;
        }
    };
}




#endif /* SRC_OPENDSME_HELPER_INFORMATIONELEMENT_H_ */
