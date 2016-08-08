#ifndef COMMUNICATION
#define COMMUNICATION

#include "Usb.h"
#include "Clock.h"
#include <stdio.h>
#include <stdarg.h>
#include <cstring>

#define CONNECTED 0x0
#define DISCONNECT_REQ 0xFF
#define MAX_PAYLOAD 62

#define MSG_LOG 0x1
#define MSG_BYTES 0x2
#define MSG_BYTES_END 0x3
#define MSG_KINECT_DATA 0x4
#define MSG_IMAGE_RECOGNITION 0x5
#define MSG_LOCATION 0x6

#define DEBUGMODE true

namespace medina {
    using namespace ecrobot;
    
    class Communication {
        unsigned char data[Usb::MAX_USB_DATA_LENGTH];
        unsigned char dataReceive[Usb::MAX_USB_DATA_LENGTH];
        Usb* usb;
        Clock* clock;
        void (*kinectDataCallback)(unsigned short *data);
        void (*imageRecognitionCallback)(unsigned short *data);

    public:
        bool lock;

        Communication(Usb* _usb, Clock* _clock) {
            usb     = _usb;
            clock   = _clock;
            lock = false;
        }

        void setKinectDataCallback(void (*cb)(unsigned short *data)){
            kinectDataCallback = cb;
        }

        void setImageRecognitionCallback(void (*cb)(unsigned short *data)) {
            imageRecognitionCallback = cb;
        }

        void log(char *msg) {
            int len = strlen(msg) + 2;

            if (len > MAX_PAYLOAD){
                len = MAX_PAYLOAD;
            }

            for (int i = 2; i < len; i++){
                data[i] = msg[i-2];
            }

            send(len, CONNECTED, MSG_LOG);
        }

        void logFormat(char *format, ...) {
            char buf[MAX_PAYLOAD];

            va_list args;
            va_start(args, format);
            vsnprintf(buf, MAX_PAYLOAD, format, args);
            va_end(args);

            log(buf);
        }

        void debug(char *format, ...) {
    #ifdef DEBUGMODE
            char buf[MAX_PAYLOAD];

            va_list args;
            va_start(args, format);
            vsnprintf(buf, MAX_PAYLOAD, format, args);
            va_end(args);

            log(buf);
    #endif
        }

        void ImageRecognition() {
            send(0, CONNECTED, MSG_IMAGE_RECOGNITION);
        }

        void sendBytes(int length, unsigned char *bytes){
            int idx = 2;
            for (int i = 0; i < length; i++){
                data[idx++] = bytes[i];

                if (idx == MAX_PAYLOAD+2){
                    send(MAX_PAYLOAD, CONNECTED, MSG_BYTES);
                    idx = 2;
                }
            }
            send(length % MAX_PAYLOAD, CONNECTED, MSG_BYTES_END);
        }

        void sendLocation(double x, double y, double theta){
            logFormat("Loc%d|%d|%d", (int)x, (int)y, (int)theta);
        }

        void send(int len, unsigned char cbyte1, unsigned char cbyte2) {
            data[0] = cbyte1;
            data[1] = cbyte2;

            usb->send(data, 0, len+2); //control bytes + payload
            memset(data, 0, 64);
            clock->wait(1);
        }

        void handler() {
            usb->commHandler();

            if (usb->isConnected()) {
                memset(dataReceive, 0, Usb::MAX_USB_DATA_LENGTH);
                int len = usb->receive(dataReceive, 0, Usb::MAX_USB_DATA_LENGTH);

                if (len > 0){
                    if (dataReceive[0] == DISCONNECT_REQ ){
                        usb->close();
                    } else {
                        handleMessage();
                    }
                }
            }
        }

        void handleMessage(){
            if (dataReceive[1] == MSG_KINECT_DATA){
                unsigned short reading[30];

                int idx = 3;
                for (int i = 0; i < 30; i++){
                    reading[i] = (dataReceive[idx] << 8) | dataReceive[idx - 1];
                    idx += 2;
                }

                if (kinectDataCallback != NULL){
                    kinectDataCallback(reading);
                }
            } else if (dataReceive[1] == MSG_IMAGE_RECOGNITION) {
                unsigned short reading[4];

                reading[0] = dataReceive[2];
                reading[1] = dataReceive[3];
                reading[2] = dataReceive[4];
                reading[3] = dataReceive[5];

                if (imageRecognitionCallback != NULL){
                    imageRecognitionCallback(reading);
                }
            }
        }
    };
}
#endif
