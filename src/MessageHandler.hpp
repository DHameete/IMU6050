#pragma once
#ifndef MESSAGEHANDLER_HPP
#define MESSAGEHANDLER_HPP

#include <Arduino.h>

struct js_event {
    uint32_t time;
    int16_t value;
    uint8_t type;
    uint8_t number;
};


class MessageHandler {
public:

    MessageHandler();
    ~MessageHandler();

    void update(uint8_t dir, int16_t value);
    void send(const js_event* table) ; 


private:
    
    js_event _js_data;
    uint8_t _size_js = sizeof(struct js_event);

};

#endif