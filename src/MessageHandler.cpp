#include "MessageHandler.hpp"

MessageHandler::MessageHandler(){}

MessageHandler::~MessageHandler() {}

void MessageHandler::update(uint8_t dir, int16_t value) {

    _js_data = {millis(), value, 2, dir};
    send(&_js_data);

}

void MessageHandler::send(const js_event* table) {

  Serial.write((const char*)table, _size_js);

}


// struct js_event js_data;
// int size_js = sizeof(struct js_event);

// // x-value
// js_data = {millis(), value, 2, 0};
// // y-value
// js_data = {millis(), value, 2, 1};
// // th-value
// js_data = {millis(), value, 2, 2};
