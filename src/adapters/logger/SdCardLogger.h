//
// Created by lucaz on 31/1/2026.
//

#ifndef SDCARDLOGGER_H
#define SDCARDLOGGER_H
#include <Printable.h>
#include <Arduino.h>
#include <SD.h>

class SdCardLogger {
    File _file;

public:
    bool init(const char* filename) {
        _file = SD.open(filename, FILE_APPEND);
        if (!_file) {
            return false;
        }

        return true;
    }

    int log(const Printable& obj) {
        if (!_file) return -1; // error
        obj.printTo(_file);
        _file.println(); // Es como printear "\n"
        _file.flush();
    }
};

#endif //SDCARDLOGGER_H
