//
// Created by lucaz on 31/1/2026.
//

#ifndef SDCARDLOGGER_H
#define SDCARDLOGGER_H
#include <Printable.h>
#include <File>

class SdCardLogger {
    File file;

public:
    bool init(const char* filename) {
        file = SD.open(filename, FILE_APPEND);
        return file;
    }

    int log(const Printable& obj) {
        if (!file) return -1; // error
        obj.printTo(file);
        file.println(); // Es como printear "\n"
        file.flush();
    }
};

#endif //SDCARDLOGGER_H
