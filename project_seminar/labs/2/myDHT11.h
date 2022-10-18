#pragma once

#include <array>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <cstddef>
#include <time.h>
#include "mbed.h"


class MyDHT {
public:
    MyDHT(PinName _pin) : 
        dataPin_c(_pin),
        lastTimeOfUpdate(time(NULL)) {};

    double getHumidity() {
        updateData();
        return lastHumidity;
    }

    double getTemperature() {
        updateData();
        return lastTemperature;
    }

private:
    static constexpr uint8_t DHT11_MICROCONTROLLER_RESOLUTION_BITS =  8;
    static constexpr uint8_t SINGLE_BUS_DATA_FRAME_SIZE_BYTES      =  5;
    static constexpr uint8_t MAXIMUM_DATA_FRAME_SIZE_BITS          = 40; // 5x8
    static constexpr double  MINIMUM_SAMPLING_PERIOD_SECONDS       =  3; // Be conservative.
    using DataFrameBytes_t = std::array<uint8_t, SINGLE_BUS_DATA_FRAME_SIZE_BYTES>;
    using DataFrameBits_t  = std::array<uint8_t, MAXIMUM_DATA_FRAME_SIZE_BITS>;

    unsigned int minimumTimeForUpdate_c = 3;
    const PinName dataPin_c;
    unsigned int lastTimeOfUpdate = 0;
    float lastHumidity = 0;
    float lastTemperature = 0;

    DataFrameBytes_t dataFrame;

    const unsigned int HIGH = 1;
    const unsigned int LOW = 0;

    /*
        Function which wait other of input logical level
    */
    bool waitSignal(DigitalInOut& io, const unsigned int& level, const unsigned int maxTime) {
        int count = 0;
        while (level == io)
        {
            if (count > maxTime)
            {
                return false;
            }
            count++;
            wait_us(1);
        }
        return true;
    }

    /*
        Function which read of transfering data from DHT11
    */
    bool readDataFromPin(DataFrameBits_t& bitData, DigitalInOut& io) {
        for (int i = 0; i < SINGLE_BUS_DATA_FRAME_SIZE_BYTES; i++) {
            for (int j = 0; j < DHT11_MICROCONTROLLER_RESOLUTION_BITS; j++) {
                if (!waitSignal(io, 0, 75))
                {
                    return false;
                }
                // logic 0 is 28us max, 1 is 70us
                wait_us(40);
                bitData[i*DHT11_MICROCONTROLLER_RESOLUTION_BITS + j] = io;
                if (!waitSignal(io, 1, 50))
                {
                    return false;
                }
            }
        }
        for (int i = 0; i < SINGLE_BUS_DATA_FRAME_SIZE_BYTES; i++)
        {
            uint8_t b = 0;
            for (int j = 0; j < DHT11_MICROCONTROLLER_RESOLUTION_BITS; j++)
            {
                if (bitData[i*DHT11_MICROCONTROLLER_RESOLUTION_BITS + j] == 1)
                {
                    b |= (1 << (7-j));
                }
            }
            dataFrame[i] = b;
        }
        return true;
    }

    /*
        Function which validating data that are getting from DHT11
    */
    bool isValidCheckSum() {
        return dataFrame[4] == ((dataFrame[0] + dataFrame[1] + dataFrame[2] + dataFrame[3]) & 0xFF);
    }

    void updateTemperature() {
        lastTemperature = float(dataFrame[2]);
    }

    void updateHumidity() {
        lastHumidity = dataFrame[0];
    }

    /*
        Full cycle of update data with tech delays and logical reading
    */
    void updateData() {
        std::array<std::array<int, 2>, 3> paramsForWaitSignal;
        paramsForWaitSignal[0] = {1, 40};
        paramsForWaitSignal[1] = {0, 100};
        paramsForWaitSignal[2] = {1, 100};

        auto currentTimeOfUpdate = time(NULL);
        if (difftime(currentTimeOfUpdate, lastTimeOfUpdate) < minimumTimeForUpdate_c) {
            return;
        }
        lastTimeOfUpdate = currentTimeOfUpdate;

        dataFrame.fill(0);

        DigitalInOut interface(dataPin_c);

        if (!waitSignal(interface, 0, 250)) {
            return;
        }

        interface.output();
        interface = LOW;

        thread_sleep_for(20); // tech moment

        interface.mode(PullUp);
        interface = HIGH;

        wait_us(30);

        interface.input();

        if (!waitSignal(interface, 1, 100)) {
            return;
        }
        if (!waitSignal(interface, 0, 100)) {
            return;
        }
        if (!waitSignal(interface, 1, 100)) {
            return;
        }

        DataFrameBits_t bitData = {};

        readDataFromPin(bitData, interface);

        if (isValidCheckSum()) {
            updateTemperature();
            updateHumidity();
        } else {
            return;
        }
        
    }
};
