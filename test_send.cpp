#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <chrono>
#include "serialport.hpp"
#include <vector>
#include <iostream>
#include <thread>
#include <signal.h>

bool running = true;
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    running = false;
}

int main() {
    signal(SIGINT, signalHandler);
    system("echo 123 | sudo -S chmod 666 /dev/ttyACM0");
    SerialPort::Config config;
    config.baudrate = B4000000;
    SerialPort serial_port("/dev/ttyACM0", config);

    while (running){
        /* code */
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        int64_t secs = ns / 1000000000;
        int64_t nsecs = ns % 1000000000;

        std::string data = std::to_string(secs) + "," + std::to_string(nsecs) + "\n";
        std::cout << "send: " << data.length() << " bytes " << data;
        serial_port.writeString(data);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    std::cout << "closing serial port\n";
    serial_port.close();
    return 0;
}