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
#include <sstream>

bool running = true;
void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    running = false;
}

int main() {
    signal(SIGINT, signalHandler);
    system("echo 123 | sudo -S chmod 666 /dev/ttyACM1");
    SerialPort::Config config;
    config.baudrate = B4000000;
    SerialPort serial_port("/dev/ttyACM1", config);

    std::string result_buffer = "";
    // serial_port.clearBothBuffers();
    while (running){
        /* code */
        // std::string result = serial_port.readline(100);
        serial_port.waitForReadyRead();
        std::string result = serial_port.readString();
        result_buffer += result;

        while ((result_buffer.find("\n") != std::string::npos) && (result_buffer.length() > 0)) {
            std::string line = result_buffer.substr(0, result_buffer.find("\n"));
            result_buffer = result_buffer.substr(result_buffer.find("\n") + 1);
            auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            int64_t secs = ns / 1000000000;
            int64_t nsecs = ns % 1000000000;
            // std::cout << "recv: " << line.length() << " bytes " << line << std::endl << std::flush;
            std::cout << "recv "<< secs  << "," << nsecs << ":" << line << std::endl << std::flush;
        }
        
        

        // std::cout << "recv "<< secs  << "," << nsecs << ":" << result << std::flush;
        
        // std::cout << result << std::flush;
    }
    std::cout << "closing serial port\n";
    serial_port.close();
    return 0;
}