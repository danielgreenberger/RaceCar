#include <iostream>
#include <string>
#include <stdio.h>
//#include "ITcpClient.h"
//#include "librealsense2/rs.hpp"
//#include "ISerial.h"
#include "RaceCar.h"
#include <unistd.h>



int main() {
    std::cout << " PID: " << getpid() << std::endl;
    RaceCar car;
    car.connect("132.68.36.138",5555,"132.68.36.107");
    car.run();

    std::cout << " ok" << std::endl;


    std::thread thread([&car]()
    {
        while(car._is_running)
        {
            char ch;
            std::cin >> ch;
            if(ch == 'q')
            {
                car._is_running = false;
                break;
            }
        }
    });

    thread.join();
    return 0;
}