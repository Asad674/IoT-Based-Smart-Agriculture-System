# IoT-Based-Smart-Agriculture-System

Implementing a sophisticated sensor network, our system utilizes three strategically placed moisture sensors, one temperature/humidity sensor, one MG-90 servo motor, and one mini-water pump. An ESP32 microcontroller efficiently manages to multitask, using FreeRTOS capabilities, collecting data from each sensor and seamlessly transmitting it to the online ThingSpeak platform which allows for monitoring of data from each sensor, pump status (on or off), and motor position. The motor and water pump are designated as shared resources between each of the moisture sensors and their access is controlled via semaphores. In response to moisture levels below a specified threshold, the ESP32 triggers the motor to face that specified direction and turns the pump on through a relay.
