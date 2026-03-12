#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <memory>

#include "SparkMax.hpp"

namespace py = pybind11;

/*
This is specifically configured for a SPARK MAX with a NEO 550 Motor. 
*/

// We use a "static" map or a class member so it stays in memory between calls
std::map<int, SparkMax> connectedMotors;

struct MotorFeedback{
    float dutyCycle;
    float motorVelocity;
};

std::vector<MotorFeedback> run_motor(std::string canbus, std::vector<int> motor_IDs, std::vector<float> dutyCycles){
    std::vector<MotorFeedback> feedbackList;
    feedbackList.reserve(motor_IDs.size()); // Pre-allocate memory for all motors

    // Check that each motor ID has an associated dutyCycle
    if (motor_IDs.size() != dutyCycles.size()){
        throw std::invalid_argument( "Number of Duty Cycles does not match the number of motors requested." );
    }

    // Initialize SparkMax object with CAN interface and CAN ID
    for (size_t i=0; i < motor_IDs.size(); ++i){

        int motor_ID = motor_IDs[i];
        float dutyCycle = dutyCycles[i];

        // Check if the motor is already connected  
        if (connectedMotors.find(motor_ID) == connectedMotors.end()){
            // connectedMotors.emplace(motor_ID, SparkMax (canbus, motor_ID));
            connectedMotors.emplace(std::piecewise_construct,
                                    std::forward_as_tuple(motor_ID),
                                    std::forward_as_tuple(canbus, motor_ID));
            // Configure and burn parameters for NEO Vortex
            SparkMax& new_motor = connectedMotors.at(motor_ID);
            new_motor.SetIdleMode(IdleMode::kBrake);
            new_motor.SetMotorType(MotorType::kBrushless);
            new_motor.SetSensorType(SensorType::kHallSensor);
            new_motor.SetRampRate(0);
            new_motor.SetInverted(false);
            new_motor.SetMotorKv(480);
            new_motor.SetEncoderCountsPerRev(4096);
            new_motor.SetSmartCurrentFreeLimit(20.0);
            new_motor.SetSmartCurrentStallLimit(80.0);
            new_motor.BurnFlash();        
        }
        SparkMax& motor = connectedMotors.at(motor_ID);

        MotorFeedback data;
        motor.Heartbeat();
        motor.SetDutyCycle(dutyCycle);
        data.motorVelocity = motor.GetVelocity();
        data.dutyCycle = motor.GetDutyCycle();
        
        feedbackList.push_back(data);
    }
    
    return feedbackList;
}




 //int main()
 //{
   //try {
     //// Initialize SparkMax object with CAN interface and CAN ID
     //SparkMax motor("can0", 6);
     //SparkMax motor2("can0", 3);

     //// Configure and burn parameters for NEO Vortex
     //motor.SetIdleMode(IdleMode::kBrake);
     //motor.SetMotorType(MotorType::kBrushless);
     //motor.SetSensorType(SensorType::kHallSensor);
     //motor.SetRampRate(0);
     //motor.SetInverted(false);
     //motor.SetMotorKv(480);
     //motor.SetEncoderCountsPerRev(4096);
     //motor.SetSmartCurrentFreeLimit(20.0);
     //motor.SetSmartCurrentStallLimit(80.0);
     //motor.BurnFlash();

     //// Configure and burn parameters for NEO Vortex
     //motor2.SetIdleMode(IdleMode::kBrake);
     //motor2.SetMotorType(MotorType::kBrushless);
     //motor2.SetSensorType(SensorType::kHallSensor);
     //motor2.SetRampRate(0);
     //motor2.SetInverted(false);
     //motor2.SetMotorKv(480);
     //motor2.SetEncoderCountsPerRev(4096);
     //motor2.SetSmartCurrentFreeLimit(20.0);
     //motor2.SetSmartCurrentStallLimit(80.0);
     //motor2.BurnFlash();

     //// Loop for 10 seconds
     //auto start = std::chrono::high_resolution_clock::now();
     //while (std::chrono::duration_cast<std::chrono::seconds>(
         //std::chrono::high_resolution_clock::now() -
         //start)
       //.count() < 10)
     //{
       //// Enable and run motors
       //motor.Heartbeat();
       //motor2.Heartbeat();

       //motor.SetDutyCycle(0.2);
       //motor2.SetDutyCycle(-0.1);
     //}
   //} catch (const std::exception & e) {
     //std::cerr << "Error: " << e.what() << std::endl;
     //return -1;
   //}
   //return 0;
 //}

// BINDING DEFINITION
PYBIND11_MODULE(motor_lib, m) {
    py::class_<MotorFeedback>(m, "MotorFeedback")
        .def_readwrite("dutyCycle", &MotorFeedback::dutyCycle)
        .def_readwrite("motorVelocity", &MotorFeedback::motorVelocity);

    m.def("run_motor", &run_motor, "Send CAN commands and get feedback");
}
