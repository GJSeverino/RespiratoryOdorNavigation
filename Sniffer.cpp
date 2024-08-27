
#include "Sniffer.h"
#include <cmath>  // for sine and exponential functions
#include "random.h"

// Constants
const double SpaceHeight = 100.0;
const double SpaceWidth = 100.0;
const int numberOfSensors = 4;

// // Constants for motion
const double MaxAngle = M_PI / 12.0; // Pi/12
const double MaxThrust = 1.5; // 0.6 is default for previous evos 
const double Friction = 0.9; // 0.9 default

// for earlier work
// const double MaxAngle = M_PI / 12; // Pi/12
// const double MaxThrust = 0.6; // 0.6 is default for previous evos 
// const double Friction = 0.9; // 0.9 default


// Contructor
Sniffer::Sniffer(int networksize) {
    Set(networksize);
}

// destructor
Sniffer::~Sniffer() {
    Set(0);
}

// Initialize the agent
void Sniffer::Set(int networksize) {
    size = networksize;
    sensorweights.SetBounds(1, numberOfSensors*size);
    sensorweights.FillContents(0.0);
    posX = 0.0;
    posY = 0.0;
    pastposX = 0.0;
    pastposY = 0.0;
    velocity = 0.0;
    pastTheta = 0.0;
    theta = 0.0;
    is_passed_out = false;
    oxygenLevel = 50.0;
    co2Level = 20.0;

    sensor = 0.0;
    leftSensor = 0.0;
    rightSensor = 0.0;

    NervousSystem.SetCenterCrossing();
}

// Reset the state of the agent
void Sniffer::Reset(double initposX, double initposY, double initTheta) {
    posX = initposX;
    posY = initposY;
    pastposX = initposX;
    pastposY = initposY;
    sensor = 0.0;
    leftSensor = 0.0;
    rightSensor = 0.0;
    velocity = 0.0;
    theta = initTheta;
    NervousSystem.RandomizeCircuitState(0.0, 0.0);

    oxygenLevel = 100.0;
    co2Level = 0.0;
    is_passed_out = false;
}

// Map the output of neuron 3 to the breathing rate
double Sniffer::MapBreathingRate(double neuronOutput){
    double minRate = 0.1;
    double maxRate = 2.0;

    return minRate + (maxRate - minRate) * neuronOutput;

}

// Define rate of change of O2 and CO2
double Sniffer::dO2dt(double R) {
    double a = 0.5, b = 0.5; 
    return a * R - b ;
}

double Sniffer::dCO2dt(double R) {
    double c = 0.5, d = 0.5; 
    return c * R - d;
}

//Respiration Single Sensor
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// void Sniffer::Sense(double chemical_concentration, double current_time) {

//     double breathingRate = MapBreathingRate(NervousSystem.NeuronOutput(3));
//     double phase = sin(current_time * 2 * M_PI * breathingRate);

//     double R = breathingRate;

//     // Update O2 and CO2 levels
//     oxygenLevel += 0.01 * dO2dt(R);
//     co2Level += 0.01 * dCO2dt(R);

//     if (phase > 0) {
//         sensor = chemical_concentration * phase;
//     } else {
//         sensor = 0.0;
//     }

//     // Update energy based on oxygen and CO2 levels
//     if (oxygenLevel < 10 || co2Level > 90) {

//         sensor = 0.0;             // Impaired sensing due to imbalanced respiratory levels
//         is_passed_out = true;
//     }
//         else {
//             is_passed_out = false;
//         }
// }

// Respiration Two Sensors 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Sniffer::SenseResp(double leftConcentration, double rightConcentration, double current_time) {

    current_time = current_time / 10.0;

    double breathingRate = MapBreathingRate(NervousSystem.NeuronOutput(3));
    double phase = sin(current_time * 2 * M_PI * breathingRate);

    double R = breathingRate;

    // Update O2 and CO2 levels
    oxygenLevel += 0.01 * dO2dt(R); // 0.01 for prev, 0.001 for recent one. 
    co2Level += 0.01 * dCO2dt(R);

    // sense oxygen and co2 levels
    o2sensor = oxygenLevel;
    co2sensor = co2Level;

    // Sense concentraion through respiratory rhythm 
    
    if (phase > 0) {
        leftSensor = leftConcentration * phase;
        rightSensor = rightConcentration * phase;

    } else {
        leftSensor = 0.0;
        rightSensor = 0.0;
    }

    // Update energy based on oxygen and CO2 levels
    if (oxygenLevel < 10 || co2Level > 90) {

        leftSensor = 0.0;             // Impaired sensing due to imbalanced respiratory levels
        rightSensor = 0.0;
        is_passed_out = true;
    }
        else {
            is_passed_out = false;
        }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// SINGLE SENSOR -- AGENT INTEGRATES OVER TIME TO SOLVE TASK
// void Sniffer::Sense(double chemical_concentration, double current_time){
//     if (current_time > 0.0) {

//         sensor = chemical_concentration;
//     } else {
//         sensor = 0.0;
//     }
// }

// TWO SENSORS -- AGENT CAN DETECT DIFFERENCE IN AMOUNT BETWEEN SENSOR OFFSET
void Sniffer::Sense(double leftConcentration, double rightConcentration) {
    leftSensor = leftConcentration;
    rightSensor = rightConcentration;
}

// Step in time
void Sniffer::Step(double StepSize) {
    
    pastposX = posX;
    pastposY = posY;
    pastTheta = theta;

// FOR SINGLE SENSOR 
    // for (int i = 1; i <= size; i++) {
    //     NervousSystem.SetNeuronExternalInput(i, sensor * sensorweights[i]);       
    // }

// FOR > 1 SENSORS 
// double sensorValues[3] = {sensor, o2sensor, co2sensor};
// double sensorValues[2] = {leftSensor, rightSensor};
double sensorValues[4] = {leftSensor, rightSensor, o2sensor, co2sensor};

for (int neuron = 1; neuron <= size; neuron++) {
    double externalInput = 0.0;
    for (int sensorType = 0; sensorType < numberOfSensors; sensorType++) {
        
        // Calculate the index for the current sensor weight
        int weightIndex = (neuron - 1) * numberOfSensors + sensorType + 1;
        
        // Add the contribution from this sensor type to the external input
        externalInput += sensorValues[sensorType] * sensorweights[weightIndex];
    }
    NervousSystem.SetNeuronExternalInput(neuron, externalInput);
}

	// Update the nervous system
    NervousSystem.EulerStep(StepSize);

    double outputMotorRight = NervousSystem.NeuronOutput(1); 
    double outputMotorLeft = NervousSystem.NeuronOutput(2); 

    // Calculate the torque and thrust based on the neural outputs
    double torque = (outputMotorRight - outputMotorLeft) * MaxAngle;
    double thrust = (outputMotorRight + outputMotorLeft) * MaxThrust;

    // Update velocity and angle
    velocity = velocity * Friction + StepSize * thrust;
    theta += StepSize * torque;

    // Calculate the new position based on velocity and angle
    posX += StepSize * velocity * cos(theta);
    posY += StepSize * velocity * sin(theta);

       // Check for lower and upper bounds
    if (oxygenLevel > 100) {oxygenLevel = 100.0;}
    if (oxygenLevel < 0) {oxygenLevel = 0;}
    if (co2Level < 0) {co2Level = 0.0;}
    if (co2Level > 100) {co2Level = 100;}


    // Zero boundary conditions
    if (posX >= SpaceWidth) posX = SpaceWidth;
    if (posX < 0.0) posX = 0.0;
    if (posY >= SpaceHeight) posY = SpaceHeight;
    if (posY < 0.0) posY = 0.0;


}




