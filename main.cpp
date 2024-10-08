// *******************************************************************************************
//                           Gabriel Juliano Severino 
//                                    2023
//
//       Chemoreceptive Navigation in Static and Fluctuating Environments
// 
//
// TO DO:
// 
// 
// *******************************************************************************************
#include "TSearch.h"
#include "Sniffer.h"
#include "CTRNN.h"
#include "random.h"
#include <random>
#include "Fluid.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

#define PRINTOFILE

// Task params
const double StepSize = 0.01;
const double RunDuration = 6000; // 6000, 5500 transient
const double TransDuration = 5500; // Transient duration 
const double EvalDuration = RunDuration - TransDuration; // Evaluation duration

const double SpaceHeight = 100.0;    // Size of the space
const double SpaceWidth = 100.0; 

const double peakPositionX = 50.0; // Position of the chemical source
const double peakPositionY = 50.0; 

const double sensorOffset = 1.0; // Offset of the sensor from the center of the agent

// EA params
const int POPSIZE = 500;    //500 
const int GENS = 1000;       //100
const double MUTVAR = 0.2;
const double CROSSPROB = 0.05;
const double EXPECTED = 1.1;
const double ELITISM = 0.02;

const int Stage1Gens = 300;

// Nervous system params
const int NumSensors = 4;
int N = 4;
const double WR = 8.0;
const double SR = 8.0;
const double BR = 8.0;
const double TMIN = 1.0;
const double TMAX = 10.0;

int	VectSize = N*N + 2*N + NumSensors*N;


// Global variable to hold the index for running on super computer

std::string fileIndex = "";

// ------------------------------------
// Genotype-Phenotype Mapping Functions
// ------------------------------------
void GenPhenMapping(TVector<double> &gen, TVector<double> &phen)
{
	int k = 1;
	// Time-constants
	for (int i = 1; i <= N; i++) {
		phen(k) = MapSearchParameter(gen(k), TMIN, TMAX);
		k++;
	}
	// Bias
	for (int i = 1; i <= N; i++) {
		phen(k) = MapSearchParameter(gen(k), -BR, BR);
		k++;
	}
	// Weights
	for (int i = 1; i <= N; i++) {
		for (int j = 1; j <= N; j++) {
			phen(k) = MapSearchParameter(gen(k), -WR, WR);
			k++;
		}
	}
	// Sensor Weights
	for (int i = 1; i <= N * NumSensors; i++) {
		phen(k) = MapSearchParameter(gen(k), -SR, SR);
		k++;
	}
}

double DistanceGradient(double posX, double posY, double peakPosX, double peakPosY, double steepness = 1.5) {
    // Calculate direct Euclidean distance along x and y axes
    double dx = std::abs(posX - peakPosX);
    double dy = std::abs(posY - peakPosY);

    // Euclidean distance in a 2D space
    double effective_distance = sqrt(dx * dx + dy * dy);

    // Normalize the distance to the maximum possible distance
    const double max_distance = sqrt((SpaceWidth * SpaceWidth) + (SpaceHeight * SpaceHeight));
    double normalized_distance = effective_distance / max_distance;

    // Calculate the gradient concentration using the normalized distance and steepness
    return 1.0 - std::abs(normalized_distance) * steepness;
}

double FitnessFunctionChemoIndex(TVector<double> &genotype, RandomState &rs)
{
	// Map genotype to phenotype
	TVector<double> phenotype;
	phenotype.SetBounds(1, VectSize);
	GenPhenMapping(genotype, phenotype);

	// Create the agent
	Sniffer Agent(N);

	// Instantiate the nervous systems
	Agent.NervousSystem.SetCircuitSize(N);

	int k = 1;

	// Time-constants
	for (int i = 1; i <= N; i++) {
		Agent.NervousSystem.SetNeuronTimeConstant(i,phenotype(k));
		k++;
	}
	// Biases
	for (int i = 1; i <= N; i++) {
		Agent.NervousSystem.SetNeuronBias(i,phenotype(k));
		k++;
	}
	// Weights
	for (int i = 1; i <= N; i++) {
		for (int j = 1; j <= N; j++) {
			Agent.NervousSystem.SetConnectionWeight(i,j,phenotype(k));
			k++;
		}
	}
	// Sensor Weights
	for (int i = 1; i <= N*NumSensors; i++) {
		Agent.SetSensorWeight(i,phenotype(k));
		k++;
	}

    double totalFit = 0.0;
    int trials = 0;

    // Vary the steepness of the gradient
    const double minSteepness = 0.1;
    const double maxSteepness = 2.0;
    const double steepnessStep = 0.5;

    for (double steepness = minSteepness; steepness <= maxSteepness; steepness += steepnessStep) {
        for (double theta = 0.0; theta < 2*M_PI; theta += M_PI/2) {  
          
            double x = rs.UniformRandom(10, SpaceWidth-10); 
            double y = rs.UniformRandom(10.0, SpaceHeight-10); 

            // Peak position of chemical gradient
            const double peakPositionX = rs.UniformRandom(10.0, SpaceWidth-10);
            const double peakPositionY = rs.UniformRandom(10.0, SpaceHeight-10); 

            // Calculate initial distance
            double initialDist = sqrt(pow(x - peakPositionX, 2) + pow(y - peakPositionY, 2));
            if (initialDist < 1.0) initialDist = 1.0; // Avoid division by zero
			
            // Set agent's position
            Agent.Reset(x, y, theta);

            double dist = 0.0;
            double totalDist = 0.0;
            double wallTouchPenalty = 0.1;

            for (double time = 0; time < RunDuration; time += StepSize) {

                // Check if the agent touches the wall
                bool touchesWall = (Agent.posX <= 0.0 || Agent.posX >= SpaceWidth ||  Agent.posY <= 0.0 || Agent.posY >= SpaceHeight);
                if (touchesWall) {
                    totalFit -= wallTouchPenalty; // Apply penalty
                }

////////// FOR TWO SENSORS 
				// // Calculate the positions of the left and right sensors
                double leftPosX = Agent.posX - sensorOffset * cos(Agent.theta + M_PI / 3);
                double leftPosY = Agent.posY - sensorOffset * sin(Agent.theta + M_PI / 3);
                double rightPosX = Agent.posX + sensorOffset * cos(Agent.theta + M_PI / 3);
                double rightPosY = Agent.posY + sensorOffset * sin(Agent.theta + M_PI / 3);
                
                // Calculate chemical gradients at the sensor positions
                double leftGradientValue = DistanceGradient(leftPosX, leftPosY, peakPositionX, peakPositionY, steepness);
                double rightGradientValue = DistanceGradient(rightPosX, rightPosY, peakPositionX, peakPositionY, steepness);

                // Sense the gradient
				Agent.Sense(leftGradientValue, rightGradientValue);
					
/////////// FOR ONE SENSOR
                // Calculate chemical gradient at Sniffer position
				// double gradientValue = DistanceGradient(Agent.posX, Agent.posY, peakPositionX, peakPositionY, steepness);

				// // Sense the gradient
				// Agent.Sense(gradientValue, time);
                
//////////////////////
				// Move based on sensed gradient
				Agent.Step(StepSize);
        
                if (time > TransDuration) {
                    double dx = std::abs(Agent.posX - peakPositionX);
                    double dy = std::abs(Agent.posY - peakPositionY);
                    dist += sqrt(dx * dx + dy * dy);
                }
            }
            double totaldist = (dist / (EvalDuration / StepSize));
            double fitnessForThisTrial = (initialDist - totaldist)/initialDist;
            fitnessForThisTrial = fitnessForThisTrial < 0.0 ? 0.0 : fitnessForThisTrial; // Ensure non-negative fitness

            totalFit += fitnessForThisTrial;
            trials++;
        // }
    }
}
    return totalFit / trials;
}

double FitnessFunctionChemoIndexResp(TVector<double> &genotype, RandomState &rs)
{
	// Map genotype to phenotype
	TVector<double> phenotype;
	phenotype.SetBounds(1, VectSize);
	GenPhenMapping(genotype, phenotype);

	// Create the agent
	Sniffer Agent(N);

    // std::cout << "Running simulation with N = " << N << std::endl;

	// Instantiate the nervous systems
	Agent.NervousSystem.SetCircuitSize(N);
	
	int k = 1;

	// Time-constants
	for (int i = 1; i <= N; i++) {
		Agent.NervousSystem.SetNeuronTimeConstant(i,phenotype(k));
		k++;
	}
	// Biases
	for (int i = 1; i <= N; i++) {
		Agent.NervousSystem.SetNeuronBias(i,phenotype(k));
		k++;
	}
	// Weights
	for (int i = 1; i <= N; i++) {
		for (int j = 1; j <= N; j++) {
			Agent.NervousSystem.SetConnectionWeight(i,j,phenotype(k));
			k++;
		}
	}
	// Sensor Weights
	for (int i = 1; i <= N*NumSensors; i++) {
		Agent.SetSensorWeight(i,phenotype(k));
		k++;
	}

    double totalFit = 0.0;
    int trials = 0;

    // Vary the steepness of the gradient
    const double minSteepness = 0.1;
    const double maxSteepness = 2.0;
    const double steepnessStep = 0.5;

    for (double steepness = minSteepness; steepness <= maxSteepness; steepness += steepnessStep) {
        for (double theta = 0.0; theta < 2*M_PI; theta += M_PI/2) {  
          
            double x = rs.UniformRandom(10, SpaceWidth-10); 
            double y = rs.UniformRandom(10.0, SpaceHeight-10); 

            // Peak position of chemical gradient
            const double peakPositionX = rs.UniformRandom(10.0, SpaceWidth-10);
            const double peakPositionY = rs.UniformRandom(10.0, SpaceHeight-10); 

            // Calculate initial distance
            double initialDist = sqrt(pow(x - peakPositionX, 2) + pow(y - peakPositionY, 2));
            if (initialDist < 1.0) initialDist = 1.0; // Avoid division by zero
			
            // Set agent's position
            Agent.Reset(x, y, theta);

            double dist = 0.0;
            double totalDist = 0.0;
            double wallTouchPenalty = 0.1;

            for (double time = 0; time < RunDuration; time += StepSize) {

                // Check if the agent touches the wall
                bool touchesWall = (Agent.posX <= 0.0 || Agent.posX >= SpaceWidth ||  Agent.posY <= 0.0 || Agent.posY >= SpaceHeight);
                if (touchesWall) {
                    totalFit -= wallTouchPenalty; // Apply penalty
                }

				if (Agent.GetPassedOutState() == true) {totalFit -= 0.5;}

////////// FOR TWO SENSORS 
				// // Calculate the positions of the left and right sensors
                double leftPosX = Agent.posX - sensorOffset * cos(Agent.theta + M_PI / 3);
                double leftPosY = Agent.posY - sensorOffset * sin(Agent.theta + M_PI / 3);
                double rightPosX = Agent.posX + sensorOffset * cos(Agent.theta + M_PI / 3);
                double rightPosY = Agent.posY + sensorOffset * sin(Agent.theta + M_PI / 3);
                
                // Calculate chemical gradients at the sensor positions
                double leftGradientValue = DistanceGradient(leftPosX, leftPosY, peakPositionX, peakPositionY, steepness);
                double rightGradientValue = DistanceGradient(rightPosX, rightPosY, peakPositionX, peakPositionY, steepness);

                // Sense the gradient
				Agent.SenseResp(leftGradientValue, rightGradientValue, time);
					
/////////// FOR ONE SENSOR
                // Calculate chemical gradient at Sniffer position
				// double gradientValue = DistanceGradient(Agent.posX, Agent.posY, peakPositionX, peakPositionY, steepness);

				// // Sense the gradient
				// Agent.Sense(gradientValue, time);
                
//////////////////////
				// Move based on sensed gradient
				Agent.Step(StepSize);
        
                if (time > TransDuration) {
                    double dx = std::abs(Agent.posX - peakPositionX);
                    double dy = std::abs(Agent.posY - peakPositionY);
                    dist += sqrt(dx * dx + dy * dy);
                }
            }
            double totaldist = (dist / (EvalDuration / StepSize));
            double fitnessForThisTrial = (initialDist - totaldist)/initialDist;
            fitnessForThisTrial = fitnessForThisTrial < 0.0 ? 0.0 : fitnessForThisTrial; // Ensure non-negative fitness

            totalFit += fitnessForThisTrial;
            trials++;
        // }
    }
}
    return totalFit / trials;
}



// 
// double Breathing(TVector<double> &genotype, RandomState &rs)
// {
// 	// Map genotype to phenotype
// 	TVector<double> phenotype;
// 	phenotype.SetBounds(1, VectSize);
// 	GenPhenMapping(genotype, phenotype);

// 	// Create the agent
// 	Sniffer Agent(N);

// 	// Instantiate the nervous systems
// 	Agent.NervousSystem.SetCircuitSize(N);
	
// 	int k = 1;

// 	// Time-constants
// 	for (int i = 1; i <= N; i++) {
// 		Agent.NervousSystem.SetNeuronTimeConstant(i,phenotype(k));
// 		k++;
// 	}
// 	// Biases
// 	for (int i = 1; i <= N; i++) {
// 		Agent.NervousSystem.SetNeuronBias(i,phenotype(k));
// 		k++;
// 	}
// 	// Weights
// 	for (int i = 1; i <= N; i++) {
// 		for (int j = 1; j <= N; j++) {
// 			Agent.NervousSystem.SetConnectionWeight(i,j,phenotype(k));
// 			k++;
// 		}
// 	}
// 	// Sensor Weights
// 	for (int i = 1; i <= N*NumSensors; i++) {
// 		Agent.SetSensorWeight(i,phenotype(k));
// 		k++;
// 	}

//  double totalFitness = 0.0;
//     int trials = 0;
//     const int NumTrials = 5;
//     const double MinOxygenLevel = 10.0;
//     const double MaxCO2Level = 90.0;

//     for (int trial = 0; trial < NumTrials; ++trial) {
//         Agent.Reset(rs.UniformRandom(0.0, SpaceWidth), 
//                     rs.UniformRandom(0.0, SpaceHeight),
//                     rs.UniformRandom(0.0, 2 * M_PI));

//         double fitnessThisTrial = 0.0;
//         bool oxygenMaximizedLast = false; // Track the last state

//         for (double time = 0; time < RunDuration; time += StepSize) {
            
			
// 				// // Calculate the positions of the left and right sensors
//                 double leftPosX = Agent.posX - sensorOffset * cos(Agent.theta + M_PI / 3);
//                 double leftPosY = Agent.posY - sensorOffset * sin(Agent.theta + M_PI / 3);
//                 double rightPosX = Agent.posX + sensorOffset * cos(Agent.theta + M_PI / 3);
//                 double rightPosY = Agent.posY + sensorOffset * sin(Agent.theta + M_PI / 3);
                
//                 // Calculate chemical gradients at the sensor positions
//                 double leftGradientValue = DistanceGradient(leftPosX, leftPosY, peakPositionX, peakPositionY,.5);
//                 double rightGradientValue = DistanceGradient(rightPosX, rightPosY, peakPositionX, peakPositionY, .5);

//                 // Sense the gradient
// 				Agent.Sense(leftGradientValue, rightGradientValue, time);
					
			
			
// 			// double gradientValue = DistanceGradient(Agent.posX, Agent.posY, peakPositionX, peakPositionY, 1.0);
        	
// 			// Agent.Sense(gradientValue, time);

// 			Agent.Step(StepSize);

//             double O2Level = Agent.GetOxygenLevel();
//             double CO2Level = Agent.GetCO2Level();
//             bool agentState = Agent.GetPassedOutState();

//             // Check if the agent is oscillating between maximizing O2 and minimizing CO2
//             if (!agentState) {
//                 if ((O2Level >= MinOxygenLevel && !oxygenMaximizedLast) || 
//                     (CO2Level <= MaxCO2Level && oxygenMaximizedLast)) {
//                     fitnessThisTrial += StepSize;
//                     oxygenMaximizedLast = !oxygenMaximizedLast;
//                 }
//             }
//         }

//         fitnessThisTrial = fitnessThisTrial / RunDuration;
//         totalFitness += fitnessThisTrial;
//         trials++;
//     }

//     return totalFitness / trials;
// }




// double CombinedFitnessFunction(TVector<double> &genotype, RandomState &rs) {
//     // calculate individual fitness scores
//     double breathingFitness = Breathing(genotype, rs);
//     double chemoIndexFitness = FitnessFunctionChemoIndex(genotype, rs);

//     //(assuming both functions return a score in [0,1])

//     const double weightBreathing = 0.15; // Adjust these weights as per requirement
//     const double weightChemoIndex = 0.85;

//     // weighted sum
//     // double combinedFitness = weightBreathing * breathingFitness + weightChemoIndex * chemoIndexFitness;

//     //  weighted product 
//     // double combinedFitness = std::pow(breathingFitness, weightBreathing) * std::pow(chemoIndexFitness, weightChemoIndex);

// 	double combinedFitness = breathingFitness * chemoIndexFitness;

//     return combinedFitness;
// }



// ================================================
// FUNCTIONS FOR ANALYZING A SUCCESFUL CIRCUIT
// ================================================

double PerformanceMap(TVector<double> &genotype)
{

	ofstream perf("PerformanceMap_4N_47.dat");

	// Map genotype to phenotype
	TVector<double> phenotype;
	phenotype.SetBounds(1, VectSize);
	GenPhenMapping(genotype, phenotype);

	// Create the agent
	Sniffer Agent(N);

    // std::cout << "Running simulation with N = " << N << std::endl;

	// Instantiate the nervous systems
	Agent.NervousSystem.SetCircuitSize(N);
	
	int k = 1;

	// Time-constants
	for (int i = 1; i <= N; i++) {
		Agent.NervousSystem.SetNeuronTimeConstant(i,phenotype(k));
		k++;
	}
	// Biases
	for (int i = 1; i <= N; i++) {
		Agent.NervousSystem.SetNeuronBias(i,phenotype(k));
		k++;
	}
	// Weights
	for (int i = 1; i <= N; i++) {
		for (int j = 1; j <= N; j++) {
			Agent.NervousSystem.SetConnectionWeight(i,j,phenotype(k));
			k++;
		}
	}
	// Sensor Weights
	for (int i = 1; i <= N*NumSensors; i++) {
		Agent.SetSensorWeight(i,phenotype(k));
		k++;
	}

    double totalFit = 0.0;
    int trials = 0;

    // Vary the steepness of the gradient
    const double minSteepness = 0.1;
    const double maxSteepness = 2.0;
    const double steepnessStep = 0.5;

for (double x = 0; x <= SpaceWidth; x += 1.0){
for (double y = 0; y <= SpaceHeight; y += 1.0){
    for (double steepness = minSteepness; steepness <= maxSteepness; steepness += steepnessStep) {
        for (double theta = 0.0; theta < 2*M_PI; theta += M_PI/2) {  

            // Peak position of chemical gradient
            const double peakPositionX = 50.0;
            const double peakPositionY = 50.0; 

            // Calculate initial distance
            double initialDist = sqrt(pow(x - peakPositionX, 2) + pow(y - peakPositionY, 2));
            if (initialDist < 1.0) initialDist = 1.0; // Avoid division by zero
			
            // Set agent's position
            Agent.Reset(x, y, theta);

            double dist = 0.0;
            double totalDist = 0.0;
            double wallTouchPenalty = 0.1;

            for (double time = 0; time < RunDuration; time += StepSize) {

                // // Punishment checks 
                // bool touchesWall = (Agent.posX <= 0.0 || Agent.posX >= SpaceWidth ||  Agent.posY <= 0.0 || Agent.posY >= SpaceHeight);
                // if (touchesWall) {totalFit -= wallTouchPenalty;}
				if (Agent.GetPassedOutState() == true) {totalFit -= 0.5;}

				// // Calculate the positions of the left and right sensors
                double leftPosX = Agent.posX - sensorOffset * cos(Agent.theta + M_PI / 3);
                double leftPosY = Agent.posY - sensorOffset * sin(Agent.theta + M_PI / 3);
                double rightPosX = Agent.posX + sensorOffset * cos(Agent.theta + M_PI / 3);
                double rightPosY = Agent.posY + sensorOffset * sin(Agent.theta + M_PI / 3);
                
                // Calculate chemical gradients at the sensor positions
                double leftGradientValue = DistanceGradient(leftPosX, leftPosY, peakPositionX, peakPositionY, steepness);
                double rightGradientValue = DistanceGradient(rightPosX, rightPosY, peakPositionX, peakPositionY, steepness);

                // Sense the gradient
				Agent.SenseResp(leftGradientValue, rightGradientValue, time);

				// Move based on sensed gradient
				Agent.Step(StepSize);
        
                if (time > TransDuration) {
                    double dx = std::abs(Agent.posX - peakPositionX);
                    double dy = std::abs(Agent.posY - peakPositionY);
                    dist += sqrt(dx * dx + dy * dy);
                }
            }
            double totaldist = (dist / (EvalDuration / StepSize));
            double fitnessForThisTrial = (initialDist - totaldist)/initialDist;
            fitnessForThisTrial = fitnessForThisTrial < 0.0 ? 0.0 : fitnessForThisTrial; // Ensure non-negative fitness
            totalFit += fitnessForThisTrial;
            trials++;
   
    } // Theta loop 
} // steepness loop 
 	perf << x << " " << y << " " << totalFit / trials << endl;
} // y loop 
} // x loop 
    perf.close();
}


// void BehavioralTraces_Specific(TVector<double> &genotype,double x1, double y1, double chemicalsourceX, double chemicalsourceY, double steepness)
// {
//     // Start output file for positions
//     ofstream positionFile("AgentPositions.dat");

//     // Map genotype to phenotype
//     TVector<double> phenotype;
//     phenotype.SetBounds(1, VectSize);
//     GenPhenMapping(genotype, phenotype);

//     // Create the agent
//     Sniffer Agent(N);

//     // Instantiate the nervous systems
//     Agent.NervousSystem.SetCircuitSize(N);
//     int k = 1;

//     // Time-constants
//     for (int i = 1; i <= N; i++) {
//         Agent.NervousSystem.SetNeuronTimeConstant(i, phenotype(k));
//         k++;
//     }
//     // Biases
//     for (int i = 1; i <= N; i++) {
//         Agent.NervousSystem.SetNeuronBias(i, phenotype(k));
//         k++;
//     }
//     // Weights
//     for (int i = 1; i <= N; i++) {
//         for (int j = 1; j <= N; j++) {
//             Agent.NervousSystem.SetConnectionWeight(i, j, phenotype(k));
//             k++;
//         }
//     }
//     // Sensor Weights
//     for (int i = 1; i <= N; i++) {
//         Agent.SetSensorWeight(i, phenotype(k));
//         k++;
//     }

//     // Run the simulation
//     // for (double y1 = 0.0; y1 < SpaceHeight; y1 += 50.0) {
//     //     for (double x1 = 0.0; x1 < SpaceWidth; x1 += 50.0) {
            
//             const double peakPositionX = chemicalsourceX;
//             const double peakPositionY = chemicalsourceY;

//             // Set agent's position
//             Agent.Reset(x1, y1, 0.0);

//             for (double time = 0; time < RunDuration; time += StepSize) {
                
//                 // Sense the gradient
//                 double gradientValue = DistanceGradient(Agent.posX, Agent.posY, peakPositionX, peakPositionY, steepness);

//                 Agent.Sense(gradientValue, time);

//                 // Move based on sensed gradient
//                 Agent.Step(StepSize);

//                 // Save the position at each timestep
//                 positionFile << Agent.posX << " " << Agent.posY << endl;
//             }
//     //     }
//     // }

//     positionFile.close();
// }

// void BehavioralTraces_Across_Conditions(TVector<double> &genotype, double steepness)
// {
//     // Start output file for positions
//     ofstream positionFile("AgentPositions.dat");

//     // Map genotype to phenotype
//     TVector<double> phenotype;
//     phenotype.SetBounds(1, VectSize);
//     GenPhenMapping(genotype, phenotype);

//     // Create the agent
//     Sniffer Agent(N);

//     // Instantiate the nervous systems
//     Agent.NervousSystem.SetCircuitSize(N);
//     int k = 1;

//     // Time-constants
//     for (int i = 1; i <= N; i++) {
//         Agent.NervousSystem.SetNeuronTimeConstant(i, phenotype(k));
//         k++;
//     }
//     // Biases
//     for (int i = 1; i <= N; i++) {
//         Agent.NervousSystem.SetNeuronBias(i, phenotype(k));
//         k++;
//     }
//     // Weights
//     for (int i = 1; i <= N; i++) {
//         for (int j = 1; j <= N; j++) {
//             Agent.NervousSystem.SetConnectionWeight(i, j, phenotype(k));
//             k++;
//         }
//     }
//     // Sensor Weights
//     for (int i = 1; i <= N; i++) {
//         Agent.SetSensorWeight(i, phenotype(k));
//         k++;
//     }

// 	const int NumPeakPositions = 6; // Change this as needed

//     // Initialize the agent's position
//     double initPosX = 50.0;
//     double initPosY = 50.0;
    
//     // Array to hold the peak positions
//     double peakPositionsX[NumPeakPositions] = { /* x coordinates */ };
//     double peakPositionsY[NumPeakPositions] = { /* y coordinates */ };

    
//     double angleStep = (2 * M_PI) / NumPeakPositions;
//     double radius = 25; // Change the radius as needed


//     for (int i = 0; i < NumPeakPositions; ++i) {
//         peakPositionsX[i] = initPosX + radius * std::cos(i * angleStep);
//         peakPositionsY[i] = initPosY + radius * std::sin(i * angleStep);
//     }

//     // Iterate over all peak positions
//     for (int i = 0; i < NumPeakPositions; ++i) {
//         // Reset agent to starting position
//         Agent.Reset(initPosX, initPosY, 0.0);

//         // Peak position of chemical gradient
//         double peakPositionX = peakPositionsX[i];
//         double peakPositionY = peakPositionsY[i];

//         for (double time = 0; time < RunDuration; time += StepSize) {
//             // Sense the gradient
//             double gradientValue = DistanceGradient(Agent.posX, Agent.posY, peakPositionX, peakPositionY, steepness);

//             Agent.Sense(gradientValue, time);

//             // Move based on sensed gradient
//             Agent.Step(StepSize);

//             // Save the position at each timestep
//             positionFile << Agent.posX << " " << Agent.posY << std::endl;
//         }
//     }

//     positionFile.close();
// }

// ================================================
// C. ADDITIONAL EVOLUTIONARY FUNCTIONS
// ================================================
int TerminationFunction(int Generation, double BestPerf, double AvgPerf, double PerfVar) {
	if (BestPerf > 0.99) return 1;
	else return 0;
}

int TerminationFunctionFirst(int Generation, double BestPerf, double AvgPerf, double PerfVar) {
	if (Generation > Stage1Gens) return 1;
	else return 0;
}


// ------------------------------------
// Display functions
// ------------------------------------
void EvolutionaryRunDisplay(int Generation, double BestPerf, double AvgPerf, double PerfVar)
{
	cout << Generation << " " << BestPerf << " " << AvgPerf << " " << PerfVar << endl;
}

void ResultsDisplay(TSearch &s)
{
	TVector<double> bestVector;
	ofstream BestIndividualFile;
	TVector<double> phenotype;
	phenotype.SetBounds(1, VectSize);

	// Save the genotype of the best individual

    // Use the global index in file names
    std::string bestGenFilename = "best.gen" + fileIndex + ".dat";
    std::string bestNsFilename = "best.ns" + fileIndex + ".dat";

	bestVector = s.BestIndividual();
	BestIndividualFile.open(bestGenFilename);
	BestIndividualFile << bestVector << endl;
	BestIndividualFile.close();

	// Also show the best individual in the Circuit Model form
	BestIndividualFile.open(bestNsFilename);
	GenPhenMapping(bestVector, phenotype);
	Sniffer Agent(N);

	// Instantiate the nervous system
	Agent.NervousSystem.SetCircuitSize(N);
	int k = 1;
	// Time-constants
	for (int i = 1; i <= N; i++) {
		Agent.NervousSystem.SetNeuronTimeConstant(i,phenotype(k));
		k++;
	}
	// Bias
	for (int i = 1; i <= N; i++) {
		Agent.NervousSystem.SetNeuronBias(i,phenotype(k));
		k++;
	}
	// Weights
	for (int i = 1; i <= N; i++) {
		for (int j = 1; j <= N; j++) {
			Agent.NervousSystem.SetConnectionWeight(i,j,phenotype(k));
			k++;
		}
	}
		// Sensor Weights
	for (int i = 1; i <= N * NumSensors; i++) {
		Agent.SetSensorWeight(i,phenotype(k));
		k++;
	}
	BestIndividualFile << Agent.NervousSystem << endl;
	BestIndividualFile << Agent.sensorweights << "\n" << endl;
	BestIndividualFile.close();
}

// ------------------------------------
// THE MAIN PROGRAM 
// ------------------------------------
int main (int argc, const char* argv[]) 
{

//-------------------------------------
// EVOLUTION 
//-------------------------------------

    long randomseed = static_cast<long>(time(NULL));
    std::string fileIndex = "";  // Default empty string for file index

    if (argc > 1) {
        randomseed += atoi(argv[1]);
        fileIndex = "_" + std::string(argv[1]); // Append the index to the file name
    }

     if (argc > 2) {
        N = atoi(argv[2]); // Convert the argument to an integer and assign it to N
        VectSize = N*N + 2*N + NumSensors*N;
    }
      // Convert N to string
    std::string nStr = std::to_string(N);

     // Create directories
    std::string baseDir = "N_" + nStr;
    mkdir(baseDir.c_str(), 0777); // Create a directory for the current N

    // Subdirectories
    std::string evoDir = baseDir + "/Evo_data";
    std::string genotypesDir = baseDir + "/Genotypes";
    std::string nervSystemsDir = baseDir + "/Nerv_Systems";
    std::string seedsDir = baseDir + "/Seeds";

    mkdir(evoDir.c_str(), 0777);
    mkdir(genotypesDir.c_str(), 0777);
    mkdir(nervSystemsDir.c_str(), 0777);
    mkdir(seedsDir.c_str(), 0777);


    TSearch s(VectSize);
	

    #ifdef PRINTOFILE
    std::ofstream file;
    std::string filename = evoDir + "/evol" + fileIndex + "_N" + nStr + ".dat"; 
    file.open(filename);
    cout.rdbuf(file.rdbuf());

    // Save the seed to a file
    std::ofstream seedfile;
    std::string seedfilename = seedsDir + "/seed"  + fileIndex + "_N" + nStr +".dat"; 
    seedfile.open(seedfilename); 
    seedfile << randomseed << std::endl;
    seedfile.close();
	
	RandomState rs(randomseed); 

	#endif
	
	// Configure the search
	s.SetRandomSeed(randomseed);
	s.SetSearchResultsDisplayFunction(ResultsDisplay);
	s.SetPopulationStatisticsDisplayFunction(EvolutionaryRunDisplay);
	s.SetSelectionMode(RANK_BASED);
	s.SetReproductionMode(GENETIC_ALGORITHM);
	s.SetPopulationSize(POPSIZE);
	s.SetMaxGenerations(GENS);
	s.SetCrossoverProbability(CROSSPROB);
	s.SetCrossoverMode(UNIFORM);
	s.SetMutationVariance(MUTVAR);
	s.SetMaxExpectedOffspring(EXPECTED);
	s.SetElitistFraction(ELITISM);
	s.SetSearchConstraint(1);
	
	// s.SetSearchTerminationFunction(TerminationFunction);
	// s.SetEvaluationFunction(FitnessFunctionChemoIndexResp);
    // s.ExecuteSearch();


	// /* Stage 1 */ // 
	// s.SetSearchTerminationFunction(TerminationFunctionFirst);
	// s.SetEvaluationFunction(FitnessFunctionChemoIndex);
    // s.ExecuteSearch();
	// s.SetGeneration(0);
    	/* Stage 2 */ //
	s.SetSearchTerminationFunction(TerminationFunction);
	s.SetEvaluationFunction(FitnessFunctionChemoIndexResp); 
	s.ExecuteSearch();




    TVector<double> bestVector;
	ofstream BestIndividualFile;
	TVector<double> phenotype;
	phenotype.SetBounds(1, VectSize);

	// Save the genotype of the best individual

    // Use the global index in file names
    std::string bestGenFilename = genotypesDir + "/best.gen" + fileIndex + "_N" + nStr + ".dat";
    std::string bestNsFilename = nervSystemsDir + "/best.ns" + fileIndex + "_N" + nStr + ".dat";

	bestVector = s.BestIndividual();
	BestIndividualFile.open(bestGenFilename);
	BestIndividualFile << bestVector << endl;
	BestIndividualFile.close();

	// Also show the best individual in the Circuit Model form
	BestIndividualFile.open(bestNsFilename);
	GenPhenMapping(bestVector, phenotype);
	Sniffer Agent(N);

	// Instantiate the nervous system
	Agent.NervousSystem.SetCircuitSize(N);
	int k = 1;
	// Time-constants
	for (int i = 1; i <= N; i++) {
		Agent.NervousSystem.SetNeuronTimeConstant(i,phenotype(k));
		k++;
	}
	// Bias
	for (int i = 1; i <= N; i++) {
		Agent.NervousSystem.SetNeuronBias(i,phenotype(k));
		k++;
	}
	// Weights
	for (int i = 1; i <= N; i++) {
		for (int j = 1; j <= N; j++) {
			Agent.NervousSystem.SetConnectionWeight(i,j,phenotype(k));
			k++;
		}
	}
		// Sensor Weights
	for (int i = 1; i <= N * NumSensors; i++) {
		Agent.SetSensorWeight(i,phenotype(k));
		k++;
	}
	BestIndividualFile << Agent.NervousSystem << endl;
	BestIndividualFile << Agent.sensorweights << "\n" << endl;
	BestIndividualFile.close();

// ================================================
// B. MAIN FOR ANALYZING A SUCCESFUL CIRCUIT
// ================================================
// 	ifstream genefile;
// 	genefile.open("/N/u/gjseveri/BigRed200/ActivePerceiver/OdorNav/N_4/Genotypes/best.gen_47_N4.dat");
// 	TVector<double> genotype(1, VectSize);
// 	genefile >> genotype;

// 	// // //     agent x position, agent y position, Peak x position, Peak y position, steepness
// 	// BehavioralTraces_Specific(genotype, 10.0, 10.0, 50.0, 50.0, 1.5);
// 	//  BehavioralTraces_Across_Conditions(genotype, 1.5);
// 	// // BehavioralTraces(genotype, 455.0, 2.5); 


// PerformanceMap(genotype);



// for (double sensorstate = 0.0; sensorstate <= 1.0; sensorstate += 0.1)
	// {
	// 	LimitSet(genotype,sensorstate);
	// }



	return 0;
}
