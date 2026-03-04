#include "veins/modules/application/traci/MyVeinsApp.h"

using namespace veins;

Define_Module(veins::MyVeinsApp);

void MyVeinsApp::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);

    if (stage == 0) {

        predictionInterval = 0.3;
        speedOfLight = 3e8;

        propagationThreshold = 20.0;
        positionThreshold = 10.0;

        TP = FP = TN = FN = 0;

        logFile.open("veins_dataset.csv");

        logFile << "senderId,"
                << "distance_sr,"
                << "deltaTime,"
                << "propagationError,"
                << "predictionError,"
                << "label\n";
    }
}
