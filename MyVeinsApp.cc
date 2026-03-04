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

void MyVeinsApp::onBSM(DemoSafetyMessage* bsm)
{
    int senderId = bsm->getSenderAddress();

    Coord senderPos = bsm->getSenderPos();
    Coord senderSpeed = bsm->getSenderSpeed();
    simtime_t t1 = bsm->getTimestamp();
    simtime_t t = simTime();

    Coord receiverPos = mobility->getCurrentPosition();

    double deltaTime = (t - t1).dbl();

    // Feature 1: sender-receiver distance
    double distance_sr =
        sqrt(pow(senderPos.x - receiverPos.x, 2) +
             pow(senderPos.y - receiverPos.y, 2));

    // Feature 2: propagation error
    double expectedDistance = speedOfLight * deltaTime;
    double propagationError =
        fabs(distance_sr - expectedDistance);

    // Feature 3: motion prediction error
    double predictionError = 0;

    if (lastBeacon.find(senderId) != lastBeacon.end()) {

        BeaconInfo prev = lastBeacon[senderId];

        Coord predicted;
        predicted.x = prev.position.x +
                      prev.speed.x * predictionInterval;
        predicted.y = prev.position.y +
                      prev.speed.y * predictionInterval;

        predictionError =
            sqrt(pow(predicted.x - senderPos.x, 2) +
                 pow(predicted.y - senderPos.y, 2));
    }

    bool detectedMalicious =
        (propagationError > propagationThreshold) ||
        (predictionError > positionThreshold);

    // Example ground truth
    bool actualMalicious = false;
    if (senderId % 10 == 0)
        actualMalicious = true;

    if (detectedMalicious && actualMalicious) TP++;
    else if (detectedMalicious && !actualMalicious) FP++;
    else if (!detectedMalicious && !actualMalicious) TN++;
    else if (!detectedMalicious && actualMalicious) FN++;

    // LOG TO CSV
    logFile << senderId << ","
            << distance_sr << ","
            << deltaTime << ","
            << propagationError << ","
            << predictionError << ","
            << actualMalicious << "\n";

    BeaconInfo info;
    info.position = senderPos;
    info.speed = senderSpeed;
    info.timestamp = t1;

    lastBeacon[senderId] = info;
}

void MyVeinsApp::finish()
{
    double accuracy =
        (double)(TP + TN) / (TP + TN + FP + FN);

    double precision =
        (TP + FP) ? (double)TP / (TP + FP) : 0;

    double recall =
        (TP + FN) ? (double)TP / (TP + FN) : 0;

    EV << "Accuracy: " << accuracy << endl;
    EV << "Precision: " << precision << endl;
    EV << "Recall: " << recall << endl;

    logFile.close();

    DemoBaseApplLayer::finish();
}
