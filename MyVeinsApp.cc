#include "MyVeinsApp.h"
#include <cmath>

Define_Module(MyVeinsApp);

void MyVeinsApp::initialize(int stage)
{
    BaseWaveApplLayer::initialize(stage);

    if (stage == 0) {
        // Attack assignment
        isAttacker = uniform(0, 1) < 0.10;        // 10% vehicles are attackers
        attackType = intuniform(1,5);             // pick attack type randomly

        falseOffset = par("falseOffset").doubleValue();

        predictionInterval = 0.3;      // 300 ms
        positionThreshold = 5.0;       // meters
        propagationThreshold = 10.0;   // meters
        speedOfLight = 3e8;            // m/s

        // open CSV for logging
        logFile.open("bsm_features.csv");
        logFile << "senderId,distance_sr,predicted_error,propagation_error,detectedMalicious\n";
    }
}

void MyVeinsApp::finish()
{
    if (logFile.is_open()) {
        logFile.close();
    }
    BaseWaveApplLayer::finish();
}

void MyVeinsApp::populateWSM(BaseFrame1609_4* wsm)
{
    Coord realPos = mobility->getCurrentPosition();
    Coord speed = mobility->getCurrentSpeed();

    Coord transmitPos = realPos;

    if (isAttacker) {
        switch (attackType) {
            case 1: // constant position
                transmitPos = Coord(1000, 1000, 0); 
                break;
            case 2: // constant offset
                transmitPos.x += falseOffset;
                transmitPos.y += falseOffset;
                break;
            case 3: // random
                transmitPos.x = uniform(0, 1000);
                transmitPos.y = uniform(0, 1000);
                break;
            case 4: // random offset
                transmitPos.x += uniform(-falseOffset, falseOffset);
                transmitPos.y += uniform(-falseOffset, falseOffset);
                break;
            case 5: // eventual stop
                if (simTime() > 10) { // starts attack after 10s
                    transmitPos = lastBeacon[wsm->getSenderAddress()].position;
                }
                break;
        }
    }

    wsm->setSenderPos(transmitPos);
    wsm->setSenderSpeed(speed);
    wsm->setTimestamp(simTime());
}

void MyVeinsApp::onWSM(BaseFrame1609_4* wsm)
{
    int senderId = wsm->getSenderAddress();
    Coord senderPos = wsm->getSenderPos();
    Coord senderSpeed = wsm->getSenderSpeed();
    simtime_t t1 = wsm->getTimestamp();
    simtime_t t = simTime();
    Coord receiverPos = mobility->getCurrentPosition();

    // ============================
    // STEP 1: Propagation Validation
    // ============================
    double propagationError = computePropagationError(senderPos, receiverPos, t - t1);

    // ============================
    // STEP 2: Motion Prediction
    // ============================
    double predictionError = 0.0;
    if (lastBeacon.find(senderId) != lastBeacon.end()) {
        Coord predicted = computePredictedPosition(
            lastBeacon[senderId].position,
            lastBeacon[senderId].speed
        );
        predictionError = computeDistance(predicted, senderPos);
    }

    // Detect suspicious
    bool detectedMalicious = (propagationError > propagationThreshold) || 
                             (predictionError > positionThreshold);

    // Log for ML phase
    logFile << senderId << ","
            << computeDistance(senderPos, receiverPos) << ","
            << predictionError << ","
            << propagationError << ","
            << detectedMalicious << "\n";

    // Store last beacon for next prediction
    BeaconInfo info;
    info.position = senderPos;
    info.speed = senderSpeed;
    info.timestamp = t1;
    lastBeacon[senderId] = info;
}

// ============================
// Helper functions
// ============================
double MyVeinsApp::computeDistance(Coord a, Coord b)
{
    return sqrt(pow(a.x - b.x,2) + pow(a.y - b.y,2));
}

Coord MyVeinsApp::computePredictedPosition(Coord pos, Coord speed)
{
    Coord predicted;
    predicted.x = pos.x + speed.x * predictionInterval;
    predicted.y = pos.y + speed.y * predictionInterval;
    predicted.z = 0;
    return predicted;
}

double MyVeinsApp::computePropagationError(Coord senderPos, Coord receiverPos, simtime_t delta)
{
    double actualDistance = computeDistance(senderPos, receiverPos);
    double expectedDistance = speedOfLight * delta.dbl();
    return fabs(actualDistance - expectedDistance);
}
