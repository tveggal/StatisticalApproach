//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/

// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "MyVeinsApp.h"
#include <cmath>

Define_Module(MyVeinsApp);

void MyVeinsApp::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);

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
    DemoBaseApplLayer::finish();
}

int MyVeinsApp::extractNumericIdFromSId(const std::string& sid)
{
    size_t pos = sid.find_last_of('.');
    if (pos == std::string::npos || pos + 1 >= sid.size())
        return -1;

    try {
        return std::stoi(sid.substr(pos + 1));
    }
    catch (...) {
        return -1;
    }
}

void MyVeinsApp::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial)
{
    auto* bsm = dynamic_cast<DemoSafetyMessage*>(wsm);
    if (!bsm) return;

    if (auto* mob = veins::TraCIMobilityAccess().get(getParentModule())) {
        bsm->setSenderId(mob->getExternalId().c_str());
    }
    else {
        bsm->setSenderId("");
    }

    const std::string senderSId = bsm->getSenderId();
    const int senderVId = extractNumericIdFromSId(senderSId);
    if (senderVId < 0) return;

    Coord realPos = mobility->getPositionAt(simTime());
    Coord transmitPos = realPos;
    double realSpeed = mobility->getSpeed();
    double posX = realPos.x;
    double posY = realPos.y;
    const int senderId = static_cast<int>(senderVId);

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
                    transmitPos = lastBeacon[senderId].position;
                }
                break;
        }
    }

    bsm->setSenderPos(transmitPos);
    bsm->setSenderSpeed(senderSpeed);
    bsm->setTimestamp(simTime());
}

void MyVeinsApp::onBSM(DemoSafetyMessage* bsm)
{
    const std::string senderSId = bsm->getSenderId();
    const int senderId = extractNumericIdFromSId(senderSId);
    if (senderId < 0) return;

    Coord senderPos = bsm->getSenderPos();
    Coord senderSpeed = bsm->getSenderSpeed();
    simtime_t t1 = bsm->getTimestamp();
    simtime_t t = simTime();
    Coord receiverPos = mobility->getPositionAt(simTime());

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
