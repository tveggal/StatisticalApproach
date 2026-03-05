//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
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

#pragma once
#include "veins/veins.h"
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include <map>
#include <fstream>
#include <cmath>
using namespace omnetpp;

namespace veins {

class VEINS_API MyVeinsApp : public DemoBaseApplLayer {

public:
    void initialize(int stage) override;
    void finish() override;

protected:
    struct BeaconInfo {
        Coord position;
        Coord speed;
        simtime_t timestamp;
    };

    std::map<int, BeaconInfo> lastBeacon;
    static std::ofstream logFile;
    static bool isWritten;

    // Simulation parameters
    bool isAttacker;
    int attackType;
    double falseOffset;
    double predictionInterval;     // 300 ms
    double positionThreshold;      // meters
    double propagationThreshold;    // meters
    double speedOfLight;

protected:
    void populateWSM(BaseFrame1609_4* wsm,
                         LAddress::L2Type rcvId = LAddress::L2BROADCAST(),
                         int serial = 0) override;
    void onWSA(DemoServiceAdvertisment* wsa) override;
    void onWSM(BaseFrame1609_4* wsm) override;
    void onBSM(DemoSafetyMessage* bsm) override;
    int extractNumericIdFromSId(const std::string& sid);
    double computeDistance(Coord a, Coord b);
    Coord computePredictedPosition(Coord pos, Coord speed);
    double computePropagationError(Coord senderPos, Coord receiverPos, simtime_t delta);
    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
};
}
