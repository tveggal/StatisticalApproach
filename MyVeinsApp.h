#ifndef __MYVEINSAPP_H_
#define __MYVEINSAPP_H_

#include <map>
#include <fstream>
#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

using namespace omnetpp;
using namespace veins;

class MyVeinsApp : public BaseWaveApplLayer {

private:

    struct BeaconInfo {
        Coord position;
        Coord speed;
        simtime_t timestamp;
    };

    std::map<int, BeaconInfo> lastBeacon;

    std::ofstream logFile;

    // Simulation parameters
    bool isAttacker;
    int attackType;       // 1=constant pos, 2=constant offset, 3=random, 4=random offset, 5=stop
    double falseOffset;
    double predictionInterval;     // 300 ms
    double positionThreshold;      // meters
    double propagationThreshold;    // meters
    double speedOfLight;

protected:

    virtual void initialize(int stage) override;
    virtual void finish() override;

    virtual void populateWSM(BaseFrame1609_4* wsm) override;
    virtual void onWSM(BaseFrame1609_4* wsm) override;

    // Helper functions
    double computeDistance(Coord a, Coord b);
    Coord computePredictedPosition(Coord pos, Coord speed);
    double computePropagationError(Coord senderPos, Coord receiverPos, simtime_t delta);
};

#endif
