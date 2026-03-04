#include <fstream>
#include <map>

struct BeaconInfo {
    Coord position;
    Coord speed;
    simtime_t timestamp;
};

class MyVeinsApp : public DemoBaseApplLayer {
private:
    std::map<int, BeaconInfo> lastBeacon;
    std::ofstream logFile;
    double predictionInterval;
    double propagationThreshold;
    double positionThreshold;
    double speedOfLight;
    int TP, FP, TN, FN;
protected:
    virtual void initialize(int stage);
    virtual void finish();
    virtual void onBSM(DemoSafetyMessage* bsm);
};
