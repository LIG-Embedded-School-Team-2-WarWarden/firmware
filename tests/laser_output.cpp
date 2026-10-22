#include "laser.hpp"
#include <stdexcept>
using namespace device;
static void check(bool ok) { if (!ok) throw std::runtime_error("laser output contract failed"); }
int main() {
    LaserOutput eff(true, true, "", 0, 1);
    check(eff.supported() && eff.read() == pb::LASER_STATE_OFF);
    check(eff.set(true) && eff.read() == pb::LASER_STATE_ON);
    eff.latch_stop();
    check(eff.estop_latched() && eff.read() == pb::LASER_STATE_OFF);
    check(!eff.set(true) && eff.set(false));
    LaserOutput obs(true, false, "", 0, 1);
    check(!obs.supported() && !obs.set(true) && obs.read() == pb::LASER_STATE_UNKNOWN);
    LaserOutput unconfigured(false, true, "", 0, 1);
    check(!unconfigured.supported() && !unconfigured.set(true));
}
