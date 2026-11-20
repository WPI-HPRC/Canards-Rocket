#include "debouncer.h"
#define TEMPLATE_STATES_OVERRIDE
#include "../State.h"
#include "Arduino.h"
#include "StateMachineConstants.h"
#include "logging.h"

#include "../boilerplate/StateExtimator/simple_estimator.h"
#include "../boilerplate/controller/controller.h"

#include <math.h>

// defined in main.cpp
extern SimpleEstimator simpleEstimator;

struct CanardsData {
    float canardAngle[4];   // servo model state, fed back to the LQR [rad]
    float canardCommand[4]; // last LQR output [rad], not sent anywhere yet
    uint32_t lastStepMs;

    float maxHeight;
    uint32_t lastBaroTime;
    Debouncer apogeeDebouncer;
};

void *canardsInit(StateData const *data) {
    CanardsData *localData = static_cast<CanardsData *>(malloc(sizeof(CanardsData)));
    for (int i = 0; i < 4; i++) {
        localData->canardAngle[i] = 0.0f;
        localData->canardCommand[i] = 0.0f;
    }
    localData->lastStepMs = millis();
    localData->maxHeight = -INFINITY;
    localData->lastBaroTime = 0;
    localData->apogeeDebouncer = Debouncer(APOGEE_DEBOUNCE_MS);
    // Debouncer starts with lastTime = 0, so the first true would pass
    // instantly; a false resets it to "not started"
    localData->apogeeDebouncer.update(false, 0);

    Log.infoln("Canards active");
    return localData;
}

// Tilt of the nose (+Z) away from vertical, as a rotation vector in body axes
// ([0] pitch about x, [1] yaw about y, [2] roll = 0). Roll angle is not an
// error; only roll rate is (via body_rate).
// TODO(GNC): confirm the sign convention against the controller sim
static void tiltError(const BLA::Matrix<3, 1> &rpy, float error[3]) {
    const float roll = rpy(0);
    const float pitch = rpy(1);

    // "up" in body axes: R^T * [0, 0, -1] (estimator quaternion is body -> NED)
    const float ux = sinf(pitch);
    const float uy = -sinf(roll) * cosf(pitch);
    const float uz = -cosf(roll) * cosf(pitch);

    // rotation taking up onto the nose: axis = up x z_hat, angle = acos(up . z_hat)
    const float ax = uy;
    const float ay = -ux;
    const float axisNorm = sqrtf(ax * ax + ay * ay);
    const float angle = acosf(fmaxf(-1.0f, fminf(1.0f, uz)));

    if (axisNorm < 1e-6f) {
        error[0] = 0.0f;
        error[1] = 0.0f;
    } else {
        error[0] = angle * ax / axisNorm;
        error[1] = angle * ay / axisNorm;
    }
    error[2] = 0.0f;
}

StateID canardsLoop(StateData const *data, Context *ctx, void *_localData) {
    CanardsData *localData = static_cast<CanardsData *>(_localData);

    // LQR at 200 Hz
    const uint32_t now = millis();
    if (now - localData->lastStepMs >= CONTROL_PERIOD_MS) {
        localData->lastStepMs = now;

        float attitudeError[3];
        tiltError(simpleEstimator.get_orientation_rpy(), attitudeError);

        // LSM6 gyro is in deg/s
        const auto &lsm6_desc = ctx->lsm.get_descriptor();
        BLA::Matrix<3, 1> gyro = {lsm6_desc.data.gyr0 * DEG_TO_RAD,
                                  lsm6_desc.data.gyr1 * DEG_TO_RAD,
                                  lsm6_desc.data.gyr2 * DEG_TO_RAD};
        BLA::Matrix<3, 1> rate = simpleEstimator.unbiased_gyro(gyro);
        const float bodyRate[3] = {rate(0), rate(1), rate(2)};

        gnc::lqrController(attitudeError, bodyRate, localData->canardAngle,
                           ctx->axialVelocity, localData->canardCommand);
        gnc::servoModelStep(localData->canardAngle, localData->canardCommand);

        // TODO: send to servos, gnc::canardToServo(gnc::kLinkageN2, canardCommand[i])
    }

    // apogee: baro height has dropped APOGEE_DROP_M below the max
    const auto &baro_desc = ctx->baro.get_descriptor();
    if (baro_desc.getLastUpdated() != localData->lastBaroTime) {
        localData->lastBaroTime = baro_desc.getLastUpdated();

        // LPS22 pressure is in hPa, get_height wants Pa
        const float height = simpleEstimator.get_height(baro_desc.data.pressure * 100.0f);
        if (height > localData->maxHeight) {
            localData->maxHeight = height;
        }

        if (localData->apogeeDebouncer.update(height < localData->maxHeight - APOGEE_DROP_M, data->currentTime)) {
            return PARACHUTE;
        }
    }

    return CANARDS;
}
