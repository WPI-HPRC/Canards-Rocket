#include "debouncer.h"
#define TEMPLATE_STATES_OVERRIDE
#include "../State.h"
#include "Arduino.h"
#include "StateMachineConstants.h"
#include "config.h"
#include "logging.h"

struct PrelaunchData {
  Debouncer accelDebouncer;
};

void *prelaunchInit(StateData const *data) {
  PrelaunchData *localData = static_cast<PrelaunchData *>(malloc(sizeof(PrelaunchData)));
  localData->accelDebouncer = Debouncer(20);

  return localData;
}

StateID prelaunchLoop(StateData const *data, Context *ctx, void *_localData) {
  PrelaunchData *localData = static_cast<PrelaunchData *>(_localData);
  //static bool ComuteInitialOrientationThisLoop = false;

  const auto &accel_desc = ctx->asm330.get_descriptor();
  BLA::Matrix<3, 1> accel = {accel_desc.data.accel0, accel_desc.data.accel1,
                             accel_desc.data.accel2};

  const auto &mag_desc = ctx->mag.get_descriptor();
  BLA::Matrix<3, 1> mag = {mag_desc.data.mag0, mag_desc.data.mag1,
                           mag_desc.data.mag2};

  const auto &gps_desc = ctx->gps.get_descriptor();
  // BLA::Matrix<3, 1> gps = {gps_desc.data.ecefX, gps_desc.data.ecefY,
  //                          gps_desc.data.ecefZ};

  // Pad loop for 2 seconds after 5 second delay
  if (data->currentTime > 5000 && data->currentTime < 7000) {
    // ctx->estimator.padLoop(accel, mag, gps);
    //ComuteInitialOrientationThisLoop = true;

    // ctx->estimator.computeInitialOrientation();
    //ComuteInitialOrientationThisLoop = false;
    // ctx->ekfLooping = true;
  }

  //if (ComuteInitialOrientationThisLoop == true) {
  //  ctx->estimator.computeInitialOrientation();
  //  ComuteInitialOrientationThisLoop = false;
  //  ctx->ekfLooping = true;
  //}

  /*
  - Poll acceleration data from ctx
  - Check acceleration to detect launch
  - Check if need to abort
  - Update sensor data and ctx for next iteration?
  */
  //    if (accel_desc.getLastUpdated() != data->lastAccelReadingTime) {
  //        data->lastAccelReadingTime = accel_desc.getLastUpdated();
  //        if(data->accelDebouncer.update(accel_desc.data.accel0 >
  //        LAUNCH_TRHESHOLD, millis())) {
  //            return BOOST;
  //        }
  //    }

  // the EKF (ctx->estimator) isn't running, so use the raw ASM330 accel
  // along the nose (+Z, m/s^2); LAUNCH_TRHESHOLD is in g
  if (localData->accelDebouncer.update(accel_desc.data.accel2 > LAUNCH_TRHESHOLD * GRAVITY_MPS2, data->currentTime)) {
    return PRECANARDS;
  }

  return PRELAUNCH;
}
