#ifdef __has_include
#if !__has_include("../states/States.h")
#define TEMPLATE_STATES_OVERRIDE
#endif
#endif

#if defined(TEMPLATE_STATES_OVERRIDE)
#include "../State.h"
#include "States.h"

hprc::States stateToTelemState(StateID state) {
  switch (state) {
  case PRELAUNCH:
    return hprc::States_PreLaunch;
  case PRECANARDS:
    return hprc::States_Boost; // schema has no PreCanards
  case CANARDS:
    return hprc::States_Canards;
  case PARACHUTE:
    return hprc::States_DrogueDescent;
  case ABORT:
    return hprc::States_Abort;
  case NUM_STATES:
    return hprc::States_Start;
  }
  return hprc::States_Start;
}

void initStateData(StateData *data) {
  data->startTime = millis();
  data->currentTime = 0;
  data->deltaTime = 0;
  data->lastLoopTime = 0;
  data->loopCount = 0;
};

void updateStateData(StateData *data) {
  long long now = millis();
  data->currentTime = now - data->startTime;
  data->deltaTime = now - data->lastLoopTime;
  data->lastLoopTime = now;
  data->loopCount++;
}

StateInitFunc initFuncs[NUM_STATES] = {};
StateLoopFunc loopFuncs[NUM_STATES] = {};

void initStateMap() {
  initFuncs[PRELAUNCH] = &prelaunchInit;
  initFuncs[PRECANARDS] = &precanardsInit;
  initFuncs[CANARDS] = &canardsInit;
  initFuncs[PARACHUTE] = &parachuteInit;
  initFuncs[ABORT] = &abortInit;

  loopFuncs[PRELAUNCH] = &prelaunchLoop;
  loopFuncs[PRECANARDS] = &precanardsLoop;
  loopFuncs[CANARDS] = &canardsLoop;
  loopFuncs[PARACHUTE] = &parachuteLoop;
  loopFuncs[ABORT] = &abortLoop;
}
#endif
