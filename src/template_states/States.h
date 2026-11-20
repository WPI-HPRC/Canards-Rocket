#pragma once

#include "States_generated.h"
enum StateID {
  PRELAUNCH,
  PRECANARDS,
  CANARDS,
  PARACHUTE,
  ABORT,
  NUM_STATES
};

hprc::States stateToTelemState(StateID state);

struct StateData {
  long long currentTime;
  long long deltaTime;
  long long loopCount;
  long long startTime;
  long long lastLoopTime;
};

struct Context;

void initStateData(StateData *data);

void updateStateData(StateData *data);

// ABORT
void *abortInit(StateData const *data);
StateID abortLoop(StateData const *data, Context *ctx, void *localData);

// PRELAUNCH
void *prelaunchInit(StateData const *data);
StateID prelaunchLoop(StateData const *data, Context *ctx, void *localData);

// PRECANARDS: launch until CANARDS_START_SPEED_METERS_PER_SEC
void *precanardsInit(StateData const *data);
StateID precanardsLoop(StateData const *data, Context *ctx, void *localData);

// CANARDS: canards active until apogee
void *canardsInit(StateData const *data);
StateID canardsLoop(StateData const *data, Context *ctx, void *localData);

// PARACHUTE: apogee onwards (drogue, main, recovery)
void *parachuteInit(StateData const *data);
StateID parachuteLoop(StateData const *data, Context *ctx, void *localData);
