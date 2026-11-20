#define TEMPLATE_STATES_OVERRIDE
#include "../State.h"
#include "StateMachineConstants.h"

void *precanardsInit(StateData const *data) { return nullptr; }

StateID precanardsLoop(StateData const *data, Context *ctx, void *_localData) {
    // TODO: hold canards centred

    // ctx->axialVelocity is integrated in velocityLoop() in main.cpp
    if (ctx->axialVelocity >= CANARDS_START_SPEED_METERS_PER_SEC) {
        return CANARDS;
    }

    return PRECANARDS;
}
