#define TEMPLATE_STATES_OVERRIDE
#include "../State.h"

void *parachuteInit(StateData const *data) { return nullptr; }

StateID parachuteLoop(StateData const *data, Context *ctx, void *_localData) {
    // TODO: drogue / main deployment, landing detection
    return PARACHUTE;
}
