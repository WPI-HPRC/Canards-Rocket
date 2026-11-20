#pragma once

constexpr float LAUNCH_TRHESHOLD = 5;

constexpr float COAST_THRESHOLD = 0.3f;

constexpr float MAIN_MIN_VEL = 5;

constexpr float MAIN_MAX_VEL = 9;

constexpr float BOOST_TIMEOUT = 0.0f;

constexpr float COAST_TIMEOUT = 0.0f;

constexpr float DROGUE_DESCENT_TIMEOUT = 0.0f;

constexpr float MAIN_DESCENT_TIMEOUT = 0.0f;

constexpr float CANARDS_START_SPEED_METERS_PER_SEC = 50.0f;

constexpr float GRAVITY_MPS2 = 9.80665f;

// LQR gains assume a 200 Hz (5 ms) step
constexpr unsigned long CONTROL_PERIOD_MS = 5;

// apogee: baro height this far below max height [m] for APOGEE_DEBOUNCE_MS
// TODO: tune
constexpr float APOGEE_DROP_M = 3.0f;
constexpr long APOGEE_DEBOUNCE_MS = 300;
