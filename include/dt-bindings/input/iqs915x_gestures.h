/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*
 * Private IQS915x gesture transport events.
 *
 * These values are intentionally separate from INPUT_EV_KEY so gesture actions
 * do not consume normal keyboard codes such as F13..F20. A firmware-side input
 * processor should consume IQS915X_INPUT_EV_GESTURE and map the gesture codes to
 * keymap positions or behaviors.
 */
#define IQS915X_INPUT_EV_GESTURE 0x7f

#define IQS915X_GESTURE_3F_LEFT  1
#define IQS915X_GESTURE_3F_UP    2
#define IQS915X_GESTURE_3F_DOWN  3
#define IQS915X_GESTURE_3F_RIGHT 4
#define IQS915X_GESTURE_4F_LEFT  5
#define IQS915X_GESTURE_4F_UP    6
#define IQS915X_GESTURE_4F_DOWN  7
#define IQS915X_GESTURE_4F_RIGHT 8
