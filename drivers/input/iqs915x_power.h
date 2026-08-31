/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/util.h>

#if IS_ENABLED(CONFIG_PM_DEVICE)
int iqs915x_pm_action(const struct device *dev, enum pm_device_action action);
#define IQS915X_PM_DEVICE_DEFINE(n) PM_DEVICE_DT_INST_DEFINE(n, iqs915x_pm_action)
#define IQS915X_PM_DEVICE_GET(n) PM_DEVICE_DT_INST_GET(n)
#else
#define IQS915X_PM_DEVICE_DEFINE(n)
#define IQS915X_PM_DEVICE_GET(n) NULL
#endif
