/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: MIT
 *
 * IQS915x runtime power control and Zephyr Device PM adapter.
 */

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>

#include <iqs915x.h>
#include "iqs915x_power.h"
#include "iqs915x_regs.h"

#define IQS915X_POWER_TRANSITION_WAIT_MS 250

static int iqs915x_request_enabled(const struct device *dev, bool enabled,
                                   bool wait_for_completion)
{
  struct iqs915x_data *data = dev->data;
  atomic_val_t previous;
  bool already_stable;

  already_stable = ((atomic_get(&data->requested_enabled) != 0) == enabled) &&
                   data->initialized && !data->active_pending &&
                   !data->lp2_pending && data->work_state == WORK_READ_DATA &&
                   (data->enabled == enabled);

  if (already_stable) {
    return 0;
  }

  if (wait_for_completion) {
    k_sem_reset(&data->transition_sem);
    atomic_set(&data->transition_result, -EINPROGRESS);
  }

  /* Close the output gate before waking the core state machine. This also
   * handles repeated requests while an earlier transition is settling. */
  atomic_clear(&data->output_enabled);

  while (true) {
    previous = atomic_get(&data->requested_enabled);
    if ((previous != 0) == enabled) {
      break;
    }

    if (atomic_cas(&data->requested_enabled, previous, enabled ? 1 : 0)) {
      break;
    }
  }

  atomic_inc(&data->request_generation);
  k_sem_give(&data->rdy_sem);

  if (!wait_for_completion) {
    return 0;
  }

  int ret = k_sem_take(&data->transition_sem,
                       K_MSEC(IQS915X_POWER_TRANSITION_WAIT_MS));
  if (ret < 0) {
    return -ETIMEDOUT;
  }

  return atomic_get(&data->transition_result);
}

#if IS_ENABLED(CONFIG_PM_DEVICE)
int iqs915x_pm_action(const struct device *dev, enum pm_device_action action)
{
  struct iqs915x_data *data = dev->data;

  switch (action) {
  case PM_DEVICE_ACTION_SUSPEND:
    data->pm_saved_enabled = atomic_get(&data->requested_enabled) != 0;
    return iqs915x_request_enabled(dev, false, true);
  case PM_DEVICE_ACTION_RESUME:
    return iqs915x_request_enabled(dev, data->pm_saved_enabled, true);
  default:
    return -ENOTSUP;
  }
}
#endif

int iqs915x_set_enabled(const struct device *dev, bool enabled)
{
  return iqs915x_request_enabled(dev, enabled, false);
}

bool iqs915x_get_enabled(const struct device *dev)
{
  struct iqs915x_data *data = dev->data;
  return atomic_get(&data->requested_enabled) != 0;
}
