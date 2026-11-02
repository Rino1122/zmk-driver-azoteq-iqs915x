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

#define IQS915X_POWER_TRANSITION_WAIT_MS 2000

static void iqs915x_set_pm_suspended(struct iqs915x_data *data, bool suspended)
{
  k_mutex_lock(&data->settings_lock, K_FOREVER);
  if (suspended) {
    atomic_set(&data->pm_suspended, 1);
  } else {
    atomic_clear(&data->pm_suspended);
  }
  k_mutex_unlock(&data->settings_lock);
}

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

  if (already_stable && !wait_for_completion) {
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

  atomic_set(&data->mode_transitioning, 1);
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
static int iqs915x_pm_action_locked(const struct device *dev, enum pm_device_action action)
{
  struct iqs915x_data *data = dev->data;

  switch (action) {
  case PM_DEVICE_ACTION_SUSPEND:
    data->pm_saved_enabled = atomic_get(&data->requested_enabled) != 0;
    data->pm_saved_mode = atomic_get(&data->requested_mode);
    atomic_set(&data->requested_mode, IQS915X_POWER_SUSPEND);
    iqs915x_set_pm_suspended(data, true);
    {
      int ret = iqs915x_request_enabled(dev, false, true);
      if (ret < 0) {
        iqs915x_set_pm_suspended(data, false);
        /* SUSPEND failed, so Zephyr leaves this device ACTIVE and will not
         * call RESUME during rollback. Restore the original intent ourselves.
         * A new generation supersedes any late LP2 completion; recovery is
         * asynchronous so the original failure is returned without another
         * two-second wait. The caller must keep I2C available on failure. */
        atomic_set(&data->requested_mode, data->pm_saved_mode);
        (void)iqs915x_request_enabled(dev, data->pm_saved_enabled, false);
      }
      return ret;
    }
  case PM_DEVICE_ACTION_RESUME:
    {
      iqs915x_set_pm_suspended(data, false);
      atomic_set(&data->requested_mode, data->pm_saved_mode);
      int ret = iqs915x_request_enabled(dev, data->pm_saved_enabled, true);
      return ret;
    }
  default:
    return -ENOTSUP;
  }
}

int iqs915x_pm_action(const struct device *dev, enum pm_device_action action)
{
  struct iqs915x_data *data = dev->data;
  if (atomic_get(&data->lifecycle) >= IQS915X_ABSENT) return 0;
  k_mutex_lock(&data->power_lock, K_FOREVER);
  int ret = iqs915x_pm_action_locked(dev, action);
  k_mutex_unlock(&data->power_lock);
  return ret;
}
#endif

int iqs915x_set_mode(const struct device *dev, enum iqs915x_mode mode)
{
  if (!dev || !device_is_ready(dev)) return -ENODEV;
  if ((unsigned)mode > IQS915X_POWER_SUSPEND) return -EINVAL;
  struct iqs915x_data *data = dev->data;
  if (atomic_get(&data->lifecycle) >= IQS915X_ABSENT) {
    /* Record user intent for the local error LED; never retry a latched IC. */
    atomic_set(&data->requested_mode, mode);
    atomic_set(&data->requested_enabled, mode == IQS915X_POWER_ACTIVE);
    return 0;
  }
  if (k_mutex_lock(&data->power_lock, K_NO_WAIT) < 0) return -EBUSY;
  int ret = -EBUSY;
  if (!atomic_get(&data->pm_suspended)) {
    if (atomic_get(&data->requested_mode) == (atomic_val_t)mode &&
        mode != IQS915X_POWER_ACTIVE && !atomic_get(&data->mode_transitioning) && data->initialized) {
      ret = 0;
    } else {
      atomic_set(&data->requested_mode, mode);
      atomic_set(&data->mode_transitioning, 1);
      /* Force a new generation even when both LP2 and Suspend disable output. */
      atomic_clear(&data->output_enabled);
      atomic_set(&data->requested_enabled, mode == IQS915X_POWER_ACTIVE);
      atomic_inc(&data->request_generation);
      k_sem_give(&data->rdy_sem);
      ret = 0;
    }
  }
  k_mutex_unlock(&data->power_lock);
  return ret;
}

int iqs915x_set_enabled(const struct device *dev, bool enabled)
{
  return iqs915x_set_mode(dev, enabled ? IQS915X_POWER_ACTIVE : IQS915X_POWER_LP2);
}

int iqs915x_get_status(const struct device *dev, struct iqs915x_status *status)
{
  if (!dev || !device_is_ready(dev) || !status) return -ENODEV;
  struct iqs915x_data *data = dev->data;
  *status = (struct iqs915x_status) {
    .lifecycle = atomic_get(&data->lifecycle),
    .requested_mode = atomic_get(&data->requested_mode),
    .actual_mode = atomic_get(&data->actual_mode),
    .ati_sequence = atomic_get(&data->ati_sequence),
    .ati_error = atomic_get(&data->ati_error),
    .transitioning = atomic_get(&data->mode_transitioning),
    .last_error = atomic_get(&data->last_error),
    .generation = atomic_get(&data->request_generation),
  };
  return 0;
}

bool iqs915x_get_enabled(const struct device *dev)
{
  struct iqs915x_data *data = dev->data;
  return atomic_get(&data->requested_enabled) != 0;
}
