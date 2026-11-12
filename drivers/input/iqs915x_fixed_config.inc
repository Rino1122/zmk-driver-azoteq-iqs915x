/* SPDX-License-Identifier: MIT */
/* Verify static export settings on cold and warm init. */
static int iqs915x_check_fixed_sensor_config(const struct device *dev)
{
  const struct iqs915x_config *config = dev->config;
  uint8_t filter[8];
  int ret = iqs915x_read_block(dev, 0x11EA, filter, sizeof(filter));
  if (ret) { return ret; }
  for (unsigned i = 0; i < sizeof(filter); i++) {
    if ((i < 5 || i == 7) && filter[i] != config->init_data[0x11EA - IQS915X_INIT_DATA_BASE_ADDR + i]) {
      return 1;
    }
  }

  static const uint16_t registers[] = {
    IQS915X_TRACKPAD_SETTINGS,
    IQS915X_ACTIVE_MODE_REPORT_RATE,
    IQS915X_LP2_MODE_REPORT_RATE,
    IQS915X_OTHER_SETTINGS,
    IQS915X_SINGLE_FINGER_GESTURES_ENABLE,
    IQS915X_TWO_FINGER_GESTURES_ENABLE,
  };
  for (unsigned i = 0; i < ARRAY_SIZE(registers); i++) {
    uint16_t value;
    ret = iqs915x_read_reg16(dev, registers[i], &value);
    if (ret) { return ret; }
    unsigned offset = registers[i] - IQS915X_INIT_DATA_BASE_ADDR;
    uint16_t expected = config->init_data[offset] | ((uint16_t)config->init_data[offset + 1] << 8);
    if (value != expected) { return 1; }
  }

  /* The next byte in the export is REF_UPDATE_TIME. Compare the retry byte. */
  uint8_t retry;
  ret = iqs915x_read_block(dev, IQS915X_REATI_RETRY_TIME, &retry, sizeof(retry));
  if (ret) { return ret; }
  return retry != config->init_data[IQS915X_REATI_RETRY_TIME - IQS915X_INIT_DATA_BASE_ADDR];
}
