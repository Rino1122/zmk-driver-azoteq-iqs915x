/* SPDX-License-Identifier: MIT */
/* Return 1 for a mismatch, 0 for a match, or a negative read error. */
static int iqs915x_check_fixed_sensor_config(const struct device *dev,
                                            const char *phase)
{
  const struct iqs915x_config *config = dev->config;
  const uint16_t filter_base = 0x11EA;
  uint8_t filter[8];
  int ret = iqs915x_read_block(dev, filter_base, filter, sizeof(filter));
  if (ret) { return ret; }
  for (unsigned i = 0; i < sizeof(filter); i++) {
    if (i == 5 || i == 6) { continue; }
    uint16_t reg = filter_base + i;
    uint8_t expected = config->init_data[reg - IQS915X_INIT_DATA_BASE_ADDR];
    if (filter[i] != expected) {
      LOG_WRN("Init: fixed sensor config mismatch (%s): filter reg=0x%04x "
              "expected=0x%02x observed=0x%02x", phase, reg, expected, filter[i]);
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
    if (value != expected) {
      LOG_WRN("Init: fixed sensor config mismatch (%s): reg=0x%04x "
              "expected=0x%04x observed=0x%04x", phase, registers[i], expected, value);
      return 1;
    }
  }

  /* The next byte in the export is REF_UPDATE_TIME. Compare the retry byte. */
  uint8_t retry;
  ret = iqs915x_read_block(dev, IQS915X_REATI_RETRY_TIME, &retry, sizeof(retry));
  if (ret) { return ret; }
  uint8_t expected = config->init_data[IQS915X_REATI_RETRY_TIME - IQS915X_INIT_DATA_BASE_ADDR];
  if (retry != expected) {
    LOG_WRN("Init: fixed sensor config mismatch (%s): Re-ATI retry time reg=0x%04x "
            "expected=0x%02x observed=0x%02x",
            phase, IQS915X_REATI_RETRY_TIME, expected, retry);
    return 1;
  }
  return 0;
}
