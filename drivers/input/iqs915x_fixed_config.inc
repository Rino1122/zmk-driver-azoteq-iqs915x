/* SPDX-License-Identifier: MIT */
/* Verify compiled settings on cold and warm init. No runtime override API. */
static int iqs915x_check_fixed_sensor_config(const struct device *dev) {
 const struct iqs915x_config *c=dev->config;
 uint8_t actual[8],settings;uint16_t period;
 int rc=iqs915x_read_block(dev,0x11EA,actual,sizeof(actual));if(rc)return rc;
 for(unsigned i=0;i<8;i++)if((i<5||i==7)&&actual[i]!=c->init_data[0x11EA-IQS915X_INIT_DATA_BASE_ADDR+i])return 1;
 rc=iqs915x_read_block(dev,IQS915X_TRACKPAD_SETTINGS,&settings,1);if(rc)return rc;
 if((settings&(BIT(3)|BIT(4)))!=(TRACKPAD_SETTINGS_0_0&(BIT(3)|BIT(4))))return 1;
 rc=iqs915x_read_reg16(dev,IQS915X_ACTIVE_MODE_REPORT_RATE,&period);if(rc)return rc;
 return period!=c->report_rate_ms;
}
