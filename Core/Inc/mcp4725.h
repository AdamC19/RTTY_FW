#ifndef _MCP4725_H_
#define _MCP4725_H_

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#define MCP4725_GENERAL_CALL_ADDR  0x00
#define MCP4725_RESET_CMD   0x06
#define MCP4725_WAKEUP_CMD  0x09
#define MCP4725_CMD_WRITE_DAC_REG (0x02 << 5)
#define MCP4725_PD_MODE_NORMAL  0x00
#define MCP4725_PD_MODE_1K      (0x01 << 1)
#define MCP4725_PD_MODE_100K    (0x02 << 1)
#define MCP4725_PD_MODE_500K    (0x03 << 1)

typedef struct mcp4725_struct {
    I2C_HandleTypeDef* hi2c;
    uint8_t addr;
    float32_t vref;
}mcp4725_t;

void general_call_reset(mcp4725_t* dac);
void general_call_wakeup(mcp4725_t* dac);
void set_vout(mcp4725_t* dac, float32_t vout);

#endif