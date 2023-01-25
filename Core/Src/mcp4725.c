#include "mcp4725.h"

void general_call_reset(mcp4725_t* dac){
    uint8_t data = MCP4725_RESET_CMD;
    HAL_I2C_Master_Transmit(dac->hi2c, MCP4725_GENERAL_CALL_ADDR, &data, 1, 10);
}

void general_call_wakeup(mcp4725_t* dac){
    uint8_t data = MCP4725_WAKEUP_CMD;
    HAL_I2C_Master_Transmit(dac->hi2c, MCP4725_GENERAL_CALL_ADDR, &data, 1, 10);
}

void set_vout(mcp4725_t* dac, float32_t vout){
    uint8_t write_cmd = 0x02;
    uint8_t data[3];
    uint16_t dac_reg = (uint16_t)((vout/dac->vref)*4096.0);
    data[0] = MCP4725_CMD_WRITE_DAC_REG | MCP4725_PD_MODE_NORMAL;
    data[1] = dac_reg << 4;
    data[2] = (dac_reg << 4) & 0xF0;
    uint8_t addr_byte = dac->addr << 1; // R/W bit is 0 (write)
    HAL_I2C_Master_Transmit(dac->hi2c, addr_byte, data, 3, 10);
}