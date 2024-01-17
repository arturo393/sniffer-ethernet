#include "lm75.h"

HAL_StatusTypeDef lm75_init(void ) {
	uint8_t cmd[2];
	cmd[0] = LM75_Conf;
	cmd[1] = 0x0;
    HAL_StatusTypeDef res;
	res = HAL_I2C_Master_Transmit(&hi2c1, LM75_ADDR<<1, cmd,2,50);

	return res;

}

uint16_t lm75_read(void) {
	uint8_t cmd[2];
	float result = 0;
	cmd[0] = LM75_Temp;
    HAL_StatusTypeDef res;
    res = HAL_I2C_Master_Transmit(&hi2c1, LM75_ADDR<<1, cmd,1,50);
	if(res != HAL_OK)
		return result;
    res = HAL_I2C_Master_Receive(&hi2c1, LM75_ADDR<<1 | 1, cmd,2,50);
	result = (float) ((cmd[0] << 8) | cmd[1]) / 256.0f;
	return result;
}
