#include <stdint.h>
#include <assert.h>
#include <string.h>

//int twi_init(void);

int twi_read_reg(uint8_t reg, uint8_t *data, uint32_t length);

int twi_write_reg(uint8_t reg, uint8_t *data, uint32_t length);
//int twi_write_reg2(uint8_t reg, uint8_t *p_data, uint32_t length);

int twi_read_mag_reg(uint8_t reg, uint8_t *data, uint32_t length);

int twi_enable(void);
int twi_disable(void);

