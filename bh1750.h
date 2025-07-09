#ifndef __BH1750_H__
#define __BH1750_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void bh1750_init(void);
void bh1750_SendCMD(uint8_t cmd);
void bh1750_ReadData(void);
uint16_t bh1750_GetLightIntensity(void);

#ifdef __cplusplus
}
#endif

#endif /* __BH1750_H__ */
