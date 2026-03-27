#pragma once
#include <stdint.h>
int iis_audio_read_16(int16_t *samples, int count);
void iis_audio_convert_16_to_8(const int16_t *src, uint8_t *dst, uint8_t len);
void iis_audio_init(void);