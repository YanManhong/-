/*
 * Copyright (c) 2024 HiSilicon Technologies CO., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AHT20_H
#define AHT20_H

#include <stdint.h>

uint32_t aht20_calibrate(void);

uint32_t aht20_start_measure(void);

uint32_t aht20_get_measure_result(float *temp, float *humi);

#endif // AHT20_H