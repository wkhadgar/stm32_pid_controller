/**
 * @file luts.h
 * @author Paulo Santos (pauloxrms@gmail.com)
 * @brief Declara o módulo de look up tables dos sensores.
 * @version 0.1
 * @date 02-03-2025
 *
 * @copyright Copyright (c) 2024 Paulo Santos
 *
 */

#ifndef LUTS_H
#define LUTS_H

#define ADC_RESOLUTION_BITS 12
#define ADC_RESOLUTION (1 << ADC_RESOLUTION_BITS)

enum sensor_id {
  SENSOR_A = 0,
  SENSOR_B,

  SENSOR_AMOUNT,
};

extern const float adc_luts[SENSOR_AMOUNT][ADC_RESOLUTION];

#endif /* LUTS_H */
