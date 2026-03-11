#ifndef BLUEJAY_FLASH_H
#define BLUEJAY_FLASH_H
#include <stdint.h>
#include <QString>
#include <QByteArray>
#include <stdint.h>

#define BLUEJAY_ARRAY_SIZE  128
#define BLUEJAY_MAX_PAIRS    62   /* (128 - 4) / 2 */

/**
 * blheli32_to_bluejay_array()
 *
 * Convert BLHeli32 music notation to the 128-byte Bluejay ESC flash array.
 *
 * @param notation   BLHeli32 notation as a QString, e.g. "A#5 8 P8 C6 4 D6 4"
 * @param bpm        Tempo in BPM (e.g. 120)
 * @param out        Output buffer — must be exactly BLUEJAY_ARRAY_SIZE (128) bytes
 *
 * @return  Number of (T4,T3) pairs written (≤ 62), or -1 on parse error.
 */
int blheli32_to_bluejay_array(const QString &notation, int bpm, uint8_t out[BLUEJAY_ARRAY_SIZE]);

/**
 * bluejay_array_to_c_literal()
 *
 * Print the 128-byte array as a C uint8_t array literal to stdout.
 *
 * @param arr       128-byte array from blheli32_to_bluejay_array()
 * @param var_name  C variable name to use
 */
void bluejay_array_to_c_literal(const uint8_t arr[BLUEJAY_ARRAY_SIZE], const char *var_name);

#endif

