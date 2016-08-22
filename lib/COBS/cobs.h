#ifndef _COBS_H_
#define _COBS_H_

#include <stdint.h>
#include <stddef.h>


/*
 * COBS_SYMBOL - any byte that will be epsend in encoded data
 *
 * */
//~ #define COBS_SYMBOL 'L'
#define COBS_SYMBOL 0

#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif

size_t cobs_encode(const uint8_t* source, size_t size, uint8_t* destination);
size_t cobs_decode(const uint8_t* source, size_t size, uint8_t* destination);
size_t cobs_get_encoded_buffer_size(size_t sourceSize);

#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _COBS_H_ */
