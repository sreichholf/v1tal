#ifndef V1SEG_CRC32_H
#define V1SEG_CRC32_H

/*
 * Named constants for the CRC-32 used by the map blobs and the last_segmap
 * sidecar files.
 *
 * This is the standard CRC-32 (IEEE 802.3 / ITU-T V.42) -- the same value
 * produced by Python's zlib.crc32() and by the host tools, so a CRC written
 * here verifies against them and vice versa.
 *
 * The table-driven loop below walks the polynomial least-significant-bit
 * first.  That path uses the *reflected* (bit-reversed) form of the
 * polynomial, not the textbook 0x04C11DB7.  The shift register is preset to
 * all-ones and the final result is XORed with all-ones.
 *
 * Spelled out here so no reader has to recognize a bare 0xEDB88320.
 */
#define CRC32_POLYNOMIAL_REFLECTED 0xEDB88320u  /* 0x04C11DB7, bit-reversed   */
#define CRC32_INITIAL_VALUE        0xFFFFFFFFu  /* shift-register preset       */
#define CRC32_FINAL_XOR            0xFFFFFFFFu  /* XOR applied to the result   */

#endif /* V1SEG_CRC32_H */
