/** ***************************************************************************
 * @file crc.c  functions for computing CRC's
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

 #include "crc.h"

static uint32_t const BITS_PER_BYTE	= 8U;

#define ONE_BYTE	(BITS_PER_BYTE)
#define TWO_BYTES	(2 * BITS_PER_BYTE)
#define THREE_BYTES	(3 * BITS_PER_BYTE)

static uint32_t const BYTE_0 =  0x000000FFU;
static uint32_t const BYTE_1 =  0x0000FF00U;
static uint32_t const BYTE_2 =	0x00FF0000U;
static uint32_t const BYTE_3 =	0xFF000000U;

static uint8_t const  MSB	= 	0x80U;

static uint8_t const  CRC_CCITT_POLY [] = { 0x10U, 0x21U };
static uint8_t const  CRC_32_POLY []    = { 0xEDU, 0xB8U, 0x83U, 0x20U };

/*******************************************
 * @brief 
 * 
 * @param poly ==
 * @param polyLength ==
 * @param seed ==
 * @param data ==
 * @param dataLength ==
 * @param crc ==
********************************************/
static void Crc (
				 uint8_t const poly [],
                 uint8_t const polyLength,
                 uint8_t const seed [],
                 uint8_t const data [],
                 uint16_t const dataLength,
                 uint8_t       crc [])
{
	uint32_t dataIndex;
	uint32_t  crcIndex;
	uint32_t  bitCount;

	if(polyLength == 0U){
		// nothing to do
		return;
	}
	/// load seed
	for (crcIndex = 0U; crcIndex < polyLength; ++crcIndex) {
		crc[crcIndex] = seed[crcIndex];
	}

	/// step through all bytes of data
	for (dataIndex = 0U; dataIndex < dataLength; ++dataIndex) {
		/// next data byte is XORed into the most-significant byte of the CRC
		crc[0] ^= data[dataIndex];

		/// index through all bits of current data byte
		for (bitCount = 0U; bitCount < BITS_PER_BYTE; ++bitCount) {
			/// top CRC bit = 1 -> apply poly to current CRC and shift CRC left
			if ((crc[0] & MSB) != 0U) {
				/// left-shift bits in all bytes of CRC except for
                /// least-significant byte
				for (crcIndex = 0U; crcIndex < (polyLength - 1U); ++crcIndex) {
					/// shift in MSB from next least-significant byte to LSB of
                    /// current byte
					crc[crcIndex] = ((uint8_t)((crc[crcIndex] << 1U) & 0xFFU)) |
                                    ((crc[crcIndex + 1U] & MSB) >> (BITS_PER_BYTE - 1U));

					/// XOR poly with current CRC value
					crc[crcIndex] ^= poly[crcIndex];
				}

				/// left-shift least-significant byte
				crc[crcIndex] = (uint8_t)((crc[crcIndex] << 1U) & 0xFFU);
				crc[crcIndex] ^= poly[crcIndex]; /// poly XOR current CRC value
			}
			else {	/// top CRC bit = 0 -> shift CRC left only
				/// left-shift bits in all bytes of CRC except for
                /// least-significant byte
				for (crcIndex = 0U; crcIndex < (polyLength - 1U); ++crcIndex) {
					/// shift in MSB from next least-significant byte to LSB of
                    /// current byte
					crc[crcIndex] = ((uint8_t)((crc[crcIndex] << 1U) & 0xFFU)) |
                                    ((crc[crcIndex + 1U] & MSB) >> (BITS_PER_BYTE - 1U));
				}
				/// left-shift least-significant byte
				crc[crcIndex] = (uint8_t)((crc[crcIndex] << 1U) & 0xFFU);
			}
		}
	}
}

/** ****************************************************************************
 * @brief split the input type into bytes
 * @param type  [in] - input
 * @param bytes [out] - shifted and masked bytes
 *  
 ******************************************************************************/
void CrcCcittTypeToBytes (CrcCcittType const type,
                          uint8_t bytes [])
{
	bytes[0] = (uint8_t)((type >> ONE_BYTE) & BYTE_0);
	bytes[1] = (uint8_t)(type & 0xFFU);
}

/** ****************************************************************************
 * @brief split the into bytes
 * @param bytes [out] - shifted andmasked bytes
 * @return
 ******************************************************************************/
CrcCcittType BytesToCrcCcittType (uint8_t const bytes [])
{
	uint16_t tmp;

	tmp  = bytes[0];
	tmp <<= 8U;
	tmp |= bytes[1];
	
	return (CrcCcittType)tmp;

}

/** ****************************************************************************
 * @brief split the input type into bytes
 * @param type  [in] - input
 * @param bytes [out] - shifted bytes
 *  
 ******************************************************************************/
void Crc32TypeToBytes (Crc32Type const type,
                       uint8_t   bytes [])
{
	bytes[0] = (uint8_t)((type >> 24U) & BYTE_0);
	bytes[1] = (uint8_t)((type >> 16U)   & BYTE_0);
	bytes[2] = (uint8_t)((type >> 8U)    & BYTE_0);
	bytes[3] = (uint8_t)(type                  & BYTE_0);
}

/** ****************************************************************************
 * @brief convert individual bytes to 32 bit type
 * @param bytes [out] - pointer to the converted
 * @return
 ******************************************************************************/
Crc32Type BytesToCrc32Type (uint8_t const bytes [])
{
	return ((((Crc32Type)bytes[0] << 24U) & BYTE_3) |
		    (((Crc32Type)bytes[1] << 16U) & BYTE_2) |
		    (((Crc32Type)bytes[2] << 8U)  & BYTE_1) |
		     ((Crc32Type)bytes[3]                 & BYTE_0));
}

/** ****************************************************************************
 * @brief perform the crc calculations
 * @param data   [out]  - pointer to the input data
 * @param length [out]  - of the input data
 * @param seed   [out]  - starting value for the calc
 * @return crc
 ******************************************************************************/
CrcCcittType CrcCcitt (uint8_t   const    data[],
                       uint16_t  const   length,
                       CrcCcittType const seed)
{
	uint8_t crc[CRC_CCITT_LENGTH];

	CrcCcittTypeToBytes(seed, crc);
	Crc(CRC_CCITT_POLY,
        CRC_CCITT_LENGTH,
        crc,
        data,
        length,
        crc);

	return (BytesToCrcCcittType(crc));
}

/** ****************************************************************************
 * @brief perform the crc calculations
 * @param data   [out] - pointer to the input data
 * @param length [out] - of the input data
 * @param seed [out] - starting value for the calc
 * @return crc
 ******************************************************************************/
Crc32Type Crc32 (uint8_t const   data[],
                 uint16_t const  length,
                 Crc32Type const seed)
{
	uint8_t crc[4];

	Crc32TypeToBytes(seed, crc);
	Crc(CRC_32_POLY,
        4U,
        crc,
        data,
        length,
        crc);

	return (BytesToCrc32Type(crc));
}

/** ****************************************************************************
 * @brief   : CrcCcittType helper for CRCing 16 bit values
 * @param v     [in] - input value
 * @param seed  [in] - crc seed
 * @return rx - data read at the address crc.c, handle_packet.c
 ******************************************************************************/
uint16_t initCRC_16bit(uint16_t const v, uint16_t const seed)
{
    uint8_t c[2];

    /// unpack 16 bit into array of 8's
    c[0] = (uint8_t)((v >> 8U) & 0xFFU);
    c[1] = (uint8_t)(v & 0x00FFU);
    return CrcCcitt(c, 2U, seed);
}
