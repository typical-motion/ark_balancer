#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "crc32.h"

#define CRCPOLYNOMIAL 0xEDB88320
static uint32_t *crc32_table = NULL;
static uint32_t *create_table(uint32_t *crctable)
{
	uint32_t crcreg;
	int i, j;
	for (i = 0; i < 256; ++i)
	{
		crcreg = i;
		for (j = 0; j < 8; ++j)
		{
			if ((crcreg & 1) != 0)
			{
				crcreg = CRCPOLYNOMIAL ^ (crcreg >> 1);
			}

			else
			{
				crcreg >>= 1;
			}
		}
		crctable[i] = crcreg;
	}
	return crctable;
}

uint32_t crc32(uint8_t *input, uint16_t len)
{
	uint8_t *p = input;
	uint32_t crcstart = 0xFFFFFFFF;
	if (crc32_table == NULL)
	{
		crc32_table = create_table((uint32_t *)malloc(sizeof(uint32_t) * 256));
	}
	while (len--)
	{
		crcstart = (crcstart >> 8) ^ crc32_table[(crcstart ^ *p++) & 0xFF];
	}
	return ~crcstart;
}