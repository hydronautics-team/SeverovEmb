#ifndef MOVING_AVERAGE_H_
#define MOVING_AVERAGE_H_

#include <stdint.h>

#define MOVING_AVERAGE_MAX_ORDER 50

typedef struct
{
	float data[MOVING_AVERAGE_MAX_ORDER];
	uint8_t order;

	uint8_t current_element;
	float average;
} Moving_Average_filter;

void movingAverageInit(Moving_Average_filter* filter, uint8_t order);
float movingAverageIterate(Moving_Average_filter* filter, float value);

#endif
