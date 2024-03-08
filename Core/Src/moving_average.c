#include "moving_average.h"

void movingAverageInit(Moving_Average_filter* filter, uint8_t order)
{
	filter->order = order;
	for(uint8_t i = 0; i<MOVING_AVERAGE_MAX_ORDER; i++)
		filter->data[i] = 0;
	filter->current_element = 0;
	filter->average = 0;
}

float movingAverageIterate(Moving_Average_filter* filter, float value)
{
	int32_t last_value = filter->data[filter->current_element];
	filter->data[filter->current_element] = value;
	filter->average = (filter->average * filter->order - last_value + value) / (float)filter->order;
	filter->current_element = (filter->current_element+1)%filter->order;
	return filter->average;
}
