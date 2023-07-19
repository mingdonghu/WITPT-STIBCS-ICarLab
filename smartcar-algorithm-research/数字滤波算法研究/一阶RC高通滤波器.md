> 通高频信息，滤低频信息

```c
#include <stdint.h>

typedef struct
{
     int16_t  input[2];
     int16_t  output[2];
     int32_t  filter_Tf;		
     int32_t  filter_Ts;
     int32_t  ky;
} High_FilterHandleTypedef;

/**
 * @brief: 一阶RC高通滤波器 
 * @param {*}
 * @retval: 
 */
void HighFilterInit(High_FilterHandleTypedef *v)
{	
     v->ky = v->filter_Tf*1024/(v->filter_Ts + v->filter_Tf);
}

int16_t HighFilterCalc(High_FilterHandleTypedef *v)
{
	int32_t tmp = 0;

	tmp = ((int32_t)v->ky*v->output[1] + v->ky*(v->input[0] - v->input[1]))/1024;
	if(tmp>32767){
		tmp = 32767;
	}
	
	if( tmp < -32768){
		tmp = -32768;
	}
	
    v->output[0] = (int16_t)tmp;
    v->output[1] = v->output[0];
    v->input[1] = v->input[0];
    
	return v->output[0];
}

//user opt

static High_FilterHandleTypedef HighFiters;

//tf= 100, tf=200;视具体处理的信号特征来确定
void HighFilters_InitParam(int32_t tf, int32_t ts)
{
    HighFiters.filter_Tf = tf;
    HighFiters.filter_Ts = ts;
    HighFiters.output[0] = 0;
    HighFiters.output[1] = 0;
    HighFiters.input[0] = 0;
    HighFiters.input[1] = 0;
    HighFilterInit(&HighFiters);
}

void HighFilters_Opt(int16_t data_in, int16_t *data_out)
{
	HighFiters.output[0] = data_in;
    *data_out = HighFilterCalc(&HighFiters);
}

```
