#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "filterFP.h"
#include "Indices.h"
#include "filterAPI.h"

    // assuming maximum filter order is 5
	typedef struct {
		uint16_t cutOff;
		uint8_t  order;
		float32_t  num[6];
		float32_t  den[6];
	}bfw_filter_t;

	static float32_t  accArrIn[NUM_SENSOR_CHIPS][24]  = {0}, accArrOut[NUM_SENSOR_CHIPS][24] = {0};
	static float32_t  rateArrIn[NUM_SENSOR_CHIPS][24] = {0}, rateArrOut[NUM_SENSOR_CHIPS][24] = {0};
	static float32_t  accArrIn2[NUM_SENSOR_CHIPS][24]  = {0}, accArrOut2[NUM_SENSOR_CHIPS][24]  = {0};
	static float32_t  rateArrIn2[NUM_SENSOR_CHIPS][24] = {0}, rateArrOut2[NUM_SENSOR_CHIPS][24] = {0};

	static Buffer bfAccIn[NUM_SENSOR_CHIPS];
	static Buffer bfAccOut[NUM_SENSOR_CHIPS];
	static Buffer bfRateIn[NUM_SENSOR_CHIPS];
	static Buffer bfRateOut[NUM_SENSOR_CHIPS];

	static Buffer bfAccIn2[NUM_SENSOR_CHIPS];
	static Buffer bfAccOut2[NUM_SENSOR_CHIPS];
	static Buffer bfRateIn2[NUM_SENSOR_CHIPS];
	static Buffer bfRateOut2[NUM_SENSOR_CHIPS];


	bfw_filter_t const  bfw_2Hz   = { 2,   1, {0.00375502186654075,  0.00375502186654075 },   {1, -0.992489956266918 } };
	bfw_filter_t const  bfw_2Hz_nd  = { 2,   2, {1.41531349876767e-05,  2.83062699753535e-05, 1.41531349876767e-05},   {1, -1.99243345001763, 0.992490062557581} };
	bfw_filter_t const  bfw_5Hz   = { 5,   1, {0.00933520503578977,  0.00933520503578977 },   {1, -0.98132958992842 } };
	bfw_filter_t const  bfw_5Hz_nd  = { 5,   2, {8.79595077762413e-05, 0.000175919015552483, 8.79595077762413e-05 },   {1, -1.9809793941374,  0.9813312321685 } };
	bfw_filter_t const  bfw_10Hz  = { 10,  1, {0.0184993422547825 ,  0.0184993422547825  },   {1, -0.963001315490435} };
	bfw_filter_t const  bfw_10Hz_nd = { 10,  2, {0.000348554406863278, 0.000697108813726557, 0.000348554406863278},   {1, -1.96161999391752, 0.963014211544969} };
	bfw_filter_t const  bfw_20Hz  = { 20,  3, {4.97283152287031e-05, 0.000149184945686109, 0.000149184945686109, 4.97283152287031e-05 },   {1, -2.84926922721289, 2.70968498227483, -0.860017928540116 } };
	bfw_filter_t const  bfw_25Hz  = { 25,  3, {9.53869806556429e-05, 0.000286160941966929, 0.000286160941966929, 9.53869806556429e-05 },   {1, -2.8116113275724,  2.64055194830999, -0.828177524892344 } };
	bfw_filter_t const  bfw_40Hz  = { 40,  3, {0.000370471411572426, 0.00111141423471728,  0.00111141423471728,  0.000370471411572426 },   {1, -2.69874780967974, 2.44118287453215, -0.739471293559831 } };
	bfw_filter_t const  bfw_50Hz  = { 50,  3, {0.000698954177629413, 0.00209686253288824,  0.00209686253288824,  0.000698954177629413 },   {1, -2.62362686956658, 2.31480639307083,-0.685587890083219  } };

	const float32_t zeros[3] = { 0.0F, 0.0F, 0.0F };


	static void bf_Put(Buffer *bf, float32_t* d)
	{
		int32_t i;
		int32_t m = bf->m;              // row number
		int32_t n = bf->n;              // column number

		bf->i += 1;                 // move to the next position
		if (bf->full == 0)           // buffer is not full
		{
			bf->num = bf->i + 1;
			if (bf->i == n - 1)        // buffer become full
			{
				bf->full = 1;
			}
		}
		if (bf->i == n)                //buffer is full, overwrite the oldest data
		{
			bf->i = 0;
		}
		// put data into buffer - m samples
		for (i = 0; i < m; i++)
		{
			bf->d[i*n + bf->i] = d[i];
		}
	}

	static int32_t bf_Get(Buffer *bf, float32_t *d, int idx)
	{
		int32_t i;
		int32_t m;
		int32_t n;

		m = bf->m;
		n = bf->n;
		if (idx >= n)  // idx exceeds the max column number
		{
			for (i = 0; i < m; i++)
				d[i] = 0;
			return 0;
		}
		if (bf->full)// buffer is full
		{
			idx = bf->i - idx;
			if (idx < 0)
				idx += n;
			for (i = 0; i < m; i++)
				d[i] = bf->d[i*n + idx];
		}
		else//buffer is not full
		{
			idx = bf->i - idx;
			if (idx < 0)// idx exceeds the storage range
			{
				for (i = 0; i < m; i++)
					d[i] = 0;
				return 0;
			}
			else// idx within the storage range
			{
				for (i = 0; i < m; i++)
					d[i] = bf->d[i*n + idx];
			}
		}
		return 1;
	}

    
	int32_t Butterworth_filterInitFP(int32_t chip)
	{
        static int init[3] = {1, 1, 1};
        
        if(!init[chip]){
            return 0;
        }
        
        init[chip]         = 0;   
		
//      first cascade
		bfAccIn[chip].d    = &accArrIn[chip][0];
		bfAccIn[chip].m    = 3;
		bfAccIn[chip].n    = 1;
		bfAccIn[chip].i    = -1;
		bfAccIn[chip].full = 0;
		bfAccIn[chip].num  = 0;

		bfAccOut[chip].d    = &accArrOut[chip][0];
		bfAccOut[chip].m    = 3;
		bfAccOut[chip].n    = 1;
		bfAccOut[chip].i    = -1;
		bfAccOut[chip].full = 0;
		bfAccOut[chip].num  = 0;

		bfRateIn[chip].d    = &rateArrIn[chip][0];
		bfRateIn[chip].m    = 3;
		bfRateIn[chip].n    = 1;
		bfRateIn[chip].i    = -1;
		bfRateIn[chip].full = 0;
		bfRateIn[chip].num  = 0;

		bfRateOut[chip].d    = &rateArrOut[chip][0];
		bfRateOut[chip].m    = 3;
		bfRateOut[chip].n    = 1;
		bfRateOut[chip].i    = -1;
		bfRateOut[chip].full = 0;
		bfRateOut[chip].num  = 0;

//      second cascade
		bfAccIn2[chip].d    = &accArrIn2[chip][0];
		bfAccIn2[chip].m    = 3;
		bfAccIn2[chip].n    = 1;
		bfAccIn2[chip].i    = -1;
		bfAccIn2[chip].full = 0;
		bfAccIn2[chip].num  = 0;

		bfAccOut2[chip].d    = &accArrOut2[chip][0];
		bfAccOut2[chip].m    = 3;
		bfAccOut2[chip].n    = 1;
		bfAccOut2[chip].i    = -1;
		bfAccOut2[chip].full = 0;
		bfAccOut2[chip].num  = 0;

		bfRateIn2[chip].d    = &rateArrIn2[chip][0];
		bfRateIn2[chip].m    = 3;
		bfRateIn2[chip].n    = 1;
		bfRateIn2[chip].i    = -1;
		bfRateIn2[chip].full = 0;
		bfRateIn2[chip].num  = 0;

		bfRateOut2[chip].d    = &rateArrOut2[chip][0];
		bfRateOut2[chip].m    = 3;
		bfRateOut2[chip].n    = 1;
		bfRateOut2[chip].i    = -1;
		bfRateOut2[chip].full = 0;
		bfRateOut2[chip].num  = 0;


		return 0;
	}

	// Filter implemented in floating point domain
    static float32_t   tmp[6];
    static float32_t   tmpU[6];
	static float32_t   tmpY[6];
    
    int32_t Butterworth_filterFP(int32_t chip, uint8_t fAccel, int32_t freq, float32_t *dataIn, float32_t *dataOut)
	{
		const bfw_filter_t *pFilt = NULL;
		const  float *num;
		const  float *den;
		static int32_t   afreq[3] = { -1, -1, -1};
		static int32_t   gfreq[3] = { -1, -1, -1 };
		int         numLen;
		int         denLen;
		Buffer      *in;
		Buffer      *out;

		switch (freq) {
		case CUTOFF_FREQ_2HZ:
			pFilt = &bfw_2Hz;
			break;
		case CUTOFF_FREQ_5HZ:
			pFilt = &bfw_5Hz;
			break;
		case CUTOFF_FREQ_10HZ:
			pFilt = &bfw_10Hz;
			break;
		case CUTOFF_FREQ_20HZ:
			pFilt = &bfw_20Hz;
			break;
		case CUTOFF_FREQ_25HZ:
			pFilt = &bfw_25Hz;
			break;
		case CUTOFF_FREQ_40HZ:
			pFilt = &bfw_40Hz;
			break;
		case CUTOFF_FREQ_50HZ:
			pFilt = &bfw_50Hz;
			break;
		default:
			if (fAccel) {
				pFilt = &bfw_5Hz;
			}
			else {
				pFilt = &bfw_25Hz;
			}
			break;
		}

		if (fAccel) {
			if (afreq[chip] != freq) {
				afreq[chip]        = freq;
				bfAccIn[chip].n    = pFilt->order + 1;
//				bfAccIn[chip].n    =  1;
				bfAccIn[chip].i    = -1;
				bfAccIn[chip].full = 0;
				bfAccIn[chip].num  = 0;
				while (!bfAccIn[chip].full) {
					bf_Put(&bfAccIn[chip], (float32_t*)zeros);
				}
				bfAccOut[chip].n    = pFilt->order + 1;
//				bfAccOut[chip].n    =  1;
				bfAccOut[chip].i    = -1;
				bfAccOut[chip].full = 0;
				bfAccOut[chip].num  = 0;
				while (!bfAccOut[chip].full) {
					bf_Put(&bfAccOut[chip], (float32_t*)zeros);
				}
			}
		}
		else {
			if (gfreq[chip] != freq) {
				gfreq[chip]         = freq;
				bfRateIn[chip].n    = pFilt->order + 1;
				bfRateIn[chip].i    = -1;
				bfRateIn[chip].full = 0;
				bfRateIn[chip].num  = 0;
				while (!bfRateIn[chip].full) {
					bf_Put(&bfRateIn[chip], (float32_t*)zeros);
				}
				bfRateOut[chip].n = pFilt->order + 1;
				bfRateOut[chip].i = -1;
				bfRateOut[chip].full = 0;
				bfRateOut[chip].num = 0;
				while (!bfRateOut[chip].full) {
					bf_Put(&bfRateOut[chip], (float32_t*)zeros);
				}
			}
		}

		num    = &pFilt->num[0];
		den    = &pFilt->den[0];
		numLen = pFilt->order + 1;
		denLen = pFilt->order + 1;

		if (fAccel) {
			in  = &bfAccIn[chip];
			out = &bfAccOut[chip];
			bf_Put(&bfAccIn[chip], dataIn);
		}
		else {
			in = &bfRateIn[chip];
			out = &bfRateOut[chip];
			bf_Put(&bfRateIn[chip], dataIn);
		}

		// denlen and numlen is filter order
		// assuming one filter is always 0
		int m, nu, ny;
		int i, j;

		m = in->m;   // number of rows
		nu = in->n;   // number of columns in buff u   
		ny = out->n;  // number of columns in buff y

		//-------------buffer depth is less than filter depth, error-----
		if (nu < numLen || ny < denLen - 1) {
			return 0;
		}

		//-------------------------begin filter--------------------------
		// set tmpU and tmpY to 0
		for (i = 0; i < m; i++)
		{
			tmpU[i] = 0.0;
			tmpY[i] = 0.0;
		}
		// calcualte tmpU = a0*u[k] + ... + an*u[k-n], u[k] is the latest (index is 0) data in u
		for (j = 0; j < numLen; j++)
		{
			if (!bf_Get(in, tmp, j)) // unavailable data are replaced by 0
			{
				for (i = 0; i < m; i++)
					tmp[i] = 0;
			}

			for (i = 0; i < m; i++)
			{
				float t = num[j];
				tmpU[i] +=  t * tmp[i];
			}
		}

		// calcualte b1*y[k-1] + ... + bn*y[k-n], y[k-1] is the latest data in y
		for (j = 0; j < denLen - 1; j++)
		{
			if (!bf_Get(out, tmp, j)) // unavailable data are replaced by 0
			{
				for (i = 0; i < m; i++)
					tmp[i] = 0;
			}
			for (i = 0; i < m; i++)
			{
				tmpY[i] += den[j + 1] * tmp[i]; // den[j+1], den[j]=1.0
			}

		}
		for (i = 0; i < m; i++)
		{
			tmp[i] = (tmpU[i] - tmpY[i]) / den[0];
			dataOut[i] = tmp[i];
		}
		// filtered data are put into out
		bf_Put(out, tmp);

		return 1;
	}

    int32_t Butterworth_filterFP2(int32_t chip, uint8_t fAccel, int32_t freq, float32_t *dataIn, float32_t *dataOut)
	{
		const bfw_filter_t *pFilt = NULL;
		const  float     *num;
		const  float     *den;
		static int32_t   afreq[3] = { -1, -1, -1};
		static int32_t   gfreq[3] = { -1, -1, -1 };
		int         numLen;
		int         denLen;
		Buffer      *in;
		Buffer      *out;

		switch (freq) {
		case CUTOFF_FREQ_2HZ:
			pFilt = &bfw_2Hz_nd;
			break;
		case CUTOFF_FREQ_5HZ:
			pFilt = &bfw_5Hz_nd;
			break;
		case CUTOFF_FREQ_10HZ:
			pFilt = &bfw_10Hz_nd;
			break;
		default:
            return 1;
		}

		if (fAccel) {
			if (afreq[chip] != freq) {
				afreq[chip]         = freq;
				bfAccIn2[chip].n    = pFilt->order + 1;
				bfAccIn2[chip].i    = -1;
				bfAccIn2[chip].full = 0;
				bfAccIn2[chip].num  = 0;
				while (!bfAccIn2[chip].full) {
					bf_Put(&bfAccIn2[chip], (float32_t*)zeros);
				}
				bfAccOut2[chip].n    = pFilt->order + 1;
				bfAccOut2[chip].i    = -1;
				bfAccOut2[chip].full = 0;
				bfAccOut2[chip].num  = 0;
				while (!bfAccOut2[chip].full) {
					bf_Put(&bfAccOut2[chip], (float32_t*)zeros);
				}
			}
		}
		else {
			if (gfreq[chip] != freq) {
				gfreq[chip]         = freq;
				bfRateIn2[chip].n    = pFilt->order + 1;
				bfRateIn2[chip].i    = -1;
				bfRateIn2[chip].full = 0;
				bfRateIn2[chip].num  = 0;
				while (!bfRateIn2[chip].full) {
					bf_Put(&bfRateIn2[chip], (float32_t*)zeros);
				}
				bfRateOut2[chip].n = pFilt->order + 1;
				bfRateOut2[chip].i = -1;
				bfRateOut2[chip].full = 0;
				bfRateOut2[chip].num = 0;
				while (!bfRateOut2[chip].full) {
					bf_Put(&bfRateOut2[chip], (float32_t*)zeros);
				}
			}
		}

		num    = &pFilt->num[0];
		den    = &pFilt->den[0];
		numLen = pFilt->order + 1;
		denLen = pFilt->order + 1;

		if (fAccel) {
			in  = &bfAccIn2[chip];
			out = &bfAccOut2[chip];
			bf_Put(&bfAccIn2[chip], dataIn);
		}
		else {
			in  = &bfRateIn2[chip];
			out = &bfRateOut2[chip];
			bf_Put(&bfRateIn2[chip], dataIn);
		}

		// denlen and numlen is filter order
		// assuming one filter is always 0
		int m, nu, ny;
		int i, j;

		m = in->m;   // number of rows
		nu = in->n;   // number of columns in buff u   
		ny = out->n;  // number of columns in buff y

		//-------------buffer depth is less than filter depth, error-----
		if (nu < numLen || ny < denLen - 1) {
			return 0;
		}

		//-------------------------begin filter--------------------------
		// set tmpU and tmpY to 0
		for (i = 0; i < m; i++)
		{
			tmpU[i] = 0.0;
			tmpY[i] = 0.0;
		}
		// calcualte tmpU = a0*u[k] + ... + an*u[k-n], u[k] is the latest (index is 0) data in u
		for (j = 0; j < numLen; j++)
		{
			if (!bf_Get(in, tmp, j)) // unavailable data are replaced by 0
			{
				for (i = 0; i < m; i++)
					tmp[i] = 0;
			}

			for (i = 0; i < m; i++)
			{
				float t = num[j];
				tmpU[i] +=  t * tmp[i];
			}
		}

		// calcualte b1*y[k-1] + ... + bn*y[k-n], y[k-1] is the latest data in y
		for (j = 0; j < denLen - 1; j++)
		{
			if (!bf_Get(out, tmp, j)) // unavailable data are replaced by 0
			{
				for (i = 0; i < m; i++)
					tmp[i] = 0;
			}
			for (i = 0; i < m; i++)
			{
				tmpY[i] += den[j + 1] * tmp[i]; // den[j+1], den[j]=1.0
			}

		}
		for (i = 0; i < m; i++)
		{
			tmp[i] = (tmpU[i] - tmpY[i]) / den[0];
			dataOut[i] = tmp[i];
		}
		// filtered data are put into out
		bf_Put(out, tmp);

		return 1;
	}

	
