// NEW 2022_10_15
//

#include <esp_log.h>
#include <driver/i2s.h>
#include <driver/i2c.h>
#include <math.h>
#include <esp_dsp.h>

#include "es8388_registers.h"

#define PIH 1.5707963267948966192313216916398f
#define PI 3.14159265359
#define TWO_PI 2*PI
#define FOURPI  (2.0 * TWO_PI)
#define SIXPI   (3.0 * TWO_PI)


/*
 * Basic I2S and I2C Configuration
 */
#define I2S_NUM I2S_NUM_0
//#define I2S_READLEN 50 * 4
//#define I2S_READLEN 192 * 4
#define I2S_READLEN 128 * 4  // Maximum is 1024 == 256 * 4

#define I2C_NUM I2C_NUM_0
#define ES8388_ADDR 0x20



/*
 * ES8388 Configuration Code
 * Configure ES8388 audio codec over I2C for AUX IN input and headphone jack output
 */
static esp_err_t es_write_reg(uint8_t slave_add, uint8_t reg_add, uint8_t data)
{
	esp_err_t res = ESP_OK;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	res |= i2c_master_start(cmd);
	res |= i2c_master_write_byte(cmd, slave_add, 1 /*ACK_CHECK_EN*/);
	res |= i2c_master_write_byte(cmd, reg_add, 1 /*ACK_CHECK_EN*/);
	res |= i2c_master_write_byte(cmd, data, 1 /*ACK_CHECK_EN*/);
	res |= i2c_master_stop(cmd);
	res |= i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
	i2c_cmd_link_delete(cmd);

	return res;
}

static esp_err_t es8388_init()
{
	esp_err_t res = ESP_OK;

	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = GPIO_NUM_18,
		.sda_pullup_en = true,
		.scl_io_num = GPIO_NUM_23,
		.scl_pullup_en = true,
		.master.clk_speed = 100000
	};

	res |= i2c_param_config(I2C_NUM, &i2c_config);
	res |= i2c_driver_install(I2C_NUM, i2c_config.mode, 0, 0, 0);

	/* mute DAC during setup, power up all systems, slave mode */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x04);
	res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL2, 0x50);
	res |= es_write_reg(ES8388_ADDR, ES8388_CHIPPOWER, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_MASTERMODE, 0x00);

	/* power up DAC and enable only LOUT1 / ROUT1, ADC sample rate = DAC sample rate */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x30);
	res |= es_write_reg(ES8388_ADDR, ES8388_CONTROL1, 0x12);

	/* DAC I2S setup: 16 bit word length, I2S format; MCLK / Fs = 256*/
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL1, 0x18);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL2, 0x02);

	/* DAC to output route mixer configuration */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL16, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL17, 0x90);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL20, 0x90);

	/* DAC and ADC use same LRCK, enable MCLK input; output resistance setup */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL21, 0x80);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL23, 0x00);

	/* DAC volume control: 0dB (maximum, unattenuated)  */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL5, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL4, 0x00);

	/* power down ADC while configuring; volume: +9dB for both channels */
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0xff);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL1, 0x88);

	/* select LINPUT2 / RINPUT2 as ADC input; stereo; 16 bit word length, format right-justified, MCLK / Fs = 256 */
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL2, 0x50); // 0x00 for built-in mics, 0x50 for AUX in
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL3, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL4, 0x0e);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL5, 0x02);

	/* set ADC volume */
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL8, 0x20);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCCONTROL9, 0x20);

	/* set LOUT1 / ROUT1 volume: 0dB (unattenuated) */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL24, 0x1e);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL25, 0x1e);

	/* power up and enable DAC; power up ADC (no MIC bias) */
	res |= es_write_reg(ES8388_ADDR, ES8388_DACPOWER, 0x3c);
	res |= es_write_reg(ES8388_ADDR, ES8388_DACCONTROL3, 0x00);
	res |= es_write_reg(ES8388_ADDR, ES8388_ADCPOWER, 0x09);

	return res;
}

/*
 * Digital Filtering Code
 * You can implement your own digital filters (e.g. FIR / IIR / biquad / ...) here.
 * Please mind that you need to use seperate storage for left / right channels for stereo filtering.
 */
static inline int16_t dummyfilter(int16_t x)
{
	return x;
}


float Izero (float x)
{
  float x2 = x / 2.0;
  float summe = 1.0;
  float ds = 1.0;
  float di = 1.0;
  float errorlimit = 1e-9;
  float tmp;
  do
  {
    tmp = x2 / di;
    tmp *= tmp;
    ds *= tmp;
    summe += ds;
    di += 1.0;
  }   while (ds >= errorlimit * summe);
  return (summe);
}  // END Izero

float m_sinc(int m, float fc)
{ // fc is f_cut/(Fsamp/2)
  // m is between -M and M step 2
  //
  float x = m * PIH;
  if (m == 0)
    return 1.0f;
  else
    return sinf(x * fc) / (fc * x);
}

void calc_FIR_coeffs (float * coeffs_I, int numCoeffs, float fc, float Astop, int type, float dfc, float Fsamprate)
// pointer to coefficients variable, no. of coefficients to calculate, frequency where it happens, stopband attenuation in dB,
// filter type, half-filter bandwidth (only for bandpass and notch)
{ // modified by WMXZ and DD4WH after
  // Wheatley, M. (2011): CuteSDR Technical Manual. www.metronix.com, pages 118 - 120, FIR with Kaiser-Bessel Window
  // assess required number of coefficients by
  //     numCoeffs = (Astop - 8.0) / (2.285 * TPI * normFtrans);
  // selecting high-pass, numCoeffs is forced to an even number for better frequency response

  float Beta;
  float izb;
  float fcf = fc;
  int nc = numCoeffs;
  fc = fc / Fsamprate;
  dfc = dfc / Fsamprate;
  // calculate Kaiser-Bessel window shape factor beta from stop-band attenuation
  if (Astop < 20.96)
    Beta = 0.0;
  else if (Astop >= 50.0)
    Beta = 0.1102 * (Astop - 8.71);
  else
    Beta = 0.5842 * powf((Astop - 20.96), 0.4) + 0.07886 * (Astop - 20.96);

  for (int i = 0; i < numCoeffs; i++) //zero pad entire coefficient buffer, important for variables from DMAMEM
  {
    coeffs_I[i] = 0.0;
  }

  izb = Izero (Beta);
  if (type == 0) // low pass filter
    //     {  fcf = fc;
  { fcf = fc * 2.0;
    nc =  numCoeffs;
  }
  else if (type == 1) // high-pass filter
  { fcf = -fc;
    nc =  2 * (numCoeffs / 2);
  }
  else if ((type == 2) || (type == 3)) // band-pass filter
  {
    fcf = dfc;
    nc =  2 * (numCoeffs / 2); // maybe not needed
  }
  else if (type == 4) // Hilbert transform
  {
    nc =  2 * (numCoeffs / 2);
    // clear coefficients
    for (int ii = 0; ii < 2 * (nc - 1); ii++) coeffs_I[ii] = 0;
    // set real delay
    coeffs_I[nc] = 1;

    // set imaginary Hilbert coefficients
    for (int ii = 1; ii < (nc + 1); ii += 2)
    {
      if (2 * ii == nc) continue;
      float x = (float)(2 * ii - nc) / (float)nc;
      float w = Izero(Beta * sqrtf(1.0f - x * x)) / izb; // Kaiser window
      coeffs_I[2 * ii + 1] = 1.0f / (PIH * (float)(ii - nc / 2)) * w ;
    }
    return;
  }

  for (int ii = - nc, jj = 0; ii < nc; ii += 2, jj++)
  {
    float x = (float)ii / (float)nc;
    float w = Izero(Beta * sqrtf(1.0f - x * x)) / izb; // Kaiser window
    coeffs_I[jj] = fcf * m_sinc(ii, fcf) * w;

  }

  if (type == 1)
  {
    coeffs_I[nc / 2] += 1;
  }
  else if (type == 2)
  {
    for (int jj = 0; jj < nc + 1; jj++) coeffs_I[jj] *= 2.0f * cosf(PIH * (2 * jj - nc) * fc);
  }
  else if (type == 3)
  {
    for (int jj = 0; jj < nc + 1; jj++) coeffs_I[jj] *= -2.0f * cosf(PIH * (2 * jj - nc) * fc);
    coeffs_I[nc / 2] += 1;
  }

} // END calc_FIR_coeffs



/*
 * Main
 */
void app_main(void)
{
	printf("[filter-dsp] Initializing audio codec via I2C...\r\n");

	if (es8388_init() != ESP_OK) {
		printf("[filter-dsp] Audio codec initialization failed!\r\n");
	} else {
		printf("[filter-dsp] Audio codec initialization OK\r\n");
	}

	/*******************/

	printf("[filter-dsp] Initializing input I2S...\r\n");

	i2s_config_t i2s_read_config = {
		.mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX,
		.sample_rate = 48000,
		.bits_per_sample = 16,
		.communication_format = I2S_COMM_FORMAT_I2S,
		.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
		.intr_alloc_flags = ESP_INTR_FLAG_LEVEL2,
		.dma_buf_count = 4,
		.dma_buf_len = I2S_READLEN,
		.use_apll = 1,
		.tx_desc_auto_clear = 1,
		.fixed_mclk = 0
	};

	i2s_pin_config_t i2s_read_pin_config = {
		.bck_io_num = GPIO_NUM_5,
		.ws_io_num = GPIO_NUM_25,
		.data_out_num = GPIO_NUM_26,
		.data_in_num = GPIO_NUM_35
	};

	i2s_driver_install(I2S_NUM, &i2s_read_config, 0, NULL);
	i2s_set_pin(I2S_NUM, &i2s_read_pin_config);

	/*******************/

	printf("[filter-dsp] Initializing MCLK output...\r\n");

	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
	WRITE_PERI_REG(PIN_CTRL, 0xFFF0);

	/*******************/

	printf("[filter-dsp] Enabling Passthrough mode...\r\n");

	size_t i2s_bytes_read = 0;
	size_t i2s_bytes_written = 0;

DMA_ATTR static	int16_t i2s_buffer_read[I2S_READLEN / sizeof(int16_t)];
DMA_ATTR static	int16_t i2s_buffer_write[I2S_READLEN / sizeof(int16_t)];
int shift = 4; 
#define BLOCK_SIZE 128
const int N_BLOCKS = 6; //9; // 6 blocks á 128 samples  = 768 samples. No. of samples has to be dividable by 2 AND by 3 AND by 4 // 6 blocks of 128 samples == 768 samples = 17.4ms
const int WINDOW_LENGTH = N_BLOCKS * BLOCK_SIZE;
const int WINDOW_LENGTH_D_2 = WINDOW_LENGTH / 2;
#define IN_BUFFER_SIZE WINDOW_LENGTH
#define HOPSIZE_4 WINDOW_LENGTH/4
#define HOPSIZE_3 WINDOW_LENGTH/3
DMA_ATTR static float in_buffer_L[4][768];
DMA_ATTR static float in_buffer_R[4][768];
//float in_buffer_L[IN_BUFFER_SIZE];
//float in_buffer_R[IN_BUFFER_SIZE];
DMA_ATTR static float out_buffer_L[768];
DMA_ATTR static float out_buffer_R[768];
DMA_ATTR static float add_buffer_L[384];
DMA_ATTR static float add_buffer_R[384];
DMA_ATTR static float window[768];
int buffer_idx = 0;
int hop0 = 0;
int hop1 = 1;
int hop2 = 2;
int hop3 = 3;

// apply windowing to array window[] by using function from the esp_dsp lib
dsps_wind_hann_f32(window, WINDOW_LENGTH);
//dsps_wind_blackman_harris_f32(window, WINDOW_LENGTH);
//dsps_wind_blackman_nuttall_f32(window, WINDOW_LENGTH);

DMA_ATTR static fir_f32_t FIR_L;
DMA_ATTR static fir_f32_t FIR_R;
DMA_ATTR static float FIR_coeffs[36];
DMA_ATTR static float FIR_L_delay[36];
DMA_ATTR static float FIR_R_delay[36];
DMA_ATTR static int FIR_N_coeffs=36;

// calculate coeffs for interpolation lowpass filter
calc_FIR_coeffs (FIR_coeffs, FIR_N_coeffs, (float)3500, 80, 0, 0.0, 48000);

// initialize FIR filter
dsps_fir_init_f32(&FIR_L, FIR_coeffs, FIR_L_delay, FIR_N_coeffs);
dsps_fir_init_f32(&FIR_R, FIR_coeffs, FIR_R_delay, FIR_N_coeffs);



	/* continuously read data over I2S, pass it through the filtering function and write it back */
	while (true) {
		i2s_bytes_read = I2S_READLEN;
		// this is a first trial to read six blocks of audio, process them and then write six blocks back

		for(uint32_t j = 0; j < N_BLOCKS; j++)
		{
		i2s_read(I2S_NUM, &i2s_buffer_read, I2S_READLEN, &i2s_bytes_read, 100);

		for (uint32_t i = 0; i < i2s_bytes_read / 4; i++)
		{
			in_buffer_L[buffer_idx][i + j * BLOCK_SIZE] = (float)i2s_buffer_read[i * 2 + 0] / 32768.0f;
			in_buffer_R[buffer_idx][i + j * BLOCK_SIZE] = (float)i2s_buffer_read[i * 2 + 1] / 32768.0f;
		}
		}

      /**********************************************************************************
          Time Domain pitch shift algorithm "OLA" by Harald Mills "Hear birds again" 
       **********************************************************************************/

      /**********************************************************************************
          1 Windowing 
       **********************************************************************************/

//             we apply the second half of the window to the first half of the input buffer
//             works for N==2 
        if(shift == 2)
        {
            for (unsigned i = 0; i < WINDOW_LENGTH_D_2; i++)
            {
              in_buffer_L[0][i] = in_buffer_L[0][i] * window[i + WINDOW_LENGTH_D_2];
              in_buffer_R[0][i] = in_buffer_R[0][i] * window[i + WINDOW_LENGTH_D_2];
            }
//             we apply the first half of the window to the second half of the input buffer
    
            for (unsigned i = 0; i < WINDOW_LENGTH_D_2; i++)
            {
              in_buffer_L[0][i + WINDOW_LENGTH_D_2] = in_buffer_L[0][i + WINDOW_LENGTH_D_2] * window[i];
              in_buffer_R[0][i + WINDOW_LENGTH_D_2] = in_buffer_R[0][i + WINDOW_LENGTH_D_2] * window[i];
            }
        }
        else if(shift == 3)
        {
           for (unsigned i = 0; i < WINDOW_LENGTH; i++)
           {
              in_buffer_L[buffer_idx][i] = in_buffer_L[buffer_idx][i] * window[i];
              in_buffer_R[buffer_idx][i] = in_buffer_R[buffer_idx][i] * window[i];
           }          
        }
        else if(shift == 4)
        {
           for (unsigned i = 0; i < WINDOW_LENGTH; i++)
           {
              in_buffer_L[buffer_idx][i] = in_buffer_L[buffer_idx][i] * window[i];
              in_buffer_R[buffer_idx][i] = in_buffer_R[buffer_idx][i] * window[i];
           }
        } // end shift == 4


    
      /**********************************************************************************
          2 Overlap & Add 
       **********************************************************************************/
        if(shift == 2)
        {
            for (unsigned i = 0; i < WINDOW_LENGTH_D_2; i++)
            {
              add_buffer_L[i] = in_buffer_L[0][i] + in_buffer_L[0][i + WINDOW_LENGTH_D_2];
              add_buffer_R[i] = in_buffer_R[0][i] + in_buffer_R[0][i + WINDOW_LENGTH_D_2];
            }
        }
        else if(shift == 3)
        {   // index of in_buffer    [0][x]  [1][x]  [2][x]  
            if(buffer_idx==2)       {hop0=2, hop1=1, hop2=0;}
            else if(buffer_idx==1)  {hop0=1, hop1=0, hop2=2;}
            else if(buffer_idx==0)  {hop0=0, hop1=2, hop2=1;}
            hop0=hop0*HOPSIZE_3;
            hop1=hop1*HOPSIZE_3;
            hop2=hop2*HOPSIZE_3;
            for (unsigned i = 0; i < HOPSIZE_3; i++)
            {
                add_buffer_L[i] = in_buffer_L[0][i + hop0] + in_buffer_L[1][i + hop1] + in_buffer_L[2][i + hop2]; 
                add_buffer_R[i] = in_buffer_R[0][i + hop0] + in_buffer_R[1][i + hop1] + in_buffer_R[2][i + hop2]; 
            }

            buffer_idx = buffer_idx + 1;  // increment
            if (buffer_idx >=3) {buffer_idx = 0;} // flip-over           
        }
        else if(shift == 4)
        {   // index of in_buffer    [0][x]  [1][x]  [2][x]  [3][x]
            if(buffer_idx == 3)     {hop0=3, hop1=2, hop2=1, hop3=0;}
            else if(buffer_idx==2)  {hop0=2, hop1=1, hop2=0, hop3=3;}
            else if(buffer_idx==1)  {hop0=1, hop1=0, hop2=3, hop3=2;}
            else if(buffer_idx==0)  {hop0=0, hop1=3, hop2=2, hop3=1;}
            hop0=hop0*HOPSIZE_4;
            hop1=hop1*HOPSIZE_4;
            hop2=hop2*HOPSIZE_4;
            hop3=hop3*HOPSIZE_4;
            for (unsigned i = 0; i < HOPSIZE_4; i++)
            {
                add_buffer_L[i] = in_buffer_L[0][i + hop0] + in_buffer_L[1][i + hop1] + in_buffer_L[2][i + hop2] + in_buffer_L[3][i + hop3]; 
                add_buffer_R[i] = in_buffer_R[0][i + hop0] + in_buffer_R[1][i + hop1] + in_buffer_R[2][i + hop2] + in_buffer_R[3][i + hop3]; 
            }

            buffer_idx = buffer_idx + 1;  // increment
            if (buffer_idx >=4) {buffer_idx = 0;} // flip-over  
        }

      /**********************************************************************************
          3 Interpolate 
       **********************************************************************************/

      // receives 512 samples and makes 1024 samples out of it
      // interpolation-by-2
      // interpolation-in-place does not work
      // blocksize is BEFORE zero stuffing
      // recycle in_buffer as temporary buffer
      if(shift == 2)
      { // zero stuffing		
            for (unsigned i = 0; i < WINDOW_LENGTH_D_2; i++)
            {
                in_buffer_L[buffer_idx][i * 2 + 0] = add_buffer_L[i];
                in_buffer_L[buffer_idx][i * 2 + 1] = 0;
                in_buffer_R[buffer_idx][i * 2 + 0] = add_buffer_R[i];
                in_buffer_R[buffer_idx][i * 2 + 1] = 0;
	    }
	// FIR lowpass filter to eliminate aliases caused by zero-stuffing
	// zero-stuffing & lowpass filtering == interpolation 
	dsps_fir_f32_ae32(&FIR_L, in_buffer_L[buffer_idx], out_buffer_L, BLOCK_SIZE * N_BLOCKS);
	dsps_fir_f32_ae32(&FIR_R, in_buffer_R[buffer_idx], out_buffer_R, BLOCK_SIZE * N_BLOCKS);

      }
      else if(shift == 3)
      {
          // simple zero-stuffing
            for (unsigned i = 0; i < HOPSIZE_3; i++)
            {
                in_buffer_L[buffer_idx][i * 3 + 0] = add_buffer_L[i];
                in_buffer_L[buffer_idx][i * 3 + 1] = 0;
                in_buffer_L[buffer_idx][i * 3 + 2] = 0;
                in_buffer_R[buffer_idx][i * 3 + 0] = add_buffer_R[i];
                in_buffer_R[buffer_idx][i * 3 + 1] = 0;
                in_buffer_R[buffer_idx][i * 3 + 2] = 0;
	    }
	// FIR lowpass filter to eliminate aliases caused by zero-stuffing
	// zero-stuffing & lowpass filtering == interpolation 
	dsps_fir_f32_ae32(&FIR_L, in_buffer_L[buffer_idx], out_buffer_L, BLOCK_SIZE * N_BLOCKS);
	dsps_fir_f32_ae32(&FIR_R, in_buffer_R[buffer_idx], out_buffer_R, BLOCK_SIZE * N_BLOCKS);
      }
      else if(shift == 4)
      {
          // simple zero-stuffing
            for (unsigned i = 0; i < HOPSIZE_4; i++)
            {
                in_buffer_L[buffer_idx][i * 4 + 0] = add_buffer_L[i];
                in_buffer_L[buffer_idx][i * 4 + 1] = 0;
                in_buffer_L[buffer_idx][i * 4 + 2] = 0;
                in_buffer_L[buffer_idx][i * 4 + 3] = 0;
                in_buffer_R[buffer_idx][i * 4 + 0] = add_buffer_R[i];
                in_buffer_R[buffer_idx][i * 4 + 1] = 0;
                in_buffer_R[buffer_idx][i * 4 + 2] = 0;
                in_buffer_R[buffer_idx][i * 4 + 3] = 0;            
	    }
	// FIR lowpass filter to eliminate aliases caused by zero-stuffing
	// zero-stuffing & lowpass filtering == interpolation 
	dsps_fir_f32_ae32(&FIR_L, in_buffer_L[buffer_idx], out_buffer_L, BLOCK_SIZE * N_BLOCKS);
	dsps_fir_f32_ae32(&FIR_R, in_buffer_R[buffer_idx], out_buffer_R, BLOCK_SIZE * N_BLOCKS);
      }

      /**********************************************************************************
           
       **********************************************************************************/


		/* now write the six blocks back to the I2S output stream */
		for(uint32_t j = 0; j < N_BLOCKS; j++)
		{
		for (uint32_t i = 0; i < i2s_bytes_read / 4; i++)
		{
			i2s_buffer_write[i * 2 + 0] = (int16_t)(out_buffer_L[i + j * BLOCK_SIZE] * 32768.0f * 5.0f); // apply some audio gain
			i2s_buffer_write[i * 2 + 1] = (int16_t)(out_buffer_R[i + j * BLOCK_SIZE] * 32768.0f * 5.0f);
		}
		i2s_write(I2S_NUM, &i2s_buffer_write, i2s_bytes_read, &i2s_bytes_written, I2S_READLEN / 2);
		}

	} // end while ("loop")
} // end main


