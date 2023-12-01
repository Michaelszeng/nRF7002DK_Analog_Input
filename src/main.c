#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/adc.h>

#define SLEEP_TIME_MS   250

#define ADC_NODE DT_NODELABEL(adc)
static const struct device* adc_dev = DEVICE_DT_GET(ADC_NODE);

#define ADC_RESOLUTION 10
#define ADC_CHANNEL    0
#define ADC_PORT       SAADC_CH_PSELP_PSELP_AnalogInput7  // number at very end is which AIN pin to use
#define ADC_REFERENCE  ADC_REF_INTERNAL
#define ADC_GAIN       ADC_GAIN_1_5



static const struct adc_channel_cfg ch0_cfg = {
	.gain = ADC_GAIN,
	.reference = ADC_REFERENCE,
	.acquisition_time = ADC_ACQ_TIME_DEFAULT,
	.channel_id = ADC_CHANNEL,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
	.input_positive = ADC_PORT,
#endif
};


int16_t sample_buffer[1];


const struct adc_sequence sequence = {
	.channels = BIT(ADC_CHANNEL),  // Can add multiple channels using bitwise OR
	.buffer = sample_buffer,
	.buffer_size = sizeof(sample_buffer),  // num_bytes
	.resolution = ADC_RESOLUTION,
};



int main(void)
{
	int err;

	k_msleep(1000);
	printk("Starting Program..\n");

	if (!device_is_ready(adc_dev)) {
		printk("adc_dev not ready. returning.\n");
		return -1;
	}

	err = adc_channel_setup(adc_dev, &ch0_cfg);
    if (err) {
	    printk("Error in adc setup: %d\n", err);
		return -1;
	}

	while (1) {

		// Perform ADC Read and populate sample_buffer
		err = adc_read(adc_dev, &sequence);
		if (err) {
  	    	printk("adc_read() failed with code %d\n", err);
		}

		int32_t adc_val = sample_buffer[0];
		int32_t adc_vref = adc_ref_internal(adc_dev);
		adc_raw_to_millivolts(adc_vref, ADC_GAIN, ADC_RESOLUTION, &adc_val);
		printk("ADC Read: %d mV\n", adc_val);

		k_msleep(SLEEP_TIME_MS);

	}
	return 0;
}














// #include <zephyr.h>
// #include <sys/printk.h>
// #include <drivers/pwm.h>

// #if defined(CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPP) ||  defined(CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPPNS) || defined(CONFIG_BOARD_NRF9160DK_NRF9160NS) || defined(CONFIG_BOARD_NRF9160DK_NRF9160)

// /*ADC definitions and includes*/
// #include <hal/nrf_saadc.h>
// #define ADC_DEVICE_NAME DT_LABEL(DT_INST(0, nordic_nrf_saadc))
// #define ADC_RESOLUTION 10
// #define ADC_GAIN ADC_GAIN_1_6
// #define ADC_REFERENCE ADC_REF_INTERNAL
// #define ADC_ACQUISITION_TIME ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 10)
// #define ADC_1ST_CHANNEL_ID 0  
// #define ADC_1ST_CHANNEL_INPUT NRF_SAADC_INPUT_AIN0

// #define PWM_DEVICE_NAME DT_PROP(DT_NODELABEL(pwm0), label)
// #define PWM_CH0_PIN DT_PROP(DT_NODELABEL(pwm0), ch0_pin)

// #else
// #error "Choose supported board or add new board for the application"
// #endif

// #include <zephyr.h>
// #include <device.h>
// #include <drivers/gpio.h>
// #include <drivers/adc.h>
// #include <string.h>
// #include <drivers/pwm.h>

// #define PWM_MAX 253
// #define TIMER_INTERVAL_MSEC 200
// #define BUFFER_SIZE 1

// struct k_timer my_timer;
// const struct device *adc_dev;
// const struct device *pwm_dev;

// K_SEM_DEFINE(adc_sem, 0, 1);

// static const struct adc_channel_cfg m_1st_channel_cfg = {
// 	.gain = ADC_GAIN,
// 	.reference = ADC_REFERENCE,
// 	.acquisition_time = ADC_ACQUISITION_TIME,
// 	.channel_id = ADC_1ST_CHANNEL_ID,
// #if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
// 	.input_positive = ADC_1ST_CHANNEL_INPUT,
// #endif
// };

// static s16_t m_sample_buffer[BUFFER_SIZE];

// static int adc_sample(void)
// {
// 	int ret;
// 	const struct adc_sequence sequence = {
// 		.channels = BIT(ADC_1ST_CHANNEL_ID),
// 		.buffer = m_sample_buffer,
// 		.buffer_size = sizeof(m_sample_buffer),
// 		.resolution = ADC_RESOLUTION,
// 	};

// 	if (!adc_dev) {
// 		return -1;
// 	}

// 	ret = adc_read(adc_dev, &sequence);
// 	if (ret) {
//         printk("adc_read() failed with code %d\n", ret);
// 	}
// 	for (int i = 0; i < BUFFER_SIZE; i++) {
//                 printk("ADC raw value: %d\n", m_sample_buffer[i]);
//                 float val = ((float)PWM_MAX/(float)568)*(float)m_sample_buffer[i];
//                 printk("Setting pulse to: %f\n", val);
//                 pwm_pin_set_usec(pwm_dev, PWM_CH0_PIN , PWM_MAX, val, 0);
// 	}

// 	return ret;
// }

// void adc_sample_event(struct k_timer *timer_id){
//     k_sem_give(&adc_sem);
//     /*int err = adc_sample();
//     if (err) {
//         printk("Error in adc sampling: %d\n", err);
//     }*/
// }

// void main(void)
// {
//     int err;
//     //PWM0 setup
//     pwm_dev = device_get_binding(PWM_DEVICE_NAME);
//     if (!pwm_dev) {
// 	    printk("device_get_binding() PWM0 failed\n");
// 	}
//     //Timer setup
//     k_timer_init(&my_timer, adc_sample_event, NULL);
//     k_timer_start(&my_timer, K_MSEC(TIMER_INTERVAL_MSEC), K_MSEC(TIMER_INTERVAL_MSEC));

     
//     //ADC0 setup
//     adc_dev = device_get_binding(ADC_DEVICE_NAME);
// 	if (!adc_dev) {
//         printk("device_get_binding ADC_0 (=%s) failed\n", ADC_DEVICE_NAME);
//     } 
//     err = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
//     if (err) {
// 	    printk("Error in adc setup: %d\n", err);
// 	}

//     #if defined(CONFIG_BOARD_NRF9160DK_NRF9160NS) ||  defined(CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPPNS)
//     NRF_SAADC_NS->TASKS_CALIBRATEOFFSET = 1;
//     #elif defined(CONFIG_BOARD_NRF9160DK_NRF9160) || defined(CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPP)
//     NRF_SAADC->TASKS_CALIBRATEOFFSET = 1;
//     #else
//     #error "Choose supported board or add new board for the application"
//     #endif
//     while(true){
//         k_sem_take(&adc_sem, K_FOREVER);
//         adc_sample();
//     }
// }