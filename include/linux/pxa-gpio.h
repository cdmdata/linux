#ifndef _LINUX_PXA_GPIO_H_
#define _LINUX_PXA_GPIO_H_

#define BASE_MAGIC 250
#define GPIO_SET_DEBOUNCE_DELAY_LOW   _IOW(BASE_MAGIC, 0x01, unsigned long)
#define GPIO_SET_DEBOUNCE_DELAY_HIGH  _IOW(BASE_MAGIC, 0x02, unsigned long)
#define GPIO_SET_CONFIG		      _IOW(BASE_MAGIC, 0x03, unsigned long)	//only for minor 255
#define GPIO_GET_DEBOUNCE_DELAY_LOW   _IOR(BASE_MAGIC, 0x01, unsigned long)
#define GPIO_GET_DEBOUNCE_DELAY_HIGH  _IOR(BASE_MAGIC, 0x02, unsigned long)

//sampleFlags bit values
#define SAMPLE_INPUT_FALLING 1
#define SAMPLE_INPUT_RISING 2
#define SAMPLE_OUTPUT_FALLING 4
#define SAMPLE_OUTPUT_RISING 8
#define SAMPLE_OUTPUT_INVERT 0x10
#define SAMPLE_FIQ 0x20			//if 1 entry has this set, then FIQ is active
					//for all entries with the same inputGp
#define SAMPLE_TIMESTAMP 0x40		//getting a time maybe slow, by not setting this flag, performance maybe improved
					//then the timestamp field of gpEvent should not be relied upon.
#define SAMPLE_OVERRUN 0x80
#define SAMPLE_INPUT_BOTH (SAMPLE_INPUT_RISING|SAMPLE_INPUT_FALLING)
#define SAMPLE_OUTPUT_BOTH (SAMPLE_OUTPUT_RISING|SAMPLE_OUTPUT_FALLING)
struct gpConfigSample {
	unsigned char inputGp;
	unsigned char sampleGp;
	unsigned char sampleFlags;
	unsigned char spare;
};
struct gpConfig {
	unsigned changeGpEventCnt;		//0 - means don't change, otherwise change number of events that can be queued
	unsigned sampleCnt;
	struct gpConfigSample gpSamples[0];
};

//to use queued input, open device major 254, minor 255
//gpEvent is the structure returned on reads
//on writes bits 0-6 is gp #, bit 7 is value
struct gpEvent {
	struct timeval timeStamp;
	unsigned char inputGp;
	unsigned char sampleGp;
	unsigned char sampleFlags;
	unsigned char level;
};

#define MAX_INVERT_BITS    128
#define INVERT_BYTES       (MAX_INVERT_BITS/8)

#define GPIO_GET_INVERT _IOR(BASE_MAGIC, 0x04, __u8[INVERT_BYTES]) // returns bit mask of pins to invert
#define GPIO_SET_INVERT _IOW(BASE_MAGIC, 0x05, __u8[INVERT_BYTES]) // accepts bit mask of pins to invert


/*
 * Pulse parameter high bit is direction, low 31 bits are duration in jiffies
 *
 */
#define PULSELOW(duration)	((duration)&0x7fffffff)
#define PULSEHIGH(duration)	(0x80000000|((duration)&0x7fffffff))
#define GPIO_PULSE _IOW(BASE_MAGIC, 0x06, __u32)

#endif
