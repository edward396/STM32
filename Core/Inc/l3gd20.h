/*
 * l3gd20.h
 *
 *  Created on: Apr 13, 2025
 *      Author: truon
 */

#ifndef INC_L3GD20_H_
#define INC_L3GD20_H_
#include <stdint.h>

#define WHO_AM_I		0x0F //Device ID register
#define CTRL_REG1		0x20 //Power mode, data rate, bandwidth, axis enable
#define CTRL_REG2		0x21 //High-pass filter config
#define CTRL_REG3		0x22 //Interrupt and output signal config
#define CTRL_REG4		0x23 //Data format and full-scale sel
#define CTRL_REG5		0x24 //FIFO, high-pass filter, output sel

#define REFERENCE		0x25 //Ref val for interrupt generation
#define OUT_TEMP		0x26 //Internal temperature output (raw, 8-bit)
#define STATUS_REG		0x27 //Data ready and overrun status

#define OUT_X_L			0x28 //X-axis angular rate (LSB)
#define OUT_X_H			0x29 //X-axis angular rate (MSB)
#define OUT_Y_L			0x2A //Y-axis angular rate (LSB)
#define OUT_Y_H			0x2B //Y-axis angular rate (MSB)
#define OUT_Z_L			0x2C //Z-axis angular rate (LSB)
#define OUT_Z_H			0x2D //Z-axis angular rate (MSB)

#define FIFO_CTRL_REG	0x2E //FIFO mode, watermark threshold
#define FIFO_SRC_REG	0x2F //FIFO status (level, full, empty, watermark)

#define INT1_CFG		0x30 //Axis, direction config for INT1 trigger
#define INT1_SRC		0x31 //INT1 status: which axis/direction triggered

#define INT1_TSH_XH		0x32 //X-axis interrupt threshold (MSB)
#define INT1_TSH_XL		0x33 //X-axis interrupt threshold (LSB)

#define INT1_TSH_YH		0x34 //Y-axis interrupt threshold (MSB)
#define INT1_TSH_YL		0x35 //Y-axis interrupt threshold (LSB)

#define INT1_TSH_ZH		0x36 //Z-axis interrupt threshold (MSB)
#define INT1_TSH_ZL		0x37 //Z-axis interrupt threshold (LSB)

#define INT1_DURATION	0x38 //Min duration condition must hold to trigger INT1

////////////////////////////END OF REGISTER MAPPING////////////////////////////



/*
 * WHO_AM_I (default: 1101 0100) (read only)
 */
#define I_AM_L3GD20		((uint8_t)0xD4) //Default value



/*
 * CTRL_REG1 (default: 0000 0111)
 * Axes enable and disable
 * Power modes
 * Output Data Rate and Bandwidth
 */
#define L3GD20_X_ENABLE		((uint8_t)0x01)
#define L3GD20_Y_ENABLE 	((uint8_t)0x02)
#define L3GD20_Z_ENABLE 	((uint8_t)0x04)
#define L3GD20_AXES_ENABLE	((uint8_t)0x07)
#define L3GD20_AXES_DISABLE	((uint8_t)0x00)

#define L3GD20_MODE_POWERDOWN	((uint8_t)0x00)
#define L3GD20_ACTIVE			((uint8_t)0x08)

#define L3GD20_ODR95HZ_BW12HZ	((uint8_t)0x00) //ODR 95Hz, Cut-off 12.5Hz
#define L3GD20_ODR95HZ_BW25HZ	((uint8_t)0x30) //ODR 95Hz, Cut-off 25Hz

#define L3GD20_ODR380HZ_BW20Hz	((uint8_t)0x80) //ODR 380Hz, Cut-off 20Hz
#define L3GD20_ODR380HZ_BW100HZ	((uint8_t)0xB0) //ODR 380Hz, Cut-off 100Hz

#define L3GD20_ODR760HZ_BW30HZ	((uint8_t)0xC0) //ODR 760Hz, Cut-off 30Hz
#define L3GD20_ODR760HZ_BW100HZ	((uint8_t)0xF0) //ODR 760Hz, Cut-off 100Hz



/*
 * CTRL_REG2 (default: 0000 0000)
 * High-pass filter mode selection
 * High-pass filter cut-off freq config
 */
#define L3GD20_HPM_NORMALMODE_MANUALRES	((uint8_t)0x00) //Only reset when read the REFERENCE reg
#define L3GD20_HPM_REFSIGNAL			((uint8_t)0x10) //Use val of REFERENCE reg as the baseline
#define L3GD20_HPM_NORMALMODE			((uint8_t)0x20) //Internal reset, auto-adjusts its baseline internally
#define L3GD20_HPM_AUTORESET_INT		((uint8_t)0x30) //Reset automatically after an interrupt event

//Correspond to 95Hz, 380Hz, 760Hz ODR
#define L3GD20_HPFCF1	((uint8_t)0x00)	//7.2HZ, 27Hz, 51.4Hz
#define L3GD20_HPFCF2	((uint8_t)0x01) //3.5Hz, 13.5Hz, 27Hz
#define L3GD20_HPFCF3	((uint8_t)0x02) //1.8Hz, 7.2Hz, 13.5Hz
#define L3GD20_HPFCF4	((uint8_t)0x03) //0.9Hz, 3.5Hz, 7.2Hz
#define L3GD20_HPFCF5	((uint8_t)0x04) //0.45Hz, 1.8Hz, 3.5Hz
#define L3GD20_HPFCF6	((uint8_t)0x05) //0.18Hz, 0.9Hz, 1.8Hz
#define L3GD20_HPFCF7	((uint8_t)0x06) //0.09Hz, 0.45Hz, 0.9Hz
#define L3GD20_HPFCF8	((uint8_t)0x07) //0.045Hz, 0.18Hz, 0.45Hz
#define L3GD20_HPFCF9	((uint8_t)0x08) //0.018Hz, 0.09Hz, 0.18Hz
#define L3GD20_HPFCF10	((uint8_t)0x09) //0.009Hz, 0.045Hz, 0.09Hz



/*
 * CTRL_REG3 (default: 0000 0000)
 */
#define L3GD20_INT2_FIFO_EMPTY_ENABLE		((uint8_t)0x01)
#define L3GD20_INT2_FIFO_OVERRUN_ENABLE		((uint8_t)0x02)
#define L3GD20_INT2_WATERMARK_ENABLE		((uint8_t)0x04)
#define L3GD20_INT2_DATAREADY_ENABLE		((uint8_t)0x08)

#define L3GD20_PUSHPULL				((uint8_t)0x00)
#define L3GD20_OPENDRAIN			((uint8_t)0x10)

#define L3GD20_INT1_HIGHEDGE		((uint8_t)0x00)
#define L3GD20_INT1_LOWEDGE			((uint8_t)0x20)

#define L3GD20_INT1_BOOT_ENABLE		((uint8_t)0x40)
#define L3GD20_INT1_BOOT_DISABLE	((uint8_t)0x00)

#define L3GD20_INT1_ENABLE			((uint8_t)0x80)
#define L3GD20_INT1_DISABLE			((uint8_t)0x00)



/*
 * CTRL_REG4 (default: 0000 0000)
 */
#define L3GD20_SPI_4W			((uint8_t)0x00) //Standard SPI, 4-wire SPI
#define L3GD20_SPI_3W			((uint8_t)0x01) //3-wire SPI

#define L3GD20_FS_250DPS		((uint8_t)0x00) //Works for slow rotations (Slow, stable drone), high resolution
#define L3GD20_FS_500DPS		((uint8_t)0x10) //Works for medium rotations (Acro Flight)
#define L3GD20_FS_2000DPS		((uint8_t)0x20) //Works for fast spins (Racing, FPV drone), low resolution

#define L3GD20_LITTLE_ENDIAN	((uint8_t)0x00) //LSB is in lower register addr (Recommended sel)
#define L3GD20_BIG_ENDIAN	 	((uint8_t)0x40) //MSB is in lower register addr

#define L3GD20_BDU_CONTINUOUS	((uint8_t)0x00) //Data can change while reading
#define L3GD20_BDU_LOCK			((uint8_t)0x80) //Lock data update until finishing reading MSB and LSB (Recommended Sel)



/*
 * CTRL_REG5 (default: 0000 0000)
 * For OUTSEL and INT1SEL, see figure 18
 */
#define L3GD20_OUTSEL_LPF1			((uint8_t)0x00) 		 //Gyro signal passes through LPF1 only
#define L3GD20_OUTSEL_LPF1_WITH_HPF	((uint8_t)(0x10 | 0x01)) //...passes through LPF1 and HPF
#define L3GD20_OUTSEL_LPF2 			((uint8_t)0x03)			 //...passes through LPF1 and LPF2 only
#define L3GD20_OUTSEL_LPF2_WITH_HPF	((uint8_t)(0x10 | 0x03)) //...passes through LPF1, HPF, LPF2

//Signal selected for interrupt system
#define L3GD20_INT1SEL_LPF1				((uint8_t)0x00)			 //Raw output from LPF1
#define L3GD20_INT1SEL_LPF1_WITH_HPF	((uint8_t)(0x10 | 0x04)) //LPF1 + HPF output
#define L3GD20_INT1SEL_LPF2				((uint8_t)0x0C)			 //LPF1 + LPF2 output
#define L3GD20_INT1SEL_LPF2_WITH_HPF	((uint8_t)(0x10 | 0x0C)) //LPF1 + HPF + LPF2 output

#define L3GD20_HIGHPASS_ENABLE		((uint8_t)0x10) //Set HPen bit to 1
#define L3GD20_HIGHPASS_DISABLE		((uint8_t)0x00) //Set HPen bit to 0

#define L3GD20_FIFO_ENABLE			((uint8_t)0x40) //Set FIFO_EN to 1
#define L3GD20_FIFO_DISABLE			((uint8_t)0x00) //Set FIFO_EN to 0

#define L3GD20_BOOT_NORMALMODE		((uint8_t)0x80)
#define L3GD20_BOOT_REBOOTMEM		((uint8_t)0x00)



/*
 * STATUS_REG
 * Read only
 * AV = available
 */
//New data available check
#define L3GD20_XDATA_AV_STT		((uint8_t)0x01) //New X data is ready to read
#define L3GD20_YDATA_AV_STT		((uint8_t)0x02) //New Y...ready to read
#define L3GD20_ZDATA_AV_STT		((uint8_t)0x04) //New Z...ready to read
#define L3GD20_XYZDATA_AV_STT	((uint8_t)0x08) //New XYZ...ready to read

//Data overrun check
#define L3GD20_XDATA_OVRN_STT		((uint8_t)0x10) //New X data has overwritten the prev data
#define L3GD20_YDATA_OVRN_STT		((uint8_t)0x20) //New Y...overrwritten the prev data
#define L3GD20_ZDATA_OVRN_STT		((uint8_t)0x40) //New Z...overrwritten the prev data
#define L3GD20_XYZDATA_OVRN_STT		((uint8_t)0x80) //New XYZ data has overwritten the prev data



/*
 * FIFO_CTRL_REG (default: 0000 0000)
 * WTM threshold can stores 5 bits (0-31 samples)
 */
#define L3GD20_FIFO_WTM(x) ((uint8_t)((x) & 0x1F)) //WTM stores 5bits (0-31), x = 0 to 31, (& 0x1F) is to avoid wrong input > 5bits

#define L3GD20_FIFO_BYPASS		((uint8_t)0x00) //FIFO disabled. Data goes directly to out reg
#define L3GD20_FIFO_FIFO		((uint8_t)0x20) //FIFO stores data until full and stops when full (read it manually)
#define L3GD20_FIFO_STREAM		((uint8_t)0x40) //FIFO continously collects data, oldest data is overwritten
#define L3GD20_FIFO_STREAM2FIFO	((uint8_t)0x60) //Starts in stream and freezes FIFO when WTM is hit
#define L3GD20_BYPASS2STREAM	((uint8_t)0x80) //Starts in bypass, then switches to stream on interrupt



/*
 * FIFO_SRC_REG (read only)
 * Just to check the status of WTM
 * STT = status
 */
#define L3GD20_FIFO_WTM_STT		((uint8_t)0x80) //FIFO reached or > predefined WTM
#define L3GD20_FIFO_OVRN_STT	((uint8_t)0x40) //FIFO is completely filled
#define L3GD20_FIFO_EMPTY_STT	((uint8_t)0x20) //FIFO is empty
#define L3GD20_FIFO_FSS_MASK	((uint8_t)0x1F) //FIFO sample count (0-31), check the sample counts form bit 0 to 4



/*
 * INT1_CFG (default: 0000 0000)
 */
#define L3GD20_INT1_XLOW_ENABLE		((uint8_t)0x01) //Enable interrupt request on measured X value lower than preset threshold
#define L3GD20_INT1_XLOW_DISABLE	((uint8_t)0x00)	//Disable...
#define L3GD20_INT1_XHIGH_ENABLE	((uint8_t)0x02) //Enable interrupt...higher than...
#define L3GD20_INT1_XHIGH_DISABLE	((uint8_t)0x00) //Disable...

#define L3GD20_INT1_YLOW_ENABLE		((uint8_t)0x04) //Enable interrupt request on measured Y value lower than present threshold
#define L3GD20_INT1_YLOW_DISABLE	((uint8_t)0x00) //Disable...
#define L3GD20_INT1_YHIGH_ENABLE	((uint8_t)0x08) //Enable interrupt...higher than...
#define L3GD20_INT1_YHIGH_DISABLE	((uint8_t)0x00) //Disable...

#define L3GD20_INT1_ZLOW_ENABLE		((uint8_t)0x10) //Enable interrupt request on measured Z value lower than present threshold
#define L3GD20_INT1_ZLOW_DISABLE	((uint8_t)0x00) //Disable...
#define L3GD20_INT1_ZHIGH_ENABLE	((uint8_t)0x20) //Enable interrupt...higher than...
#define L3GD20_INT1_ZHIGH_DISABLE	((uint8_t)0x00) //Disable...

#define L3GD20_INT1_LATCH			((uint8_t)0x40) //Interrupt stays active(latched) until INT1_SRC reg is read
#define L3GD20_INT1_NOTLATCH		((uint8_t)0x00) //Interrupt auto clears when condition is no longer true
#define L3GD20_INT1_AND				((uint8_t)0x80) //Interrupt only if all conditions are met
#define L3GD20_INT1_OR				((uint8_t)0x00) //Interrupt if any condition is met



/*
 * INT1_SRC (read only)
 * This register tells "Why the interrupt happened"
 * It's like a note from Gyro saying "Hey! I triggered an interrupt, and here's what caused it"
 * When LIR = 1, interrupt stays ON until INT1_SRC is read
 */
#define L3GD20_INT1_XLOW_STT	((uint8_t)0x01) //X Low event occurred
#define L3GD20_INT1_XHIGH_STT	((uint8_t)0x02) //X High event occurred
#define L3GD20_INT1_YLOW_STT	((uint8_t)0x04) //Y Low...
#define L3GD20_INT1_YHIGH_STT	((uint8_t)0x08) //Y High...
#define L3GD20_INT1_ZLOW_STT	((uint8_t)0x10) //Z Low...
#define L3GD20_INT1_ZHIGH_STT	((uint8_t)0x20) //Z High...
#define L3GD20_INT1_INTACTIVE	((uint8_t)0x40)	//One or more interrupts have been generated



/*
 * Set thresholds of XYZ for interrupt events
 * thrs = threshold
 * XL(8bits) + XH(7bits) = 15 bits = 0 to 32767
 *
 */
#define L3GD20_THS_XL(x_thrs)	((uint8_t)((x_thrs) & 0xFF))		//Reg: INT1_THS_XL (0x33), 8 bits (masked with 0xFF just for safe, clear)
#define L3GD20_THS_XH(x_thrs)	((uint8_t)(((x_thrs) >> 8) & 0x7F))	//Reg: INT1_THS_XH (0x32), 7 bits (masked with 0x7F to keep 7 bits only)

#define L3GD20_THS_YL(y_thrs)	((uint8_t)((y_thrs) & 0xFF))		//Reg: INT1_THS_YL (0x35),...
#define L3GD20_THS_YH(y_thrs)	((uint8_t)(((y_thrs) >> 8) & 0x7F)) //Reg: INT1_THS_YH (0x34),...

#define L3GD20_THS_ZL(z_thrs)	((uint8_t)((z_thrs) & 0xFF))		//Reg: INT1_THS_ZL (0x37),...
#define L3GD20_THS_ZH(z_thrs)	((uint8_t)(((z_thrs) >> 8) & 0x7F)) //Reg: INT1_THS_ZH (0x36),...



/*
 * INT1_DURATION (default: 0000 0000)
 */
#define L3GD20_INT1_WAIT_ENABLE	 ((uint8_t)0x80) //Interrupt waits until the signal stays below the thrs for the same duration
#define L3GD20_INT1_WAIT_DISABLE ((uint8_t)0x00) //Interrupt turns OFF/ON immediately when the signal passes the thrs

#define L3GD20_INT1_DURATION_SET(duration) ((uint8_t)((duration) & 0x7F)) //Clamp to 7 bits, prevents overflow

#endif /* INC_L3GD20_H_ */
