/*
 * imu.c
 * Driver for LSM9DS0 IMU
 * Filip Dziwulski
 */

#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>
#include "imu.h"
#include "defs.h"


#define ACCEL_DATA_RATE		0x5	//50hz

#define DEFAULT_ACCEL_RANGE	2	//G
#define DEFAULT_GYRO_RANGE	225	//degrees/s
#define DEFAULT_MAG_RANGE	4	//gauss

#define IMU_I2C_TIMEOUT		500 //ms

#define IMU_DEBUG			0

/***********************************************************
 * DATA
 ***********************************************************/
extern uint32_t g_ticks;

/***********************************************************
 * LOCAL FUNCTIONS
 ***********************************************************/
void imu_write(uint8_t address, uint8_t reg, uint8_t data);
void imu_read(uint8_t address, uint8_t reg, uint16_t *data);

void initAcc(void);
void initRot(void);
void initMag(void);
void init_I2C1(void);


//I2C helpers (should probably be pulled out into an I2C file)
uint8_t imu_receiveData(uint16_t* data, uint32_t timeout);
uint8_t imu_sendSUB(uint8_t reg, uint32_t timeout, uint8_t autoNext);
uint8_t imu_sendSAD_R(uint8_t address, uint32_t timeout);
uint8_t imu_sendSAD_W(uint8_t address, uint32_t timeout);
uint8_t imu_sendST(uint32_t timeout);
uint8_t imu_sendSR(uint32_t timeout);
uint8_t imu_sendDATA(uint8_t data, uint32_t timeout);
uint8_t imu_sendSP(void);
uint8_t waitForI2CEvent(uint32_t state, uint32_t timeout);
uint8_t waitForI2CAvailable(uint32_t timeout);

uint8_t isTimeout(uint32_t startTime, uint32_t timeout_ms);


/***********************************************************
 * uint8_t init_IMU(void)
 *
 * Initializes I2C, And IMU components
 *
 ***********************************************************/
uint8_t init_IMU(void)
{
	init_I2C1();
	initAcc();
	initRot();
	initMag();

	return TRUE;
}

/***********************************************************
 * void initAcc(void)
 *
 * Initializes Accelerometer portion of IMU
 *
 ***********************************************************/
void initAcc(void)
{

	uint8_t dataRate 	= 0;
	uint8_t enable		= 0;
	uint8_t dataOut		= 0;

	dataRate = (((uint8_t)ACCEL_DATA_RATE << 4) & (0xF0));
	enable	= (uint8_t) 0x07;

	dataOut = dataRate | enable;

	//enable accelerometer and set data rate
	imu_write(IMU_ACCELMAG_I2C_ADDR, CTRL_REG1_XM, dataOut);

}

/***********************************************************
 * void initRot(void)
 *
 * Initializes Gyro portion of IMU
 *
 ***********************************************************/
void initRot(void)
{
	//enable gyro
	imu_write(IMU_ROTATION_I2C_ADDR, CTRL_REG1_G, (uint8_t) 0x0F);
}

/***********************************************************
 * void initMag(void)
 *
 * Initializes Magnetic component of IMU
 *
 ***********************************************************/
void initMag(void)
{
	imu_write(IMU_ACCELMAG_I2C_ADDR, CTRL_REG5_XM, (uint8_t) 0x14);
	imu_write(IMU_ACCELMAG_I2C_ADDR, CTRL_REG7_XM, (uint8_t) 0x00);
}

/***********************************************************
 * void imu_getAcc(struct Components *acc)
 *
 * Polls IMU for acceleration data
 *
 ***********************************************************/
void imu_getAcc(struct Components *acc)
{
	short int 	datax	= 0x0000,
				datay	= 0x0000,
				dataz	= 0x0000;

	imu_read(IMU_ACCELMAG_I2C_ADDR, OUT_X_L_A, &datax);
	acc->x = (((float) datax)/32767.0) * (float)DEFAULT_ACCEL_RANGE;

	imu_read(IMU_ACCELMAG_I2C_ADDR, OUT_Y_L_A, &datay);
	acc->y = (((float) datay)/32767.0) * (float)DEFAULT_ACCEL_RANGE;

	imu_read(IMU_ACCELMAG_I2C_ADDR, OUT_Z_L_A, &dataz);
	acc->z = (((float) dataz)/32767.0) * (float)DEFAULT_ACCEL_RANGE;

}

/***********************************************************
 * void imu_getMag(struct Components *mag)
 *
 * Polls IMU for magnetic data
 *
 ***********************************************************/
void imu_getMag(struct Components *mag)
{
	short int 	datax	= 0x0000,
				datay	= 0x0000,
				dataz	= 0x0000;

	imu_read(IMU_ACCELMAG_I2C_ADDR, OUT_X_L_M, &datax);
	mag->x = (((float) datax)/32767.0) * (float)DEFAULT_MAG_RANGE;

	imu_read(IMU_ACCELMAG_I2C_ADDR, OUT_Y_L_M, &datay);
	mag->y = (((float) datay)/32767.0) * (float)DEFAULT_MAG_RANGE;

	imu_read(IMU_ACCELMAG_I2C_ADDR, OUT_Z_L_M, &dataz);
	mag->z = (((float) dataz)/32767.0) * (float)DEFAULT_MAG_RANGE;
}

/***********************************************************
 * void imu_getRot(struct Components *rot)
 *
 * Polls IMU for Gyro data
 *
 ***********************************************************/
void imu_getRot(struct Components *rot)
{

	short int 	datax	= 0x0000,
				datay	= 0x0000,
				dataz	= 0x0000;

	imu_read(IMU_ROTATION_I2C_ADDR, OUT_X_L_G, &datax);
	rot->x = (((float) datax)/32767.0) * (float)DEFAULT_GYRO_RANGE;

	imu_read(IMU_ROTATION_I2C_ADDR, OUT_Y_L_G, &datay);
	rot->y = (((float) datay)/32767.0) * (float)DEFAULT_GYRO_RANGE;

	imu_read(IMU_ROTATION_I2C_ADDR, OUT_Z_L_G, &dataz);
	rot->z = (((float) dataz)/32767.0) * (float)DEFAULT_GYRO_RANGE;
}

/***********************************************************
 * void imu_getDOF(imuDof_S *dof)
 *
 * Polls the IMU for all DOF data
 *
 ***********************************************************/
void imu_getDOF(imuDof_S *dof)
{
	imu_getAcc(&dof->acc);
	imu_getMag(&dof->mag);
	imu_getRot(&dof->rot);
}

/***********************************************************
 * void imu_write(uint8_t address, uint8_t reg, uint8_t data)
 *
 * Routine for writing a byte of data to IMU
 *
 ***********************************************************/
void imu_write(uint8_t address, uint8_t reg, uint8_t data)
{
    waitForI2CAvailable(IMU_I2C_TIMEOUT);
    imu_sendST(IMU_I2C_TIMEOUT);
    imu_sendSAD_W(address, IMU_I2C_TIMEOUT);
    imu_sendSUB(reg, IMU_I2C_TIMEOUT, FALSE);
    imu_sendDATA(data, IMU_I2C_TIMEOUT);
    imu_sendSP();
}

/***********************************************************
 * void imu_read(uint8_t address, uint8_t reg, uint16_t *data)
 *
 * Routine for reading two bytes of data from IMU
 *
 ***********************************************************/
void imu_read(uint8_t address, uint8_t reg, uint16_t *data)
{
	waitForI2CAvailable(IMU_I2C_TIMEOUT);
    imu_sendST(IMU_I2C_TIMEOUT);
    imu_sendSAD_W(address, IMU_I2C_TIMEOUT);
    imu_sendSUB(reg, IMU_I2C_TIMEOUT, TRUE);
    imu_sendSR(IMU_I2C_TIMEOUT);
    imu_sendSAD_R(address, IMU_I2C_TIMEOUT);
    imu_receiveData(data, IMU_I2C_TIMEOUT);
    imu_sendSP();

	return;
}

/***********************************************************
 * void init_I2C1(void)
 *
 * Initializes I2C1 bus
 *
 ***********************************************************/
void init_I2C1(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB

	// Connect I2C1 pins to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA

	// configure I2C1
	I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses

	I2C_Init(IMU_I2C_PERIPH, &I2C_InitStruct);				// init I2C1

	// enable I2C1
	I2C_Cmd(IMU_I2C_PERIPH, ENABLE);
}

/***********************************************************
 * uint8_t imu_receiveData(uint16_t* data, uint32_t timeout)
 *
 * Routine for receiving data from IMU
 *
 ***********************************************************/
uint8_t imu_receiveData(uint16_t* data, uint32_t timeout)
{

	uint16_t data_L	=	0x00;
	uint16_t data_H = 	0x00;

	I2C_NACKPositionConfig(IMU_I2C_PERIPH, I2C_NACKPosition_Next);

	if( waitForI2CEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, timeout) )
	{
		data_L = I2C_ReceiveData(IMU_I2C_PERIPH);
	}
	else
	{
		my_printf("\t\tFailure receiving Data_L\r\n");
		return FALSE;
	}

	if( waitForI2CEvent(I2C_EVENT_MASTER_BYTE_RECEIVED, timeout) )
	{
		data_H = I2C_ReceiveData(IMU_I2C_PERIPH);
	}
	else
	{
		my_printf("\t\tFailure receiving Data_H\r\n");
		return FALSE;
	}

	*data = (((uint16_t) data_H) << 8 ) | ((uint16_t) data_L);

	return TRUE;

}

/***********************************************************
 * uint8_t imu_sendSP(void)
 *
 * sends stop to IMU
 *
 ***********************************************************/
uint8_t imu_sendSP(void)
{
	I2C_GenerateSTOP(IMU_I2C_PERIPH, ENABLE);

	return TRUE;
}

/***********************************************************
 * uint8_t imu_sendDATA(uint8_t data, uint32_t timeout)
 *
 * Sends data to IMU
 *
 ***********************************************************/
uint8_t imu_sendDATA(uint8_t data, uint32_t timeout)
{
	I2C_SendData(IMU_I2C_PERIPH, data);

	return waitForI2CEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, timeout);
}

/***********************************************************
 * uint8_t imu_sendSUB(uint8_t reg, uint32_t timeout, uint8_t autoNext)
 *
 * sends a sub address to IMU
 *
 ***********************************************************/
uint8_t imu_sendSUB(uint8_t reg, uint32_t timeout, uint8_t autoNext)
{
	uint8_t regOut = 0x00;

	if(TRUE == autoNext)
	{
		regOut = reg | 0x80;
	}
	else
	{
		regOut = reg;
	}

	I2C_SendData(IMU_I2C_PERIPH, regOut);

	return waitForI2CEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED, timeout);
}

/***********************************************************
 * uint8_t imu_sendSAD_R(uint8_t address, uint32_t timeout)
 *
 * Send address and Receive command to IMU
 *
 ***********************************************************/
uint8_t imu_sendSAD_R(uint8_t address, uint32_t timeout)
{

	I2C_Send7bitAddress(IMU_I2C_PERIPH, address, I2C_Direction_Receiver);

	return waitForI2CEvent(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, timeout);
}

/***********************************************************
 * uint8_t imu_sendSAD_W(uint8_t address, uint32_t timeout)
 *
 * Sends address and write command to IMU
 *
 ***********************************************************/
uint8_t imu_sendSAD_W(uint8_t address, uint32_t timeout)
{
	I2C_Send7bitAddress(IMU_I2C_PERIPH, address, I2C_Direction_Transmitter);

	return waitForI2CEvent(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, timeout);
}

/***********************************************************
 * uint8_t imu_sendST(uint32_t timeout)
 *
 * Sends Start to the IMU
 *
 ***********************************************************/
uint8_t imu_sendST(uint32_t timeout)
{
	I2C_GenerateSTART(IMU_I2C_PERIPH, ENABLE);

	return waitForI2CEvent(I2C_EVENT_MASTER_MODE_SELECT, timeout);
}

/***********************************************************
 * uint8_t imu_sendSR(uint32_t timeout)
 *
 * Sends a Start resend
 *
 ***********************************************************/
uint8_t imu_sendSR(uint32_t timeout)
{
	return imu_sendST(timeout);
}

/***********************************************************
 * uint8_t waitForI2CEvent(uint32_t state, uint32_t timeout)
 *
 * Waits for I2C state, returns whether a timeout occured
 *
 ***********************************************************/
uint8_t waitForI2CEvent(uint32_t state, uint32_t timeout)
{
	uint32_t startTime	=	g_ticks;

	while(!I2C_CheckEvent(IMU_I2C_PERIPH, state) && (!isTimeout(startTime, timeout)));

	return isTimeout(startTime, timeout) ? FALSE : TRUE;
}

/***********************************************************
 * uint8_t waitForI2CAvailable(uint32_t timeout)
 *
 * Waits for I2C bus to become available, returns whether
 * a timeout occured.
 *
 ***********************************************************/
uint8_t waitForI2CAvailable(uint32_t timeout)
{
	uint32_t startTime	=	g_ticks;

	while(I2C_GetFlagStatus(IMU_I2C_PERIPH, I2C_FLAG_BUSY)  && (!isTimeout(startTime, timeout)));

	return isTimeout(startTime, timeout) ? FALSE : TRUE;
}

/***********************************************************
 * uint8_t isTimeout(uint32_t startTime, uint32_t timeout_ms)
 *
 * Checks for timeout
 *
 ***********************************************************/
uint8_t isTimeout(uint32_t startTime, uint32_t timeout_ms)
{
	if((g_ticks - startTime) >= timeout_ms)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}
