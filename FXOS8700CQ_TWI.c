/*** @file
* @brief FXOS8700CQ I2C Driver
*
***/

#include <stdbool.h>
#include "boards.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "FXOS8700CQ_TWI.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"

#define fxos_debug

/* TWI instance */

const nrf_drv_twi_t p_twi = NRF_DRV_TWI_INSTANCE(0);

/* initialize twi init function */	
bool twi_init(void)
{
	uint32_t err_code;
	nrf_drv_twi_config_t p_twi_config = NRF_DRV_TWI_DEFAULT_CONFIG(0);
	
	p_twi_config.scl = TWI_SCL_PIN;
	p_twi_config.sda = TWI_SDA_PIN;
	p_twi_config.frequency = NRF_TWI_FREQ_400K;
	
	err_code = nrf_drv_twi_init(&p_twi, &p_twi_config, NULL, NULL);
	
	if(err_code != NRF_SUCCESS)	return false ;
	
	nrf_drv_twi_enable(&p_twi);
	return true;
}

/* Write in a registerss */
void FXOS8700CQ_TWI_register_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t value)
{
    ret_code_t ret_code;
    uint8_t tx_data[2]= {reg_addr, value};
    ret_code = nrf_drv_twi_tx(&p_twi, slave_addr, tx_data, sizeof(tx_data), false);
		//APP_ERROR_CHECK (ret_code);
		#ifdef fxos_debug
			SEGGER_RTT_printf(0, "\nWrite register error code:%x", ret_code);
		#endif
}

/* Read register on slave */
void FXOS8700CQ_TWI_register_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t *p_data, uint32_t bytes)
{   
    ret_code_t ret_code;
   	ret_code = nrf_drv_twi_tx(&p_twi, slave_addr, &reg_addr, 1, true);
		#ifdef fxos_debug
			SEGGER_RTT_printf(0, "\nRead register tx error code:%x", ret_code);
		#endif
		//APP_ERROR_CHECK (ret_code);
    ret_code = nrf_drv_twi_rx(&p_twi, slave_addr, p_data, bytes);
		#ifdef fxos_debug
		SEGGER_RTT_printf(0, "\nRead register rx error cdoe:%x", ret_code);
		//SEGGER_RTT_printf(0, "\nRead register val:%d", pdata);
		#endif		
		//APP_ERROR_CHECK (ret_code);
}

//Set FXOS stanby mode write 0x00 in register
void FXOS8700CQ_standby()
{
	uint8_t ctrl_reg1_val[1];
	FXOS8700CQ_TWI_register_read(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_CTRL_REG1, &ctrl_reg1_val[0], 1);
	#ifdef fxos_debug
	SEGGER_RTT_printf(0, "\nFXOS8700CQ_CTRL_REG1_VAL:%d", ctrl_reg1_val[0]);
	#endif		
	
	FXOS8700CQ_TWI_register_write(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_CTRL_REG1, FXOS8700CQ_STANDBY_MODE_VAL); 
}

//Set FXOS normal mode write 0x0D in register
void FXOS8700CQ_normal()
{
	uint8_t ctrl_reg1_val[1];
	
	FXOS8700CQ_TWI_register_write (FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_CTRL_REG1, FXOS8700CQ_NORMAL_MODE_VAL); 
	FXOS8700CQ_TWI_register_read(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_CTRL_REG1, &ctrl_reg1_val[0], 1);
	#ifdef fxos_debug
	SEGGER_RTT_printf(0, "\nFXOS8700CQ_CTRL_REG1_VAL:%d", ctrl_reg1_val[0]);
	#endif		
	
}

// Read status and the three channels of accelerometer and magnetometer data from FXOS8700CQ (13 bytes)
uint8_t FXOS8700CQ_TWI_acc_data_read(fxos_data_t *pAccData, fxos_data_t *pMagData)
{
	 

	uint8_t   Buffer[6]; // read buffer
	//Read FXOS8700CQ_READ_LEN=13 bytes (status byte and the six channels of data)
	FXOS8700CQ_TWI_register_read(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_OUT_X_MSB, &Buffer[0], sizeof(Buffer));
	if(sizeof(Buffer) == 6) 		   
	{
		//Copy the 14 bit accelerometer byte data into 16 bit words
		pAccData->x =Buffer[0];
		pAccData->y =Buffer[2];
		pAccData->z =Buffer[4];
	
		#ifdef fxos_debug
		SEGGER_RTT_printf(0,"\nAcc_Buffer_size Buffer[0]:%d ", Buffer[0]);
		SEGGER_RTT_printf(0,"\nAcc_Buffer_size Buffer[2]:%d ", Buffer[2]);
		SEGGER_RTT_printf(0,"\nAcc_Buffer_size Buffer[4]:%d ", Buffer[4]);
		#endif		
	
	}
	else	
	{
		return  0x02; 
	}
	return 0x00;
}

//init function
bool FXOS8700CQ_TWI_init(void)
{

	uint8_t data;
	uint8_t r_data[1];
	
	
	if(!twi_init())	return false;			
	
	//Read WHOAMI register 
	FXOS8700CQ_TWI_register_read(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_WHO_AM_I, &r_data[0], 1);
	
	
	data = r_data[0];
	#ifdef fxos_debug
	SEGGER_RTT_printf(0, "\nWHO_AM_I Read Value:%d", data);
	#endif		
		
	//Compear WHOAMI register value 
	if (data != FXOS8700CQ_WHO_AM_I_VAL)		return false;

	//stand by mode 
	FXOS8700CQ_standby();	
   
	//Write 0x1F in register FXOS8700CQ_CTRL_REG1 for reduce noice from over sampling.
	//FXOS8700CQ_TWI_register_write(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_M_CTRL_REG1, FXOS8700CQ_M_CTRL_REG1_VAL); 
		
	//Write 0x1F in register FXOS8700CQ_M_CTRL_REG2 for magnetic reset etc
	//FXOS8700CQ_TWI_register_write(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_M_CTRL_REG2, FXOS8700CQ_M_CTRL_REG2_VAL); 
	
	//Write 0x01 in register FXOS8700CQ_XYZ_DATA_CFG for acc 4G range 
	FXOS8700CQ_TWI_register_write(FXOS8700CQ_SLAVE_ADDR, FXOS8700CQ_XYZ_DATA_CFG, FXOS8700CQ_XYZ_DATA_CFG_VAL); 
	
	//normal mode  
	FXOS8700CQ_normal();	
	return true;
}
