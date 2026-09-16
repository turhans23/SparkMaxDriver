#include "Rake_sparkmax.h"
#include <string.h>

static const uint8_t SPARKMAX_HEARTBEAT_DATA[8] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};

/* Initializes only one SparkMax  */
SparkMax_Status_t SparkMax_Init(SparkMax_t *motor, CAN_HandleTypeDef *hcan, uint8_t motor_id)
{
    if (motor == NULL || hcan == NULL || motor_id == 0)
		{
       return SPARKMAX_ERR_PARAM;
    }

	 /* General Parameters           */	
    motor->hcan = hcan;
    motor->motor_id = motor_id;
    motor->current_rpm = 0.0f;
		motor->temp = 0;
		motor->voltage = 0.0f;
		motor->current = 0.0f;
	
	 /* Duty Cycle Header Parameters */	
    motor->duty_header.ExtId = SPARKMAX_BASE_DUTY_ID | (uint32_t)motor_id;
    motor->duty_header.IDE = CAN_ID_EXT;
    motor->duty_header.RTR = CAN_RTR_DATA;
    motor->duty_header.DLC = 8;
    motor->duty_header.TransmitGlobalTime = DISABLE;

	 /* Heartbeat Header Parameters  */		
    motor->heartbeat_header.ExtId = SPARKMAX_BASE_HEARTBEAT_ID | (uint32_t)motor_id;
    motor->heartbeat_header.IDE = CAN_ID_EXT;
    motor->heartbeat_header.RTR = CAN_RTR_DATA;
    motor->heartbeat_header.DLC = 8;
    motor->heartbeat_header.TransmitGlobalTime = DISABLE;

    return SPARKMAX_OK;
}

/*  Initializes 'count' number of SparkMax 								    */
/*  *motors parameter is an array of SparkMax_t struct type   */

SparkMax_Status_t SparkMax_InitAll(SparkMax_t *motors, CAN_HandleTypeDef *hcan, uint8_t count)
{
    if (motors == NULL || hcan == NULL ||  count == 0)
		{
        return SPARKMAX_ERR_PARAM;
    }

    for (uint8_t i = 0; i < count; i++) 
		{
        SparkMax_Status_t status = SparkMax_Init(&motors[i], hcan, i+1);
			
        if (status != SPARKMAX_OK) 
				{
            return status;
        }
    }

    return SPARKMAX_OK;
}

/* Can filter for SparkMax drivers. This function may called for different filter bank values and FIFO's. filterBaseAddr
indicates the base address of the filter and may arranged for different periodic status addresses which are defined in Sparkmax.h */

HAL_StatusTypeDef SparkMax_FilterConfig(CAN_HandleTypeDef *hcan, uint8_t filter_bank, uint32_t fifo, uint32_t filterBaseAddr)
{
    CAN_FilterTypeDef filterConfig;
    filterConfig.FilterBank = filter_bank;
    filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    filterConfig.FilterFIFOAssignment = fifo;
    filterConfig.FilterActivation = ENABLE;
    filterConfig.SlaveStartFilterBank = 14;

    /* This mask allows values between 0x02051840-0x0205184F (ids between 0-15) to pass filter. User may change if needed  */
    uint32_t baseID = filterBaseAddr;
    uint32_t mask   = 0xFFFFFFF0U;
    
    /* 3 bits shifted because of can filters' register */
		uint32_t id_shifted   = baseID << 3;
    uint32_t mask_shifted = mask << 3;
    filterConfig.FilterIdHigh     = (id_shifted >> 16) & 0xFFFF;
    filterConfig.FilterIdLow      = (id_shifted & 0xFFFF) | (1 << 2);
    filterConfig.FilterMaskIdHigh = (mask_shifted >> 16) & 0xFFFF;
    filterConfig.FilterMaskIdLow  = (mask_shifted & 0xFFFF) | (1 << 2);

    return HAL_CAN_ConfigFilter(hcan, &filterConfig);
}

/* Calling both SetDuty and Heartbeet functions continously in a loop without controlling with a time based source 
is not recommended. This action will cause CanTx Mailboxes to overflow. */

SparkMax_Status_t SparkMax_SetDuty(SparkMax_t *motor, float duty_cycle)
{
    if (motor == NULL || motor->hcan == NULL) 
		{
        return SPARKMAX_ERR_PARAM;
    }
		
		if (duty_cycle > 1.0f) duty_cycle = 1.0f;
    if (duty_cycle < -1.0f) duty_cycle = -1.0f;
		
    uint8_t tx_data[8] = {0}; 									
    uint32_t tx_mailbox;													
    memcpy(tx_data, &duty_cycle, sizeof(float));  

    if (HAL_CAN_AddTxMessage(motor->hcan, &(motor->duty_header), tx_data, &tx_mailbox) != HAL_OK) 
		{ 
        return SPARKMAX_ERR_CAN;
    }

    return SPARKMAX_OK;
}

/* Heartbeet function must be excecuted minimum of every 20ms (50Hz) to SparkMax operate without shutting itself down.*/
SparkMax_Status_t SparkMax_SendHeartbeat(SparkMax_t *motor)
{
    if (motor == NULL || motor->hcan == NULL)
		{
        return SPARKMAX_ERR_PARAM;
    }

     uint32_t tx_mailbox;

   
    if (HAL_CAN_AddTxMessage(motor->hcan, &motor->heartbeat_header, (uint8_t *)SPARKMAX_HEARTBEAT_DATA, &tx_mailbox) != HAL_OK)
		{
        return SPARKMAX_ERR_CAN;
    }

    return SPARKMAX_OK;
}

/* Function that should be called in CanRxCallback function. This function is called  
every single time when a data arrives so for n motor drivers it will be executed for n times */

SparkMax_Status_t SparkMax_ProcessStatus1(SparkMax_t *motors, uint8_t motor_count, uint32_t ext_id, const uint8_t *data)
{
    if (motors == NULL || data == NULL || motor_count == 0)
		{
        return SPARKMAX_ERR_PARAM;
    }

    uint8_t incoming_id = (uint8_t)(ext_id & 0x3F);
		
    if (incoming_id < 1 || incoming_id > motor_count) 
		{
        return SPARKMAX_ERR_PARAM; 
    }

    uint8_t idx = incoming_id - 1; 

/* 
		SparMax drivers send 8 Bytes 
		4 Byte is Motor RPM
		1 Byte Temperature
		12 bit Voltage (1.5 Bytes)
		12 bit Current (1.5 Bytes)
		
*/		
		
    memcpy(&motors[idx].current_rpm, data, sizeof(float));
    motors[idx].temp = data[4];

    uint16_t raw_volt = (uint16_t)(data[5] | ((data[6] & 0x0F) << 8));
    motors[idx].voltage = raw_volt * 0.05f;

    uint16_t raw_curr = (uint16_t)(((data[6] & 0xF0) >> 4) | (data[7] << 4));
    motors[idx].current = raw_curr * 0.05f;
		
		motors[idx].last_update_time = HAL_GetTick();
		
    return SPARKMAX_OK;
}

SparkMax_Status_t SparkMax_GetData(SparkMax_t *motor, SparkMax_Data_t *out)
{
    if (motor == NULL || out == NULL) return SPARKMAX_ERR_PARAM;

    SPARKMAX_ENTER_CRITICAL();
    out->rpm             = motor->current_rpm;
    out->temp             = motor->temp;
    out->voltage         = motor->voltage;
    out->current         = motor->current;
    out->last_update_time = motor->last_update_time;
    SPARKMAX_EXIT_CRITICAL();

    return SPARKMAX_OK;
}