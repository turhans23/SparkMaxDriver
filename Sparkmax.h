#ifndef INC_SPARKMAX_H_
#define INC_SPARKMAX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* SparkMAX CAN API Base ID definitions */
#define SPARKMAX_BASE_DUTY_ID       			 0x02050080  
#define SPARKMAX_BASE_HEARTBEAT_ID  			 0x02052C80  
#define SPARKMAX_BASE_PERIODICSTATUS0_ID   0X02051800  
#define SPARKMAX_BASE_PERIODICSTATUS1_ID   0x02051840  // RPM, temperature, voltage, current.
#define SPARKMAX_BASE_PERIODICSTATUS2_ID   0x02051880 
#define SPARKMAX_BASE_PERIODICSTATUS3_ID   0x020518C0 
#define SPARKMAX_BASE_PERIODICSTATUS4_ID   0x02051900


#define SPARKMAX_ENTER_CRITICAL()  __disable_irq()
#define SPARKMAX_EXIT_CRITICAL()   __enable_irq()
	
/* Status Definitions */
typedef enum {
    SPARKMAX_OK = 0,
    SPARKMAX_ERR_TIMEOUT,
    SPARKMAX_ERR_PARAM,
    SPARKMAX_ERR_CAN
} SparkMax_Status_t;

/* Driver Handler */
typedef struct {
    CAN_HandleTypeDef *hcan;
    uint8_t motor_id;        								// 1,2,3 ...   						
    CAN_TxHeaderTypeDef duty_header;
    CAN_TxHeaderTypeDef heartbeat_header;		
    float current_rpm;      								// Values Read from drivers.			 			 
		uint8_t temp;
		float voltage;
		float current;
		uint32_t last_update_time;
} SparkMax_t;

typedef struct {
    float rpm;
    uint8_t temp;
    float voltage;
    float current;
    uint32_t last_update_time;
} SparkMax_Data_t;


/* Function Declerations */
SparkMax_Status_t SparkMax_Init(SparkMax_t *motor, CAN_HandleTypeDef *hcan, uint8_t motor_id);
SparkMax_Status_t SparkMax_InitAll(SparkMax_t *motors, CAN_HandleTypeDef *hcan, uint8_t count);

SparkMax_Status_t SparkMax_SetDuty(SparkMax_t *motor, float duty_cycle);
SparkMax_Status_t SparkMax_SendHeartbeat(SparkMax_t *motor);

HAL_StatusTypeDef SparkMax_FilterConfig(CAN_HandleTypeDef *hcan, uint8_t filter_bank, uint32_t fifo, uint32_t filterBaseAddr);
SparkMax_Status_t SparkMax_ProcessStatus1(SparkMax_t *motors, uint8_t motor_count, uint32_t ext_id, const uint8_t *data);

SparkMax_Status_t SparkMax_GetData(SparkMax_t *motor, SparkMax_Data_t *out);
#ifdef __cplusplus
}
#endif

#endif /* INC_SPARKMAX_H_ */