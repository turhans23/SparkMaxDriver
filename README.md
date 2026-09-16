# SparkMaxDriver

A simple C library for controlling REV SparkMax motor controllers over CAN bus, written for STM32 (HAL library) based projects.

## What it does

- Initializes multiple SparkMax controllers at once
- Sends duty cycle commands
- Sends 50Hz heartbeat messages (required to keep the SparkMax from shutting itself down)
- Parses periodic status data (RPM, temperature, voltage, current) coming from the CAN RX interrupt and stores it into a struct
- Configures CAN filters

Designed to work in both bare-metal and FreeRTOS projects.

## Files

- `Sparkmax.c` / `Sparkmax.h` — the driver itself
- Your project needs the HAL library set up via STM32CubeIDE or Keil (`main.h` is expected to provide `CAN_HandleTypeDef`, etc.)

## Usage

```c
SparkMax_t motors[3];

SparkMax_InitAll(motors, &hcan1, 3);
SparkMax_FilterConfig(&hcan1, 0, CAN_RX_FIFO0, SPARKMAX_BASE_PERIODICSTATUS1_ID);

// Inside CAN RX callback:
SparkMax_ProcessStatus1(motors, 3, rx_header.ExtId, rx_data);

// Inside main loop:
SparkMax_SendHeartbeat(&motors[0]);   // should be called every 20ms
SparkMax_SetDuty(&motors[0], 0.5f);   // %50 duty
```

```c
#include "main.h"
#include "sparkmax.h"

#define NUM_MOTORS 2
SparkMax_t motors[NUM_MOTORS];

/* CAN RX Interrupt Callback */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        if (rx_header.IDE == CAN_ID_EXT && 
           ((rx_header.ExtId & 0xFFFFFFC0U) == SPARKMAX_BASE_PERIODICSTATUS1_ID))
        {
            SparkMax_ProcessStatus1(motors, NUM_MOTORS, rx_header.ExtId, rx_data);
        }
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_CAN1_Init();

    /* 1. Initialize Motors (ID: 1 and 2) */
    SparkMax_InitAll(motors, &hcan1, NUM_MOTORS);

    /* 2. Configure CAN Hardware Filter for Status 1 frames */
    SparkMax_FilterConfig(&hcan1, 0, CAN_RX_FIFO0, SPARKMAX_BASE_PERIODICSTATUS1_ID);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    uint32_t last_heartbeat = 0;

    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* Send Heartbeat & Commands every 20ms (50Hz) */
        if (now - last_heartbeat >= 20)
        {
            last_heartbeat = now;

            for (uint8_t i = 0; i < NUM_MOTORS; i++)
            {
                SparkMax_SendHeartbeat(&motors[i]);
                SparkMax_SetDuty(&motors[i], 0.25f); /* 25% Forward */
            }
        }

        /* Safely read telemetry */
        SparkMax_Data_t motor1_data;
        if (SparkMax_GetData(&motors[0], &motor1_data) == SPARKMAX_OK)
        {
            /* motor1_data.rpm, motor1_data.voltage, etc. */
        }
    }
}
```

```c
SparkMax_t motors[3];

SparkMax_InitAll(motors, &hcan1, 3);
SparkMax_FilterConfig(&hcan1, 0, CAN_RX_FIFO0, SPARKMAX_BASE_PERIODICSTATUS1_ID);

// Inside CAN RX callback:
SparkMax_ProcessStatus1(motors, 3, rx_header.ExtId, rx_data);

// Inside main loop:
SparkMax_SendHeartbeat(&motors[0]);   // should be called every 20ms
SparkMax_SetDuty(&motors[0], 0.5f);   // %50 duty
```

```c
#include "FreeRTOS.h"
#include "task.h"
#include "sparkmax.h"

#define NUM_MOTORS 4
SparkMax_t motors[NUM_MOTORS];

/* CAN RX Interrupt Callback */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        if (rx_header.IDE == CAN_ID_EXT && 
           ((rx_header.ExtId & 0xFFFFFFC0U) == SPARKMAX_BASE_PERIODICSTATUS1_ID))
        {
            SparkMax_ProcessStatus1(motors, NUM_MOTORS, rx_header.ExtId, rx_data);
        }
    }
}

/* Transmit Task: Distributes heartbeats smoothly across the 20ms window */
void MotorControlTask(void *argument)
{
    uint8_t current_motor = 0;
    /* 20ms period / 4 motors = 5ms interval per motor */
    const TickType_t xFrequency = pdMS_TO_TICKS(5); 

    for (;;)
    {
        /* Send commands to one motor per tick (avoids filling the 3 Tx mailboxes) */
        SparkMax_SendHeartbeat(&motors[current_motor]);
        SparkMax_SetDuty(&motors[current_motor], 0.30f);

        current_motor = (current_motor + 1) % NUM_MOTORS;

        vTaskDelay(xFrequency);
    }
}

/* Telemetry Reader Task */
void TelemetryTask(void *argument)
{
    SparkMax_Data_t telemetry;

    for (;;)
    {
        for (uint8_t i = 0; i < NUM_MOTORS; i++)
        {
            if (SparkMax_GetData(&motors[i], &telemetry) == SPARKMAX_OK)
            {
                /* Check for 250ms timeout (disconnected motor) */
                if ((HAL_GetTick() - telemetry.last_update_time) > 250)
                {
                    /* Handle motor timeout / error */
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```


## Known limitations / TODO

- The heartbeat needs to be tied to a fixed-rate timer/task; calling it in an uncontrolled loop can overflow the Tx mailboxes.
- Watch out for race conditions when reading data written from the interrupt in the main code — using a `GetData` function with a critical section is recommended.
