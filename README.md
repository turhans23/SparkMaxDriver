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

## Known limitations / TODO

- The heartbeat needs to be tied to a fixed-rate timer/task; calling it in an uncontrolled loop can overflow the Tx mailboxes.
- Watch out for race conditions when reading data written from the interrupt in the main code — using a `GetData` function with a critical section is recommended.
