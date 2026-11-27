/* Frozen shared contract v1.1 — owner: member 1 (integration).
 * Merge of member2/member6 handoff contracts. The only signature change from
 * the v1 draft is W4ImuSync_Init(): member4 implements it as osStatus_t, so the
 * contract is frozen to match. Do not re-declare it as void anywhere.
 * This file must sit FIRST on the include path so it overrides the member copies.
 */
#ifndef W4_CONTRACT_H
#define W4_CONTRACT_H
#include <stdint.h>
#include <stdbool.h>
#include "cmsis_os2.h"
typedef struct { uint32_t seq, stamp_ms; uint16_t offset, length; uint8_t half; } W4LidarBlock;
typedef enum { W4_CMD_GOTO=1, W4_CMD_STOP=2, W4_CMD_MAZE=3, W4_CMD_RETURN=4 } W4CmdType;
typedef struct { uint32_t cmd_id; W4CmdType type; float x_m,y_m; } W4Command;
typedef struct { uint32_t cmd_id; uint16_t index; uint8_t last; float x_m,y_m; } W4Waypoint;
typedef struct { uint32_t stamp_ms; float x_m,y_m,theta_rad; uint8_t valid; } W4Pose;
typedef struct { uint32_t stamp_ms; float heading_rad,gz_rad_s; uint8_t valid,calibrated; } W4ImuSample;
typedef struct { uint32_t stamp_ms; int32_t left_delta,right_delta; uint8_t valid; } W4EncoderDelta;
void W4LidarTask(void *argument);
void W4ImuTask(void *argument);
void W4BluetoothRxTask(void *argument);
void W4PathTask(void *argument);
void W4ControlTask(void *argument);
void W4OledTask(void *argument);
void W4BluetoothTxTask(void *argument);
void W4Queues_Init(void);
osStatus_t W4ImuSync_Init(void);   /* frozen: matches member4 implementation */
void W4I2cSync_Init(void);
bool W4Imu_GetSnapshot(W4ImuSample *out);   /* integration adapter over member4 raw snapshot */
bool W4Pose_GetSnapshot(W4Pose *out);
bool W4Encoder_TakeDelta(W4EncoderDelta *out);
void W4Motor_SetWheelSpeeds(float left_mps,float right_mps);
void W4Motor_Stop(void);
void W4Nav_RequestStop(void);
bool W4I2c_MemRead(uint8_t addr7,uint8_t reg,uint8_t *buf,uint16_t len,uint32_t timeout_ms);
bool W4I2c_Write(uint8_t addr7,const uint8_t *buf,uint16_t len,uint32_t timeout_ms);
bool W4I2c_MemWrite(uint8_t addr7,uint8_t reg,const uint8_t *buf,uint16_t len,uint32_t timeout_ms);
bool W4I2c_Recover(uint32_t timeout_ms);
bool W4Telemetry_SubmitLine(const char *line,uint16_t len);
#endif