// Copyright 2018 navx-rs Developers.
// Licensed under the Apache License, Version 2.0 <LICENSE-APACHE or
// http://www.apache.org/licenses/LICENSE-2.0> or the MIT license
// <LICENSE-MIT or http://opensource.org/licenses/MIT>, at your
// option. This file may not be copied, modified, or distributed
// except according to those terms.

#![allow(dead_code)]

//TODO these are u32 cause they used to be macros, we need to find out what they actually are
pub const PACKET_START_CHAR: u8 = 33u8;
pub const CHECKSUM_LENGTH: u32 = 2;
pub const TERMINATOR_LENGTH: u32 = 2;
pub const PROTOCOL_FLOAT_LENGTH: u32 = 7;
pub const MSGID_YPR_UPDATE: u8 = 121u8;
pub const YPR_UPDATE_YAW_VALUE_INDEX: u32 = 2;
pub const YPR_UPDATE_ROLL_VALUE_INDEX: u32 = 9;
pub const YPR_UPDATE_PITCH_VALUE_INDEX: u32 = 16;
pub const YPR_UPDATE_COMPASS_VALUE_INDEX: u32 = 23;
pub const YPR_UPDATE_CHECKSUM_INDEX: u32 = 30;
pub const YPR_UPDATE_TERMINATOR_INDEX: u32 = 32;
pub const YPR_UPDATE_MESSAGE_LENGTH: u32 = 34;
pub const MSGID_QUATERNION_UPDATE: u8 = 113u8;
pub const QUATERNION_UPDATE_QUAT1_VALUE_INDEX: u32 = 2;
pub const QUATERNION_UPDATE_QUAT2_VALUE_INDEX: u32 = 6;
pub const QUATERNION_UPDATE_QUAT3_VALUE_INDEX: u32 = 10;
pub const QUATERNION_UPDATE_QUAT4_VALUE_INDEX: u32 = 14;
pub const QUATERNION_UPDATE_ACCEL_X_VALUE_INDEX: u32 = 18;
pub const QUATERNION_UPDATE_ACCEL_Y_VALUE_INDEX: u32 = 22;
pub const QUATERNION_UPDATE_ACCEL_Z_VALUE_INDEX: u32 = 26;
pub const QUATERNION_UPDATE_MAG_X_VALUE_INDEX: u32 = 30;
pub const QUATERNION_UPDATE_MAG_Y_VALUE_INDEX: u32 = 34;
pub const QUATERNION_UPDATE_MAG_Z_VALUE_INDEX: u32 = 38;
pub const QUATERNION_UPDATE_TEMP_VALUE_INDEX: u32 = 42;
pub const QUATERNION_UPDATE_CHECKSUM_INDEX: u32 = 49;
pub const QUATERNION_UPDATE_TERMINATOR_INDEX: u32 = 51;
pub const QUATERNION_UPDATE_MESSAGE_LENGTH: u32 = 53;
pub const MSGID_GYRO_UPDATE: u8 = 103u8;
pub const GYRO_UPDATE_MESSAGE_LENGTH: u32 = 46;
pub const GYRO_UPDATE_GYRO_X_VALUE_INDEX: u32 = 2;
pub const GYRO_UPDATE_GYRO_Y_VALUE_INDEX: u32 = 6;
pub const GYRO_UPDATE_GYRO_Z_VALUE_INDEX: u32 = 10;
pub const GYRO_UPDATE_ACCEL_X_VALUE_INDEX: u32 = 14;
pub const GYRO_UPDATE_ACCEL_Y_VALUE_INDEX: u32 = 18;
pub const GYRO_UPDATE_ACCEL_Z_VALUE_INDEX: u32 = 22;
pub const GYRO_UPDATE_MAG_X_VALUE_INDEX: u32 = 26;
pub const GYRO_UPDATE_MAG_Y_VALUE_INDEX: u32 = 30;
pub const GYRO_UPDATE_MAG_Z_VALUE_INDEX: u32 = 34;
pub const GYRO_UPDATE_TEMP_VALUE_INDEX: u32 = 38;
pub const GYRO_UPDATE_CHECKSUM_INDEX: u32 = 42;
pub const GYRO_UPDATE_TERMINATOR_INDEX: u32 = 44;
pub const MSGID_STREAM_CMD: u8 = 83u8;
pub const STREAM_CMD_STREAM_TYPE_YPR: u8 = 121u8;
pub const STREAM_CMD_STREAM_TYPE_QUATERNION: u8 = 113u8;
pub const STREAM_CMD_STREAM_TYPE_INDEX: u32 = 2;
pub const STREAM_CMD_UPDATE_RATE_HZ_INDEX: u32 = 3;
pub const STREAM_CMD_CHECKSUM_INDEX: u32 = 5;
pub const STREAM_CMD_TERMINATOR_INDEX: u32 = 7;
pub const STREAM_CMD_MESSAGE_LENGTH: u32 = 9;
pub const MSG_ID_STREAM_RESPONSE: u8 = 115u8;
pub const STREAM_RESPONSE_STREAM_TYPE_INDEX: u32 = 2;
pub const STREAM_RESPONSE_GYRO_FULL_SCALE_DPS_RANGE: u32 = 3;
pub const STREAM_RESPONSE_ACCEL_FULL_SCALE_G_RANGE: u32 = 7;
pub const STREAM_RESPONSE_UPDATE_RATE_HZ: u32 = 11;
pub const STREAM_RESPONSE_YAW_OFFSET_DEGREES: u32 = 15;
pub const STREAM_RESPONSE_QUAT1_OFFSET: u32 = 22;
pub const STREAM_RESPONSE_QUAT2_OFFSET: u32 = 26;
pub const STREAM_RESPONSE_QUAT3_OFFSET: u32 = 30;
pub const STREAM_RESPONSE_QUAT4_OFFSET: u32 = 34;
pub const STREAM_RESPONSE_FLAGS: u32 = 38;
pub const STREAM_RESPONSE_CHECKSUM_INDEX: u32 = 42;
pub const STREAM_RESPONSE_TERMINATOR_INDEX: u32 = 44;
pub const STREAM_RESPONSE_MESSAGE_LENGTH: u32 = 46;
pub const NAV6_FLAG_MASK_CALIBRATION_STATE: u32 = 3;
pub const NAV6_CALIBRATION_STATE_WAIT: u32 = 0;
pub const NAV6_CALIBRATION_STATE_ACCUMULATE: u32 = 1;
pub const NAV6_CALIBRATION_STATE_COMPLETE: u32 = 2;
pub const IMU_PROTOCOL_MAX_MESSAGE_LENGTH: u32 = QUATERNION_UPDATE_MESSAGE_LENGTH;

#[derive(Debug, Default, Clone)]
pub struct YPRUpdate {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    pub compass_heading: f32,
}

#[derive(Debug, Default, Clone)]
pub struct GyroUpdate {
    pub gyro_x: i16,
    pub gyro_y: i16,
    pub gyro_z: i16,
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub mag_x: i16,
    pub mag_y: i16,
    pub mag_z: i16,
    pub temp_c: f32,
}

#[derive(Debug, Default, Clone)]
pub struct QuaternionUpdate {
    pub q1: i16,
    pub q2: i16,
    pub q3: i16,
    pub q4: i16,
    pub accel_x: i16,
    pub accel_y: i16,
    pub accel_z: i16,
    pub mag_x: i16,
    pub mag_y: i16,
    pub mag_z: i16,
    pub temp_c: f32,
}

#[derive(Debug, Default, Clone)]
pub struct StreamResponse {
    pub stream_type: u8,
    pub gyro_fsr_dps: i16,
    pub accel_fsr_g: i16,
    pub update_rate_hz: i16,
    pub yaw_offset_degrees: f32,
    pub q1_offset: i16,
    pub q2_offset: i16,
    pub q3_offset: i16,
    pub q4_offset: i16,
    pub flags: i16,
}

// static int encodeYPRUpdate(char *protocol_buffer, float yaw, float pitch, float roll, float compass_heading)
//     {
//         unimplemented!()
//     }

//     static int encodeQuaternionUpdate(char *protocol_buffer,
//                                       uint16_t q1, uint16_t q2, uint16_t q3, uint16_t q4,
//                                       uint16_t accel_x, uint16_t accel_y, uint16_t accel_z,
//                                       int16_t mag_x, int16_t mag_y, int16_t mag_z,
//                                       float temp_c)
//     {
//         unimplemented!()
//     }

//     static int encodeGyroUpdate(char *protocol_buffer,
//                                 uint16_t gyro_x, uint16_t gyro_y, uint16_t gyro_z,
//                                 uint16_t accel_x, uint16_t accel_y, uint16_t accel_z,
//                                 int16_t mag_x, int16_t mag_y, int16_t mag_z,
//                                 float temp_c)
//     {
//         unimplemented!()
//     }

//     static int encodeStreamCommand(char *protocol_buffer, char stream_type, unsigned char update_rate_hz)
//     {
//         unimplemented!()
//     }

//     static int encodeStreamResponse(char *protocol_buffer, char stream_type,
//                                     uint16_t gyro_fsr_dps, uint16_t accel_fsr_g, uint16_t update_rate_hz,
//                                     float yaw_offset_degrees,
//                                     uint16_t q1_offset, uint16_t q2_offset, uint16_t q3_offset, uint16_t q4_offset,
//                                     uint16_t flags)
//     {
//         unimplemented!()
//     }

//     static int decodeStreamResponse(char *buffer, int length, struct StreamResponse &rsp)
//     {
//         unimplemented!()
//     }

//     static int decodeStreamCommand(char *buffer, int length, char &stream_type, unsigned char &update_rate_hz)
//     {
//         unimplemented!()
//     }

//     static int decodeYPRUpdate(char *buffer, int length, struct YPRUpdate &update)
//     {
//         unimplemented!()
//     }

//     static int decodeQuaternionUpdate(char *buffer, int length, struct QuaternionUpdate &update)
//     {
//        unimplemented!()
//     }

//     static int decodeGyroUpdate(char *buffer, int length, struct GyroUpdate &update)
//     {
//         unimplemented!()
//     }

//   protected:
//     static void encodeTermination(char *buffer, int total_length, int content_length)
//     {
//         unimplemented!()
//     }

//     // Formats a float as follows
//     //
//     // e.g., -129.235
//     //
//     // "-129.24"
//     //
//     // e.g., 23.4
//     //
//     // "+023.40"

//     static void encodeProtocolFloat(float f, char *buff)
//     {
//         unimplemented!()
//     }

//     static void encodeProtocolUint16(uint16_t value, char *buff)
//     {
//         unimplemented!()
//     }

//     static uint16_t decodeProtocolUint16(char *uint16_string)
//     {
//         unimplemented!()
//     }

//     /* 0 to 655.35 */
//     static inline float decodeProtocolUnsignedHundredthsFloat(char *uint8_unsigned_hundredths_float)
//     {
//         unimplemented!()
//     }
//     static inline void encodeProtocolUnsignedHundredthsFloat(float input, char *uint8_unsigned_hundredths_float)
//     {
//        unimplemented!()
//     }

//     static bool verifyChecksum(char *buffer, int content_length)
//     {
//         unimplemented!()
//     }

//     static unsigned char decodeUint8(char *checksum)
//     {
//         unimplemented!()
//     }

//     static float decodeProtocolFloat(char *buffer)
//     {
//        unimplemented!()
//     }
