#![allow(dead_code)]

mod ahrs;
mod registers;

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

#[derive(Debug, Copy, Clone)]
pub struct YPRUpdate {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    pub compass_heading: f32,
}

#[derive(Debug, Copy, Clone)]
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

#[derive(Debug, Copy, Clone)]
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

#[derive(Debug, Copy, Clone)]
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
