// This file is part of "navx-rs", which is free software: you
// can redistribute it and/or modify it under the terms of the GNU General
// Public License version 3 as published by the Free Software Foundation. See
// <https://www.gnu.org/licenses/> for a copy.

#![allow(dead_code)]
#![allow(non_snake_case)]
/*******************************************************************/
/*******************************************************************/
/*                      Register Definitions                       */
/*******************************************************************/
/* NOTE:  All multi-byte registers are in little-endian format.    */
/*        All registers with 'signed' data are twos-complement.    */
/*        Data Type Summary:                                       */
/*        unsigned byte:           0   - 255    (8 bits)           */
/*        unsigned short:          0   - 65535  (16 bits)          */
/*        signed short:        -32768  - 32767  (16 bits)          */
/*        signed hundredeths:  -327.68 - 327.67 (16 bits)		   */
/*        unsigned hundredths:    0.0  - 655.35 (16 bits)          */
/*        signed thousandths:  -32.768 - 32.767 (16 bits)          */
/*        signed short ratio: -1/16384 - 1/16384 (16 bits)         */
/*        16:16:           -32768.9999 - 32767.9999 (32 bits)      */
/*        unsigned long:             0 - 4294967295 (32 bits)      */
/*******************************************************************/
#[derive(Shrinkwrap, Debug, Eq, PartialEq, From, Add)]
pub struct I16HundrethsFloat(i16);
#[derive(Shrinkwrap, Debug, Eq, PartialEq, From, Add)]
pub struct U16HundrethsFloat(u16);
#[derive(Shrinkwrap, Debug, Eq, PartialEq, From, Add)]
pub struct I16ThousandthsFloat(i16);
#[derive(Shrinkwrap, Debug, Eq, PartialEq, From, Add)]
pub struct I16RatioFloat(i16);
#[derive(Shrinkwrap, Debug, Eq, PartialEq, From, Add)]
pub struct IQ1616(i32);

/**********************************************/
/* Device Identification Registers            */
/**********************************************/

pub const NAVX_REG_WHOAMI: usize = 0x00; /* IMU_MODEL_XXX */
pub const NAVX_REG_HW_REV: usize = 0x01;
pub const NAVX_REG_FW_VER_MAJOR: usize = 0x02;
pub const NAVX_REG_FW_VER_MINOR: usize = 0x03;

/**********************************************/
/* Status and Control Registers               */
/**********************************************/

/* Read-write */
pub const NAVX_REG_UPDATE_RATE_HZ: usize = 0x04; /* Range:  4 - 50 [unsigned byte] */
/* Read-only */
/* Accelerometer Full-Scale Range:  in units of G [unsigned byte] */
pub const NAVX_REG_ACCEL_FSR_G: usize = 0x05;
/* Gyro Full-Scale Range (Degrees/Sec):  Range:  250, 500, 1000 or 2000 [unsigned short] */
pub const NAVX_REG_GYRO_FSR_DPS_L: usize = 0x06; /* Lower 8-bits of Gyro Full-Scale Range */
pub const NAVX_REG_GYRO_FSR_DPS_H: usize = 0x07; /* Upper 8-bits of Gyro Full-Scale Range */
pub const NAVX_REG_OP_STATUS: usize = 0x08; /* NAVX_OP_STATUS_XXX */
pub const NAVX_REG_CAL_STATUS: usize = 0x09; /* NAVX_CAL_STATUS_XXX */
pub const NAVX_REG_SELFTEST_STATUS: usize = 0x0A; /* NAVX_SELFTEST_STATUS_XXX */
pub const NAVX_REG_CAPABILITY_FLAGS_L: usize = 0x0B;
pub const NAVX_REG_CAPABILITY_FLAGS_H: usize = 0x0C;

/**********************************************/
/* Processed Data Registers                   */
/**********************************************/

pub const NAVX_REG_SENSOR_STATUS_L: usize = 0x10; /* NAVX_SENSOR_STATUS_XXX */
pub const NAVX_REG_SENSOR_STATUS_H: usize = 0x11;
/* Timestamp:  [unsigned long] */
pub const NAVX_REG_TIMESTAMP_L_L: usize = 0x12;
pub const NAVX_REG_TIMESTAMP_L_H: usize = 0x13;
pub const NAVX_REG_TIMESTAMP_H_L: usize = 0x14;
pub const NAVX_REG_TIMESTAMP_H_H: usize = 0x15;

/* Yaw, Pitch, Roll:  Range: -180.00 to 180.00 [signed hundredths] */
/* Compass Heading:   Range: 0.00 to 360.00 [unsigned hundredths] */
/* Altitude in Meters:  In units of meters [16:16] */

pub const NAVX_REG_YAW_L: usize = 0x16; /* Lower 8 bits of Yaw     */
pub const NAVX_REG_YAW_H: usize = 0x17; /* Upper 8 bits of Yaw     */
pub const NAVX_REG_ROLL_L: usize = 0x18; /* Lower 8 bits of Roll    */
pub const NAVX_REG_ROLL_H: usize = 0x19; /* Upper 8 bits of Roll    */
pub const NAVX_REG_PITCH_L: usize = 0x1A; /* Lower 8 bits of Pitch   */
pub const NAVX_REG_PITCH_H: usize = 0x1B; /* Upper 8 bits of Pitch   */
pub const NAVX_REG_HEADING_L: usize = 0x1C; /* Lower 8 bits of Heading */
pub const NAVX_REG_HEADING_H: usize = 0x1D; /* Upper 8 bits of Heading */
pub const NAVX_REG_FUSED_HEADING_L: usize = 0x1E; /* Upper 8 bits of Fused Heading */
pub const NAVX_REG_FUSED_HEADING_H: usize = 0x1F; /* Upper 8 bits of Fused Heading */
pub const NAVX_REG_ALTITUDE_I_L: usize = 0x20;
pub const NAVX_REG_ALTITUDE_I_H: usize = 0x21;
pub const NAVX_REG_ALTITUDE_D_L: usize = 0x22;
pub const NAVX_REG_ALTITUDE_D_H: usize = 0x23;

/* World-frame Linear Acceleration: In units of +/- G * 1000 [signed thousandths] */

pub const NAVX_REG_LINEAR_ACC_X_L: usize = 0x24; /* Lower 8 bits of Linear Acceleration X */
pub const NAVX_REG_LINEAR_ACC_X_H: usize = 0x25; /* Upper 8 bits of Linear Acceleration X */
pub const NAVX_REG_LINEAR_ACC_Y_L: usize = 0x26; /* Lower 8 bits of Linear Acceleration Y */
pub const NAVX_REG_LINEAR_ACC_Y_H: usize = 0x27; /* Upper 8 bits of Linear Acceleration Y */
pub const NAVX_REG_LINEAR_ACC_Z_L: usize = 0x28; /* Lower 8 bits of Linear Acceleration Z */
pub const NAVX_REG_LINEAR_ACC_Z_H: usize = 0x29; /* Upper 8 bits of Linear Acceleration Z */

/* Quaternion:  Range -1 to 1 [signed short ratio] */

pub const NAVX_REG_QUAT_W_L: usize = 0x2A; /* Lower 8 bits of Quaternion W */
pub const NAVX_REG_QUAT_W_H: usize = 0x2B; /* Upper 8 bits of Quaternion W */
pub const NAVX_REG_QUAT_X_L: usize = 0x2C; /* Lower 8 bits of Quaternion X */
pub const NAVX_REG_QUAT_X_H: usize = 0x2D; /* Upper 8 bits of Quaternion X */
pub const NAVX_REG_QUAT_Y_L: usize = 0x2E; /* Lower 8 bits of Quaternion Y */
pub const NAVX_REG_QUAT_Y_H: usize = 0x2F; /* Upper 8 bits of Quaternion Y */
pub const NAVX_REG_QUAT_Z_L: usize = 0x30; /* Lower 8 bits of Quaternion Z */
pub const NAVX_REG_QUAT_Z_H: usize = 0x31; /* Upper 8 bits of Quaternion Z */

/**********************************************/
/* Raw Data Registers                         */
/**********************************************/

/* Sensor Die Temperature:  Range +/- 150, In units of Centigrade * 100 [signed hundredths float */

pub const NAVX_REG_MPU_TEMP_C_L: usize = 0x32; /* Lower 8 bits of Temperature */
pub const NAVX_REG_MPU_TEMP_C_H: usize = 0x33; /* Upper 8 bits of Temperature */

/* Raw, Calibrated Angular Rotation, in device units.  Value in DPS = units / GYRO_FSR_DPS [signed short] */

pub const NAVX_REG_GYRO_X_L: usize = 0x34;
pub const NAVX_REG_GYRO_X_H: usize = 0x35;
pub const NAVX_REG_GYRO_Y_L: usize = 0x36;
pub const NAVX_REG_GYRO_Y_H: usize = 0x37;
pub const NAVX_REG_GYRO_Z_L: usize = 0x38;
pub const NAVX_REG_GYRO_Z_H: usize = 0x39;

/* Raw, Calibrated, Acceleration Data, in device units.  Value in G = units / ACCEL_FSR_G [signed short] */

pub const NAVX_REG_ACC_X_L: usize = 0x3A;
pub const NAVX_REG_ACC_X_H: usize = 0x3B;
pub const NAVX_REG_ACC_Y_L: usize = 0x3C;
pub const NAVX_REG_ACC_Y_H: usize = 0x3D;
pub const NAVX_REG_ACC_Z_L: usize = 0x3E;
pub const NAVX_REG_ACC_Z_H: usize = 0x3F;

/* Raw, Calibrated, Un-tilt corrected Magnetometer Data, in device units.  1 unit = 0.15 uTesla [signed short] */

pub const NAVX_REG_MAG_X_L: usize = 0x40;
pub const NAVX_REG_MAG_X_H: usize = 0x41;
pub const NAVX_REG_MAG_Y_L: usize = 0x42;
pub const NAVX_REG_MAG_Y_H: usize = 0x43;
pub const NAVX_REG_MAG_Z_L: usize = 0x44;
pub const NAVX_REG_MAG_Z_H: usize = 0x45;

/* Calibrated Pressure in millibars Valid Range:  10.00 Max:  1200.00 [16:16 float]  */

pub const NAVX_REG_PRESSURE_IL: usize = 0x46;
pub const NAVX_REG_PRESSURE_IH: usize = 0x47;
pub const NAVX_REG_PRESSURE_DL: usize = 0x48;
pub const NAVX_REG_PRESSURE_DH: usize = 0x49;

/* Pressure Sensor Die Temperature:  Range +/- 150.00C [signed hundredths] */

pub const NAVX_REG_PRESSURE_TEMP_L: u8 = 0x4A;
pub const NAVX_REG_PRESSURE_TEMP_H: u8 = 0x4B;

/**********************************************/
/* Calibration Registers                      */
/**********************************************/

/* Yaw Offset: Range -180.00 to 180.00 [signed hundredths] */

pub const NAVX_REG_YAW_OFFSET_L: u8 = 0x4C; /* Lower 8 bits of Yaw Offset */
pub const NAVX_REG_YAW_OFFSET_H: u8 = 0x4D; /* Upper 8 bits of Yaw Offset */

/* Quaternion Offset:  Range: -1 to 1 [signed short ratio]  */

pub const NAVX_REG_QUAT_OFFSET_W_L: u8 = 0x4E; /* Lower 8 bits of Quaternion W */
pub const NAVX_REG_QUAT_OFFSET_W_H: u8 = 0x4F; /* Upper 8 bits of Quaternion W */
pub const NAVX_REG_QUAT_OFFSET_X_L: u8 = 0x50; /* Lower 8 bits of Quaternion X */
pub const NAVX_REG_QUAT_OFFSET_X_H: u8 = 0x51; /* Upper 8 bits of Quaternion X */
pub const NAVX_REG_QUAT_OFFSET_Y_L: u8 = 0x52; /* Lower 8 bits of Quaternion Y */
pub const NAVX_REG_QUAT_OFFSET_Y_H: u8 = 0x53; /* Upper 8 bits of Quaternion Y */
pub const NAVX_REG_QUAT_OFFSET_Z_L: u8 = 0x54; /* Lower 8 bits of Quaternion Z */
pub const NAVX_REG_QUAT_OFFSET_Z_H: u8 = 0x55; /* Upper 8 bits of Quaternion Z */

/**********************************************/
/* Integrated Data Registers                  */
/**********************************************/

/* Integration Control (Write-Only)           */
pub const NAVX_REG_INTEGRATION_CTL: u8 = 0x56;
pub const NAVX_REG_PAD_UNUSED: u8 = 0x57;

/* Velocity:  Range -32768.9999 - 32767.9999 in units of Meters/Sec      */

pub const NAVX_REG_VEL_X_I_L: usize = 0x58;
pub const NAVX_REG_VEL_X_I_H: usize = 0x59;
pub const NAVX_REG_VEL_X_D_L: usize = 0x5A;
pub const NAVX_REG_VEL_X_D_H: usize = 0x5B;
pub const NAVX_REG_VEL_Y_I_L: usize = 0x5C;
pub const NAVX_REG_VEL_Y_I_H: usize = 0x5D;
pub const NAVX_REG_VEL_Y_D_L: usize = 0x5E;
pub const NAVX_REG_VEL_Y_D_H: usize = 0x5F;
pub const NAVX_REG_VEL_Z_I_L: usize = 0x60;
pub const NAVX_REG_VEL_Z_I_H: usize = 0x61;
pub const NAVX_REG_VEL_Z_D_L: usize = 0x62;
pub const NAVX_REG_VEL_Z_D_H: usize = 0x63;

/* Displacement:  Range -32768.9999 - 32767.9999 in units of Meters      */

pub const NAVX_REG_DISP_X_I_L: usize = 0x64;
pub const NAVX_REG_DISP_X_I_H: usize = 0x65;
pub const NAVX_REG_DISP_X_D_L: usize = 0x66;
pub const NAVX_REG_DISP_X_D_H: usize = 0x67;
pub const NAVX_REG_DISP_Y_I_L: usize = 0x68;
pub const NAVX_REG_DISP_Y_I_H: usize = 0x69;
pub const NAVX_REG_DISP_Y_D_L: usize = 0x6A;
pub const NAVX_REG_DISP_Y_D_H: usize = 0x6B;
pub const NAVX_REG_DISP_Z_I_L: usize = 0x6C;
pub const NAVX_REG_DISP_Z_I_H: usize = 0x6D;
pub const NAVX_REG_DISP_Z_D_L: usize = 0x6E;
pub const NAVX_REG_DISP_Z_D_H: usize = 0x6F;

pub const NAVX_REG_LAST: usize = NAVX_REG_DISP_Z_D_H;

/* NAVX_MODEL */

pub const NAVX_MODEL_NAVX_MXP: u8 = 0x32;

/* NAVX_CAL_STATUS */

pub const NAVX_CAL_STATUS_IMU_CAL_STATE_MASK: u8 = 0x03;
pub const NAVX_CAL_STATUS_IMU_CAL_INPROGRESS: u8 = 0x00;
pub const NAVX_CAL_STATUS_IMU_CAL_ACCUMULATE: u8 = 0x01;
pub const NAVX_CAL_STATUS_IMU_CAL_COMPLETE: u8 = 0x02;

pub const NAVX_CAL_STATUS_MAG_CAL_COMPLETE: u8 = 0x04;
pub const NAVX_CAL_STATUS_BARO_CAL_COMPLETE: u8 = 0x08;

/* NAVX_SELFTEST_STATUS */

pub const NAVX_SELFTEST_STATUS_COMPLETE: u8 = 0x80;

pub const NAVX_SELFTEST_RESULT_GYRO_PASSED: u8 = 0x01;
pub const NAVX_SELFTEST_RESULT_ACCEL_PASSED: u8 = 0x02;
pub const NAVX_SELFTEST_RESULT_MAG_PASSED: u8 = 0x04;
pub const NAVX_SELFTEST_RESULT_BARO_PASSED: u8 = 0x08;

/* NAVX_OP_STATUS */

pub const NAVX_OP_STATUS_INITIALIZING: u8 = 0x00;
pub const NAVX_OP_STATUS_SELFTEST_IN_PROGRESS: u8 = 0x01;
pub const NAVX_OP_STATUS_ERROR: u8 = 0x02; /* E.g., Self-test fail, I2C error */
pub const NAVX_OP_STATUS_IMU_AUTOCAL_IN_PROGRESS: u8 = 0x03;
pub const NAVX_OP_STATUS_NORMAL: u8 = 0x04;

/* NAVX_SENSOR_STATUS */

pub const NAVX_SENSOR_STATUS_MOVING: u8 = 0x01;
pub const NAVX_SENSOR_STATUS_YAW_STABLE: u8 = 0x02;
pub const NAVX_SENSOR_STATUS_MAG_DISTURBANCE: u8 = 0x04;
pub const NAVX_SENSOR_STATUS_ALTITUDE_VALID: u8 = 0x08;
pub const NAVX_SENSOR_STATUS_SEALEVEL_PRESS_SET: u8 = 0x10;
pub const NAVX_SENSOR_STATUS_FUSED_HEADING_VALID: u8 = 0x20;

/* NAVX_REG_CAPABILITY_FLAGS (Aligned w/NAV6 Flags, see imu.rs) */

pub const NAVX_CAPABILITY_FLAG_OMNIMOUNT: i16 = 0x0004;
pub const NAVX_CAPABILITY_FLAG_OMNIMOUNT_CONFIG_MASK: i16 = 0x0038;
pub const NAVX_CAPABILITY_FLAG_VEL_AND_DISP: i16 = 0x0040;
pub const NAVX_CAPABILITY_FLAG_YAW_RESET: i16 = 0x0080;
pub const NAVX_CAPABILITY_FLAG_AHRSPOS_TS: i16 = 0x0100;

/* NAVX_OMNIMOUNT_CONFIG */

pub const OMNIMOUNT_DEFAULT: u8 = 0; /* Same as Y_Z_UP */
pub const OMNIMOUNT_YAW_X_UP: u8 = 1;
pub const OMNIMOUNT_YAW_X_DOWN: u8 = 2;
pub const OMNIMOUNT_YAW_Y_UP: u8 = 3;
pub const OMNIMOUNT_YAW_Y_DOWN: u8 = 4;
pub const OMNIMOUNT_YAW_Z_UP: u8 = 5;
pub const OMNIMOUNT_YAW_Z_DOWN: u8 = 6;

/* NAVX_INTEGRATION_CTL */

pub const NAVX_INTEGRATION_CTL_RESET_VEL_X: u8 = 0x01;
pub const NAVX_INTEGRATION_CTL_RESET_VEL_Y: u8 = 0x02;
pub const NAVX_INTEGRATION_CTL_RESET_VEL_Z: u8 = 0x04;
pub const NAVX_INTEGRATION_CTL_RESET_DISP_X: u8 = 0x08;
pub const NAVX_INTEGRATION_CTL_RESET_DISP_Y: u8 = 0x10;
pub const NAVX_INTEGRATION_CTL_RESET_DISP_Z: u8 = 0x20;
pub const NAVX_INTEGRATION_CTL_VEL_AND_DISP_MASK: u8 = 0x3F;

pub const NAVX_INTEGRATION_CTL_RESET_YAW: u8 = 0x80;

use byteorder::{ByteOrder, LittleEndian};

#[inline(always)]
pub fn dec_prot_u16(b: &[u8]) -> u16 {
    LittleEndian::read_u16(b)
}

#[inline(always)]
pub fn enc_prot_u16(val: u16, b: &mut [u8]) {
    LittleEndian::write_u16(b, val);
}

#[inline(always)]
pub fn dec_prot_i16(b: &[u8]) -> i16 {
    LittleEndian::read_i16(b)
}
#[inline(always)]
pub fn enc_prot_i16(val: i16, b: &mut [u8]) {
    LittleEndian::write_i16(b, val);
}

#[inline(always)]
pub fn dec_prot_u32(b: &[u8]) -> u32 {
    LittleEndian::read_u32(b)
}

#[inline(always)]
pub fn dec_prot_i32(b: &[u8]) -> i32 {
    LittleEndian::read_i32(b)
}
#[inline(always)]
pub fn enc_prot_i32(val: i32, b: &mut [u8]) {
    LittleEndian::write_i32(b, val)
}

/* -327.68 to +327.68 */
#[inline(always)]
pub fn dec_prot_signed_hundreths_float(b: &[u8]) -> f32 {
    let mut signed_angle = dec_prot_i16(b) as f32;
    signed_angle /= 100.;
    return signed_angle;
}

#[inline(always)]
pub fn enc_prot_signed_hundreths_float(input: f32, b: &mut [u8]) {
    let input_as_int = (input * 100.) as i16;
    enc_prot_i16(input_as_int, b);
}

#[inline(always)]
pub fn encodeSignedHundredthsFloat(input: f32) -> I16HundrethsFloat {
    I16HundrethsFloat::from((input * 100.) as i16)
}
#[inline(always)]
pub fn encodeUnsignedHundredthsFloat(input: f32) -> U16HundrethsFloat {
    U16HundrethsFloat::from((input * 100.) as u16)
}

#[inline(always)]
pub fn encodeRatioFloat(input_ratio: f32) -> I16RatioFloat {
    I16RatioFloat::from((input_ratio * 32768.0) as i16)
}
#[inline(always)]
pub fn encodeSignedThousandthsFloat(input: f32) -> I16ThousandthsFloat {
    I16ThousandthsFloat::from((input * 1000.) as i16)
}

/* 0 to 655.35 */
#[inline(always)]
pub fn decodeProtocolUnsignedHundredthsFloat(b: &[u8]) -> f32 {
    let mut unsigned_float = dec_prot_u16(b) as f32;
    unsigned_float /= 100.;
    unsigned_float
}
#[inline(always)]
pub fn encodeProtocolUnsignedHundredthsFloat(input: f32, b: &mut [u8]) {
    let input_as_uint = (input * 100.) as u16;
    enc_prot_u16(input_as_uint, b);
}

/* -32.768 to +32.768 */
#[inline(always)]
pub fn decodeProtocolSignedThousandthsFloat(b: &[u8]) -> f32 {
    let mut signed_angle = dec_prot_i16(b) as f32;
    signed_angle /= 1000.;
    return signed_angle;
}
#[inline(always)]
pub fn encodeProtocolSignedThousandthsFloat(input: f32, b: &mut [u8]) {
    let input_as_int = (input * 1000.) as i16;
    enc_prot_i16(input_as_int, b);
}

/* In units of -1 to 1, multiplied by 16384 */
#[inline(always)]
pub fn decodeProtocolRatio(b: &[u8]) -> f32 {
    let mut ratio = dec_prot_i16(b) as f32;
    ratio /= 32768.;
    return ratio;
}
#[inline(always)]
pub fn encodeProtocolRatio(ratio: f32, b: &mut [u8]) {
    enc_prot_i16((ratio * 32768.) as i16, b);
}

/* <int16>.<uint16> (-32768.9999 to 32767.9999) */
#[inline(always)]
pub fn decodeProtocol1616Float(b: &[u8]) -> f32 {
    let mut result = dec_prot_i32(b) as f32;
    result /= 65536.;
    return result;
}
#[inline(always)]
pub fn encodeProtocol1616Float(val: f32, b: &mut [u8]) {
    let packed_float = (val * 65536.) as i32;
    enc_prot_i32(packed_float, b);
}

pub fn buildCRCLookupTable(table: &mut [u8]) {
    let mut crc: u8;
    let length = table.len();
    if length == 256 {
        for i in 0..length {
            crc = i as u8;
            for _ in 0..8 {
                if crc & 1 != 0 {
                    crc ^= CRC7_POLY;
                }
                crc >>= 1;
            }
            table[i] = crc;
        }
    }
}

pub fn getCRCWithTable(table: &[u8], message: &[u8], length: u8) -> u8 {
    let mut crc = 0;

    for i in 0..length {
        crc ^= message[i as usize];
        crc = table[crc as usize];
    }
    crc
}

pub fn getCRC(message: &[u8], length: u8) -> u8 {
    let mut crc = 0;

    for i in 0..length {
        crc ^= message[i as usize];
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc ^= CRC7_POLY;
            }
            crc >>= 1;
        }
    }
    crc
}

pub const CRC7_POLY: u8 = 0x91;
