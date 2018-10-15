// This file is part of "navx-rs", which is free software: you
// can redistribute it and/or modify it under the terms of the GNU General
// Public License version 3 as published by the Free Software Foundation. See
// <https://www.gnu.org/licenses/> for a copy.

/*****************************************************************************/
/* This protocol, introduced first with the navX MXP, expands upon the IMU   */
/* protocol by adding the following new functionality:                       */
/*         																	 */
/* AHRS Update:  Includes Fused Heading and Altitude Info                    */
/* Magnetometer Calibration:  Enables configuration of coefficients from PC  */
/* Board Identity:  Enables retrieval of Board Identification Info           */
/* Fusion Tuning:  Enables configuration of key thresholds/coefficients used */
/*                 in data fusion algorithms from a remote client            */
/*                                                                           */
/* In addition, the navX enable stream command has been extended with a new  */
/* Stream type, in order to enable AHRS Updates.                             */
/*****************************************************************************/

macro_rules! char_u8_const {
    ( $name:ident, $ch:expr, $value:expr, $test:ident ) => {
        pub const $name: u8 = $value;
        #[cfg(test)]
        mod $test {
            #[test]
            fn test() {
                assert_eq!(char::from(super::$name), $ch);
            }
        }
    };
}

// const BINARY_PACKET_INDICATOR_CHAR: u8 = 35;
char_u8_const!(BINARY_PACKET_INDICATOR_CHAR, '#', 35, bpic);

/* AHRS Protocol encodes certain data in binary format, unlike the IMU  */
/* protocol, which encodes all data in ASCII characters.  Thus, the     */
/* packet start and message termination sequences may occur within the  */
/* message content itself.  To support the binary format, the binary    */
/* message has this format:                                             */
/*                                                                      */
/* [start][binary indicator][len][msgid]<MESSAGE>[checksum][terminator] */
/*                                                                      */
/* (The binary indicator and len are not present in the ASCII protocol) */
/*                                                                      */
/* The [len] does not include the length of the start and binary        */
/* indicator characters, but does include all other message items,      */
/* including the checksum and terminator sequence.                      */
#[allow(non_camel_case_types)]
#[repr(i32)]
enum AHRS_TUNING_VAR_ID {
    UNSPECIFIED = 0,
    MOTION_THRESHOLD = 1,          /* In G */
    YAW_STABLE_THRESHOLD = 2,      /* In Degrees */
    MAG_DISTURBANCE_THRESHOLD = 3, /* Ratio */
    SEA_LEVEL_PRESSURE = 4,        /* Millibars */
                                   // MIN_TUNING_VAR_ID = MOTION_THRESHOLD,
                                   // MAX_TUNING_VAR_ID = SEA_LEVEL_PRESSURE,
}

impl AHRS_TUNING_VAR_ID {
    pub const MIN_TUNING_VAR_ID: i32 = AHRS_TUNING_VAR_ID::MOTION_THRESHOLD as i32;
    pub const MAX_TUNING_VAR_ID: i32 = AHRS_TUNING_VAR_ID::SEA_LEVEL_PRESSURE as i32;
}

#[allow(non_camel_case_types)]
#[repr(i32)]
enum AHRS_DATA_TYPE {
    TUNING_VARIABLE = 0,
    MAG_CALIBRATION = 1,
    BOARD_IDENTITY = 2,
}

#[allow(non_camel_case_types)]
#[repr(i32)]
enum AHRS_DATA_ACTION {
    DATA_GET = 0,
    DATA_SET = 1,
    DATA_SET_TO_DEFAULT = 2,
}

pub const DATA_GETSET_SUCCESS: u32 = 0;
pub const DATA_GETSET_ERROR: u32 = 1;

// AHRS Update Packet - e.g., !a[yaw][pitch][roll][heading][altitude][fusedheading][accelx/y/z][angular rot x/y/z][opstatus][fusionstatus][cr][lf]

// const MSGID_AHRS_UPDATE: u8 = 97;
// #[test]
// fn test_update_char() {
//     assert_eq!(char::from(MSGID_AHRS_UPDATE), 'a')
// }

char_u8_const!(MSGID_AHRS_UPDATE, 'a', 97, mau);
pub const AHRS_UPDATE_YAW_VALUE_INDEX: usize = 4; /* Degrees.  Signed Hundredths */
pub const AHRS_UPDATE_ROLL_VALUE_INDEX: usize = 6; /* Degrees.  Signed Hundredths */
pub const AHRS_UPDATE_PITCH_VALUE_INDEX: usize = 8; /* Degrees.  Signed Hundredeths */
pub const AHRS_UPDATE_HEADING_VALUE_INDEX: usize = 10; /* Degrees.  Unsigned Hundredths */
pub const AHRS_UPDATE_ALTITUDE_VALUE_INDEX: usize = 12; /* Meters.   Signed 16:16 */
pub const AHRS_UPDATE_FUSED_HEADING_VALUE_INDEX: usize = 16; /* Degrees.  Unsigned Hundredths */
pub const AHRS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX: usize = 18; /* Inst. G.  Signed Thousandths */
pub const AHRS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX: usize = 20; /* Inst. G.  Signed Thousandths */
pub const AHRS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX: usize = 22; /* Inst. G.  Signed Thousandths */
pub const AHRS_UPDATE_CAL_MAG_X_VALUE_INDEX: usize = 24; /* Int16 (Device Units) */
pub const AHRS_UPDATE_CAL_MAG_Y_VALUE_INDEX: usize = 26; /* Int16 (Device Units) */
pub const AHRS_UPDATE_CAL_MAG_Z_VALUE_INDEX: usize = 28; /* Int16 (Device Units) */
pub const AHRS_UPDATE_CAL_MAG_NORM_RATIO_VALUE_INDEX: usize = 30; /* Ratio.  Unsigned Hundredths */
pub const AHRS_UPDATE_CAL_MAG_SCALAR_VALUE_INDEX: usize = 32; /* Coefficient. Signed q16:16 */
pub const AHRS_UPDATE_MPU_TEMP_VAUE_INDEX: usize = 36; /* Centigrade.  Signed Hundredths */
pub const AHRS_UPDATE_RAW_MAG_X_VALUE_INDEX: usize = 38; /* INT16 (Device Units) */
pub const AHRS_UPDATE_RAW_MAG_Y_VALUE_INDEX: usize = 40; /* INT16 (Device Units) */
pub const AHRS_UPDATE_RAW_MAG_Z_VALUE_INDEX: usize = 42; /* INT16 (Device Units) */
pub const AHRS_UPDATE_QUAT_W_VALUE_INDEX: usize = 44; /* INT16 */
pub const AHRS_UPDATE_QUAT_X_VALUE_INDEX: usize = 46; /* INT16 */
pub const AHRS_UPDATE_QUAT_Y_VALUE_INDEX: usize = 48; /* INT16 */
pub const AHRS_UPDATE_QUAT_Z_VALUE_INDEX: usize = 50; /* INT16 */
pub const AHRS_UPDATE_BARO_PRESSURE_VALUE_INDEX: usize = 52; /* millibar.  Signed 16:16 */
pub const AHRS_UPDATE_BARO_TEMP_VAUE_INDEX: usize = 56; /* Centigrade.  Signed  Hundredths */
pub const AHRS_UPDATE_OPSTATUS_VALUE_INDEX: usize = 58; /* NAVX_OP_STATUS_XXX */
pub const AHRS_UPDATE_SENSOR_STATUS_VALUE_INDEX: usize = 59; /* NAVX_SENSOR_STATUS_XXX */
pub const AHRS_UPDATE_CAL_STATUS_VALUE_INDEX: usize = 60; /* NAVX_CAL_STATUS_XXX */
pub const AHRS_UPDATE_SELFTEST_STATUS_VALUE_INDEX: usize = 61; /* NAVX_SELFTEST_STATUS_XXX */
pub const AHRS_UPDATE_MESSAGE_CHECKSUM_INDEX: usize = 62;
pub const AHRS_UPDATE_MESSAGE_TERMINATOR_INDEX: usize = 64;
pub const AHRS_UPDATE_MESSAGE_LENGTH: usize = 66;

// AHRSAndPositioning Update Packet (similar to AHRS, but removes magnetometer and adds velocity/displacement) */
// const MSGID_AHRSPOS_UPDATE: u8 = 112;
// #[test]
// fn test_update_char() {
//     assert_eq!(char::from(MSGID_AHRSPOS_UPDATE), 'p')
// }
char_u8_const!(MSGID_AHRSPOS_UPDATE, 'p', 112, mapu);
pub const AHRSPOS_UPDATE_YAW_VALUE_INDEX: usize = 4; /* Degrees.  Signed Hundredths */
pub const AHRSPOS_UPDATE_ROLL_VALUE_INDEX: usize = 6; /* Degrees.  Signed Hundredths */
pub const AHRSPOS_UPDATE_PITCH_VALUE_INDEX: usize = 8; /* Degrees.  Signed Hundredeths */
pub const AHRSPOS_UPDATE_HEADING_VALUE_INDEX: usize = 10; /* Degrees.  Unsigned Hundredths */
pub const AHRSPOS_UPDATE_ALTITUDE_VALUE_INDEX: usize = 12; /* Meters.   Signed 16:16 */
pub const AHRSPOS_UPDATE_FUSED_HEADING_VALUE_INDEX: usize = 16; /* Degrees.  Unsigned Hundredths */
pub const AHRSPOS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX: usize = 18; /* Inst. G.  Signed Thousandths */
pub const AHRSPOS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX: usize = 20; /* Inst. G.  Signed Thousandths */
pub const AHRSPOS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX: usize = 22; /* Inst. G.  Signed Thousandths */
pub const AHRSPOS_UPDATE_VEL_X_VALUE_INDEX: usize = 24; /* Signed 16:16, in meters/sec */
pub const AHRSPOS_UPDATE_VEL_Y_VALUE_INDEX: usize = 28; /* Signed 16:16, in meters/sec */
pub const AHRSPOS_UPDATE_VEL_Z_VALUE_INDEX: usize = 32; /* Signed 16:16, in meters/sec */
pub const AHRSPOS_UPDATE_DISP_X_VALUE_INDEX: usize = 36; /* Signed 16:16, in meters */
pub const AHRSPOS_UPDATE_DISP_Y_VALUE_INDEX: usize = 40; /* Signed 16:16, in meters */
pub const AHRSPOS_UPDATE_DISP_Z_VALUE_INDEX: usize = 44; /* Signed 16:16, in meters */
pub const AHRSPOS_UPDATE_QUAT_W_VALUE_INDEX: usize = 48; /* INT16 */
pub const AHRSPOS_UPDATE_QUAT_X_VALUE_INDEX: usize = 50; /* INT16 */
pub const AHRSPOS_UPDATE_QUAT_Y_VALUE_INDEX: usize = 52; /* INT16 */
pub const AHRSPOS_UPDATE_QUAT_Z_VALUE_INDEX: usize = 54; /* INT16 */
pub const AHRSPOS_UPDATE_MPU_TEMP_VAUE_INDEX: usize = 56; /* Centigrade.  Signed Hundredths */
pub const AHRSPOS_UPDATE_OPSTATUS_VALUE_INDEX: usize = 58; /* NAVX_OP_STATUS_XXX */
pub const AHRSPOS_UPDATE_SENSOR_STATUS_VALUE_INDEX: usize = 59; /* NAVX_SENSOR_STATUS_XXX */
pub const AHRSPOS_UPDATE_CAL_STATUS_VALUE_INDEX: usize = 60; /* NAVX_CAL_STATUS_XXX */
pub const AHRSPOS_UPDATE_SELFTEST_STATUS_VALUE_INDEX: usize = 61; /* NAVX_SELFTEST_STATUS_XXX */
pub const AHRSPOS_UPDATE_MESSAGE_CHECKSUM_INDEX: usize = 62;
pub const AHRSPOS_UPDATE_MESSAGE_TERMINATOR_INDEX: usize = 64;
pub const AHRSPOS_UPDATE_MESSAGE_LENGTH: usize = 66;

// AHRSAndPositioningWithTimestamp Update Packet (similar to AHRSPos, but adds sample timestamp)

// #define MSGID_AHRSPOS_TS_UPDATE 't'
char_u8_const!(MSGID_AHRSPOS_TS_UPDATE, 't', 116, matu);
pub const AHRSPOS_TS_UPDATE_YAW_VALUE_INDEX: usize = 4; /* Signed 16:16.  Signed Hundredths */
pub const AHRSPOS_TS_UPDATE_ROLL_VALUE_INDEX: usize = 8; /* Signed 16:16.  Signed Hundredths */
pub const AHRSPOS_TS_UPDATE_PITCH_VALUE_INDEX: usize = 12; /* Signed 16:16.  Signed Hundredeths */
pub const AHRSPOS_TS_UPDATE_HEADING_VALUE_INDEX: usize = 16; /* Signed 16:16.  Unsigned Hundredths */
pub const AHRSPOS_TS_UPDATE_ALTITUDE_VALUE_INDEX: usize = 20; /* Meters.   Signed 16:16 */
pub const AHRSPOS_TS_UPDATE_FUSED_HEADING_VALUE_INDEX: usize = 24; /* Degrees.  Unsigned Hundredths */
pub const AHRSPOS_TS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX: usize = 28; /* Inst. G.  Signed 16:16 */
pub const AHRSPOS_TS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX: usize = 32; /* Inst. G.  Signed 16:16 */
pub const AHRSPOS_TS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX: usize = 36; /* Inst. G.  Signed 16:16 */
pub const AHRSPOS_TS_UPDATE_VEL_X_VALUE_INDEX: usize = 40; /* Signed 16:16, in meters/sec */
pub const AHRSPOS_TS_UPDATE_VEL_Y_VALUE_INDEX: usize = 44; /* Signed 16:16, in meters/sec */
pub const AHRSPOS_TS_UPDATE_VEL_Z_VALUE_INDEX: usize = 48; /* Signed 16:16, in meters/sec */
pub const AHRSPOS_TS_UPDATE_DISP_X_VALUE_INDEX: usize = 52; /* Signed 16:16, in meters */
pub const AHRSPOS_TS_UPDATE_DISP_Y_VALUE_INDEX: usize = 56; /* Signed 16:16, in meters */
pub const AHRSPOS_TS_UPDATE_DISP_Z_VALUE_INDEX: usize = 60; /* Signed 16:16, in meters */
pub const AHRSPOS_TS_UPDATE_QUAT_W_VALUE_INDEX: usize = 64; /* Signed 16:16 */
pub const AHRSPOS_TS_UPDATE_QUAT_X_VALUE_INDEX: usize = 68; /* Signed 16:16 */
pub const AHRSPOS_TS_UPDATE_QUAT_Y_VALUE_INDEX: usize = 72; /* Signed 16:16 */
pub const AHRSPOS_TS_UPDATE_QUAT_Z_VALUE_INDEX: usize = 76; /* Signed 16:16 */
pub const AHRSPOS_TS_UPDATE_MPU_TEMP_VAUE_INDEX: usize = 80; /* Centigrade.  Signed Hundredths */
pub const AHRSPOS_TS_UPDATE_OPSTATUS_VALUE_INDEX: usize = 82; /* NAVX_OP_STATUS_XXX */
pub const AHRSPOS_TS_UPDATE_SENSOR_STATUS_VALUE_INDEX: usize = 83; /* NAVX_SENSOR_STATUS_XXX */
pub const AHRSPOS_TS_UPDATE_CAL_STATUS_VALUE_INDEX: usize = 84; /* NAVX_CAL_STATUS_XXX */
pub const AHRSPOS_TS_UPDATE_SELFTEST_STATUS_VALUE_INDEX: usize = 85; /* NAVX_SELFTEST_STATUS_XXX */
pub const AHRSPOS_TS_UPDATE_TIMESTAMP_INDEX: usize = 86; /* UINT32 Timestamp, in milliseconds */
pub const AHRSPOS_TS_UPDATE_MESSAGE_CHECKSUM_INDEX: usize = 90;
pub const AHRSPOS_TS_UPDATE_MESSAGE_TERMINATOR_INDEX: usize = 92;
pub const AHRSPOS_TS_UPDATE_MESSAGE_LENGTH: usize = 94;

// Data Get Request:  Tuning Variable, Mag Cal, Board Identity (Response message depends upon request type)
// #define MSGID_DATA_REQUEST 'D'
char_u8_const!(MSGID_DATA_REQUEST, 'D', 68, mdr);
pub const DATA_REQUEST_DATATYPE_VALUE_INDEX: usize = 4;
pub const DATA_REQUEST_VARIABLEID_VALUE_INDEX: usize = 5;
pub const DATA_REQUEST_CHECKSUM_INDEX: usize = 6;
pub const DATA_REQUEST_TERMINATOR_INDEX: usize = 8;
pub const DATA_REQUEST_MESSAGE_LENGTH: usize = 10;

/* Data Set Response Packet (in response to MagCal SET and Tuning SET commands. */
// #define MSGID_DATA_SET_RESPONSE 'v'
char_u8_const!(MSGID_DATA_SET_RESPONSE, 'v', 118, mdsr);
pub const DATA_SET_RESPONSE_DATATYPE_VALUE_INDEX: usize = 4;
pub const DATA_SET_RESPONSE_VARID_VALUE_INDEX: usize = 5;
pub const DATA_SET_RESPONSE_STATUS_VALUE_INDEX: usize = 6;
pub const DATA_SET_RESPONSE_MESSAGE_CHECKSUM_INDEX: usize = 7;
pub const DATA_SET_RESPONSE_MESSAGE_TERMINATOR_INDEX: usize = 9;
pub const DATA_SET_RESPONSE_MESSAGE_LENGTH: usize = 11;

/* Integration Control Command Packet */
// #define MSGID_INTEGRATION_CONTROL_CMD 'I'
char_u8_const!(MSGID_INTEGRATION_CONTROL_CMD, 'I', 73, micc);
pub const INTEGRATION_CONTROL_CMD_ACTION_INDEX: usize = 4;
pub const INTEGRATION_CONTROL_CMD_PARAMETER_INDEX: usize = 5;
pub const INTEGRATION_CONTROL_CMD_MESSAGE_CHECKSUM_INDEX: usize = 9;
pub const INTEGRATION_CONTROL_CMD_MESSAGE_TERMINATOR_INDEX: usize = 11;
pub const INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH: usize = 13;

/* Integration Control Response Packet */
// #define MSGID_INTEGRATION_CONTROL_RESP 'j'
char_u8_const!(MSGID_INTEGRATION_CONTROL_RESP, 'j', 106, micr);
pub const INTEGRATION_CONTROL_RESP_ACTION_INDEX: usize = 4;
pub const INTEGRATION_CONTROL_RESP_PARAMETER_INDEX: usize = 5;
pub const INTEGRATION_CONTROL_RESP_MESSAGE_CHECKSUM_INDEX: usize = 9;
pub const INTEGRATION_CONTROL_RESP_MESSAGE_TERMINATOR_INDEX: usize = 11;
pub const INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH: usize = 13;

/* Magnetometer Calibration Packet
 ** This message may be used to SET (store) a new calibration into the navX board, or may be used
 ** to retrieve the current calibration data from the navX board. */
// #define MSGID_MAG_CAL_CMD 'M'
char_u8_const!(MSGID_MAG_CAL_CMD, 'M', 77, mmcc);
pub const MAG_CAL_DATA_ACTION_VALUE_INDEX: usize = 4;
pub const MAG_X_BIAS_VALUE_INDEX: usize = 5; /* signed short */
pub const MAG_Y_BIAS_VALUE_INDEX: usize = 7;
pub const MAG_Z_BIAS_VALUE_INDEX: usize = 9;
pub const MAG_XFORM_1_1_VALUE_INDEX: usize = 11; /* signed 16:16 */
pub const MAG_XFORM_1_2_VALUE_INDEX: usize = 15;
pub const MAG_XFORM_1_3_VALUE_INDEX: usize = 19;
pub const MAG_XFORM_2_1_VALUE_INDEX: usize = 23;
pub const MAG_XFORM_2_2_VALUE_INDEX: usize = 27;
pub const MAG_XFORM_2_3_VALUE_INDEX: usize = 31;
pub const MAG_XFORM_3_1_VALUE_INDEX: usize = 35;
pub const MAG_XFORM_3_2_VALUE_INDEX: usize = 39;
pub const MAG_XFORM_3_3_VALUE_INDEX: usize = 43;
pub const MAG_CAL_EARTH_MAG_FIELD_NORM_VALUE_INDEX: usize = 47;
pub const MAG_CAL_CMD_MESSAGE_CHECKSUM_INDEX: usize = 51;
pub const MAG_CAL_CMD_MESSAGE_TERMINATOR_INDEX: usize = 53;
pub const MAG_CAL_CMD_MESSAGE_LENGTH: usize = 55;

/* Tuning Variable Packet
 ** This message may be used to SET (modify) a tuning variable into the navX board,
 ** or to retrieve a current tuning variable from the navX board. */
// #define MSGID_FUSION_TUNING_CMD 'T'
char_u8_const!(MSGID_FUSION_TUNING_CMD, 'T', 84, mftc);
pub const FUSION_TUNING_DATA_ACTION_VALUE_INDEX: usize = 4;
pub const FUSION_TUNING_CMD_VAR_ID_VALUE_INDEX: usize = 5;
pub const FUSION_TUNING_CMD_VAR_VALUE_INDEX: usize = 6;
pub const FUSION_TUNING_CMD_MESSAGE_CHECKSUM_INDEX: usize = 10;
pub const FUSION_TUNING_CMD_MESSAGE_TERMINATOR_INDEX: usize = 12;
pub const FUSION_TUNING_CMD_MESSAGE_LENGTH: usize = 14;

// Board Identity Response Packet
// Sent in response to a Data Get (Board ID) message
// #define MSGID_BOARD_IDENTITY_RESPONSE 'i'
char_u8_const!(MSGID_BOARD_IDENTITY_RESPONSE, 'i', 105, mbir);
pub const BOARD_IDENTITY_BOARDTYPE_VALUE_INDEX: usize = 4;
pub const BOARD_IDENTITY_HWREV_VALUE_INDEX: usize = 5;
pub const BOARD_IDENTITY_FW_VER_MAJOR: usize = 6;
pub const BOARD_IDENTITY_FW_VER_MINOR: usize = 7;
pub const BOARD_IDENTITY_FW_VER_REVISION_VALUE_INDEX: usize = 8;
pub const BOARD_IDENTITY_UNIQUE_ID_0: usize = 10;
pub const BOARD_IDENTITY_UNIQUE_ID_1: usize = 11;
pub const BOARD_IDENTITY_UNIQUE_ID_2: usize = 12;
pub const BOARD_IDENTITY_UNIQUE_ID_3: usize = 13;
pub const BOARD_IDENTITY_UNIQUE_ID_4: usize = 14;
pub const BOARD_IDENTITY_UNIQUE_ID_5: usize = 15;
pub const BOARD_IDENTITY_UNIQUE_ID_6: usize = 16;
pub const BOARD_IDENTITY_UNIQUE_ID_7: usize = 17;
pub const BOARD_IDENTITY_UNIQUE_ID_8: usize = 18;
pub const BOARD_IDENTITY_UNIQUE_ID_9: usize = 19;
pub const BOARD_IDENTITY_UNIQUE_ID_10: usize = 20;
pub const BOARD_IDENTITY_UNIQUE_ID_11: usize = 21;
pub const BOARD_IDENTITY_RESPONSE_CHECKSUM_INDEX: usize = 22;
pub const BOARD_IDENTITY_RESPONSE_TERMINATOR_INDEX: usize = 24;
pub const BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH: usize = 26;

pub const AHRS_PROTOCOL_MAX_MESSAGE_LENGTH: usize = AHRS_UPDATE_MESSAGE_LENGTH;

#[derive(Debug, Clone, Default)]
pub struct AHRSUpdateBase {
    pub yaw: f32,
    pub pitch: f32,
    pub roll: f32,
    pub compass_heading: f32,
    pub altitude: f32,
    pub fused_heading: f32,
    pub linear_accel_x: f32,
    pub linear_accel_y: f32,
    pub linear_accel_z: f32,
    pub mpu_temp: f32,
    pub quat_w: f32,
    pub quat_x: f32,
    pub quat_y: f32,
    pub quat_z: f32,
    pub barometric_pressure: f32,
    pub baro_temp: f32,
    pub op_status: u8,
    pub sensor_status: u8,
    pub cal_status: u8,
    pub selftest_status: u8,
}
#[derive(Debug, Default, Clone)]
pub struct AHRSUpdate {
    pub base: AHRSUpdateBase,
    pub cal_mag_x: i16,
    pub cal_mag_y: i16,
    pub cal_mag_z: i16,
    pub mag_field_norm_ratio: f32,
    pub mag_field_norm_scalar: f32,
    pub raw_mag_x: i16,
    pub raw_mag_y: i16,
    pub raw_mag_z: i16,
}

#[derive(Debug, Default, Clone)]
pub struct AHRSPosUpdate {
    pub base: AHRSUpdateBase,
    pub vel_x: f32,
    pub vel_y: f32,
    pub vel_z: f32,
    pub disp_x: f32,
    pub disp_y: f32,
    pub disp_z: f32,
}
#[derive(Debug, Default, Clone)]
pub struct AHRSPosTSUpdate {
    pos_upd: AHRSPosUpdate,
    timestamp: u32,
}
#[derive(Debug, Default, Clone)]
pub struct BoardID {
    pub type_: u8,
    pub hw_rev: u8,
    pub fw_ver_major: u8,
    pub fw_ver_minor: u8,
    pub fw_revision: i16,
    pub unique_id: [u8; 12],
}
#[derive(Debug, Default, Clone)]
pub struct IntegrationControl {
    action: u8,
    parameter: i32,
}

// public:

//     static int encodeIntegrationControlCmd( char *protocol_buffer, pub struct IntegrationControl& cmd )
//     {
//         // Header
//         protocol_buffer[0] = PACKET_START_CHAR;
//         protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
//         protocol_buffer[2] = INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH - 2;
//         protocol_buffer[3] = MSGID_INTEGRATION_CONTROL_CMD;
//         // Data
//         protocol_buffer[INTEGRATION_CONTROL_CMD_ACTION_INDEX] = cmd.action;
//         IMURegisters::encodeProtocolInt32(cmd.parameter,&protocol_buffer[INTEGRATION_CONTROL_CMD_PARAMETER_INDEX]);
//         // Footer
//         encodeTermination( protocol_buffer, INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH, INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH - 4 );
//         return INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH;
//     }

//     static int decodeIntegrationControlCmd( char *buffer, int length, uint8_t& action, int32_t& parameter)
//     {
//         if ( length < INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH ) return 0;
//         if ( ( buffer[0] == PACKET_START_CHAR ) &&
//                 ( buffer[1] == BINARY_PACKET_INDICATOR_CHAR ) &&
//                 ( buffer[2] == INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH - 2) &&
//                 ( buffer[3] == MSGID_INTEGRATION_CONTROL_CMD ) )
//         {
//             if ( !verifyChecksum( buffer, INTEGRATION_CONTROL_CMD_MESSAGE_CHECKSUM_INDEX ) ) return 0;

//             // Data
//             action = (uint8_t)buffer[INTEGRATION_CONTROL_CMD_ACTION_INDEX];
//             parameter = IMURegisters::decodeProtocolInt32(&buffer[INTEGRATION_CONTROL_CMD_PARAMETER_INDEX]);
//             return INTEGRATION_CONTROL_CMD_MESSAGE_LENGTH;
//         }
//         return 0;
//     }

//     static int encodeIntegrationControlResponse( char *protocol_buffer, uint8_t action, int32_t parameter )
//     {
//         // Header
//         protocol_buffer[0] = PACKET_START_CHAR;
//         protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
//         protocol_buffer[2] = INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH - 2;
//         protocol_buffer[3] = MSGID_INTEGRATION_CONTROL_RESP;
//         // Data
//         protocol_buffer[INTEGRATION_CONTROL_RESP_ACTION_INDEX] = action;
//         IMURegisters::encodeProtocolInt32(parameter,&protocol_buffer[INTEGRATION_CONTROL_RESP_PARAMETER_INDEX]);
//         // Footer
//         encodeTermination( protocol_buffer, INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH, INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH - 4 );
//         return INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH;
//     }

//     static int decodeIntegrationControlResponse( char *buffer, int length, pub struct IntegrationControl& rsp)
//     {
//         if ( length < INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH ) return 0;
//         if ( ( buffer[0] == PACKET_START_CHAR ) &&
//                 ( buffer[1] == BINARY_PACKET_INDICATOR_CHAR ) &&
//                 ( buffer[2] == INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH - 2) &&
//                 ( buffer[3] == MSGID_INTEGRATION_CONTROL_RESP ) )
//         {
//             if ( !verifyChecksum( buffer, INTEGRATION_CONTROL_RESP_MESSAGE_CHECKSUM_INDEX ) ) return 0;

//             // Data
//             rsp.action = (uint8_t)buffer[INTEGRATION_CONTROL_RESP_ACTION_INDEX];
//             rsp.parameter = IMURegisters::decodeProtocolInt32(&buffer[INTEGRATION_CONTROL_RESP_PARAMETER_INDEX]);
//             return INTEGRATION_CONTROL_RESP_MESSAGE_LENGTH;
//         }
//         return 0;
//     }

//     static int encodeTuningVariableCmd( char *protocol_buffer, AHRS_DATA_ACTION getset, AHRS_TUNING_VAR_ID id, float val )
//     {
//         // Header
//         protocol_buffer[0] = PACKET_START_CHAR;
//         protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
//         protocol_buffer[2] = FUSION_TUNING_CMD_MESSAGE_LENGTH - 2;
//         protocol_buffer[3] = MSGID_FUSION_TUNING_CMD;
//         // Data
//         protocol_buffer[FUSION_TUNING_DATA_ACTION_VALUE_INDEX] = getset;
//         protocol_buffer[FUSION_TUNING_CMD_VAR_ID_VALUE_INDEX] = id;
//         IMURegisters::encodeProtocol1616Float(val,&protocol_buffer[FUSION_TUNING_CMD_VAR_VALUE_INDEX]);
//         // Footer
//         encodeTermination( protocol_buffer, FUSION_TUNING_CMD_MESSAGE_LENGTH, FUSION_TUNING_CMD_MESSAGE_LENGTH - 4 );
//         return FUSION_TUNING_CMD_MESSAGE_LENGTH;
//     }

//     static int decodeTuningVariableCmd( char *buffer, int length, AHRS_DATA_ACTION& getset, AHRS_TUNING_VAR_ID& id, float& val )
//     {
//         if ( length < FUSION_TUNING_CMD_MESSAGE_LENGTH ) return 0;
//         if ( ( buffer[0] == PACKET_START_CHAR ) &&
//                 ( buffer[1] == BINARY_PACKET_INDICATOR_CHAR ) &&
//                 ( buffer[2] == FUSION_TUNING_CMD_MESSAGE_LENGTH - 2) &&
//                 ( buffer[3] == MSGID_FUSION_TUNING_CMD ) )
//         {
//             if ( !verifyChecksum( buffer, FUSION_TUNING_CMD_MESSAGE_CHECKSUM_INDEX ) ) return 0;

//             // Data
//             getset = (AHRS_DATA_ACTION)buffer[FUSION_TUNING_DATA_ACTION_VALUE_INDEX];
//             id = (AHRS_TUNING_VAR_ID)buffer[FUSION_TUNING_CMD_VAR_ID_VALUE_INDEX];
//             val = IMURegisters::decodeProtocol1616Float(&buffer[FUSION_TUNING_CMD_VAR_VALUE_INDEX]);
//             return FUSION_TUNING_CMD_MESSAGE_LENGTH;
//         }
//         return 0;
//     }

//     static int encodeAHRSUpdate( char *protocol_buffer,
//             float yaw, float pitch, float roll,
//             float compass_heading, float altitude, float fused_heading,
//             float linear_accel_x, float linear_accel_y, float linear_accel_z,
//             float mpu_temp_c,
//             int16_t raw_mag_x, int16_t raw_mag_y, int16_t raw_mag_z,
//             int16_t cal_mag_x, int16_t cal_mag_y, int16_t cal_mag_z,
//             float mag_norm_ratio, float mag_norm_scalar,
//             int16_t quat_w, int16_t quat_x, int16_t quat_y, int16_t quat_z,
//             float baro_pressure, float baro_temp_c,
//             uint8_t op_status, uint8_t sensor_status,
//             uint8_t cal_status, uint8_t selftest_status )
//     {
//         // Header
//         protocol_buffer[0] = PACKET_START_CHAR;
//         protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
//         protocol_buffer[2] = AHRS_UPDATE_MESSAGE_LENGTH - 2;
//         protocol_buffer[3] = MSGID_AHRS_UPDATE;
//         // data
//         IMURegisters::encodeProtocolSignedHundredthsFloat(yaw, &protocol_buffer[AHRS_UPDATE_YAW_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedHundredthsFloat(pitch, &protocol_buffer[AHRS_UPDATE_PITCH_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedHundredthsFloat(roll, &protocol_buffer[AHRS_UPDATE_ROLL_VALUE_INDEX]);
//         IMURegisters::encodeProtocolUnsignedHundredthsFloat(compass_heading, &protocol_buffer[AHRS_UPDATE_HEADING_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(altitude,&protocol_buffer[AHRS_UPDATE_ALTITUDE_VALUE_INDEX]);
//         IMURegisters::encodeProtocolUnsignedHundredthsFloat(fused_heading, &protocol_buffer[AHRS_UPDATE_FUSED_HEADING_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedThousandthsFloat(linear_accel_x,&protocol_buffer[AHRS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedThousandthsFloat(linear_accel_y,&protocol_buffer[AHRS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedThousandthsFloat(linear_accel_z,&protocol_buffer[AHRS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(cal_mag_x, &protocol_buffer[AHRS_UPDATE_CAL_MAG_X_VALUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(cal_mag_y, &protocol_buffer[AHRS_UPDATE_CAL_MAG_Y_VALUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(cal_mag_z, &protocol_buffer[AHRS_UPDATE_CAL_MAG_Z_VALUE_INDEX]);
//         IMURegisters::encodeProtocolUnsignedHundredthsFloat(mag_norm_ratio, &protocol_buffer[AHRS_UPDATE_CAL_MAG_NORM_RATIO_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(mag_norm_scalar, &protocol_buffer[AHRS_UPDATE_CAL_MAG_SCALAR_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedHundredthsFloat(mpu_temp_c, &protocol_buffer[AHRS_UPDATE_MPU_TEMP_VAUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(raw_mag_x, &protocol_buffer[AHRS_UPDATE_RAW_MAG_X_VALUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(raw_mag_y, &protocol_buffer[AHRS_UPDATE_RAW_MAG_Y_VALUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(raw_mag_z, &protocol_buffer[AHRS_UPDATE_RAW_MAG_Z_VALUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(quat_w, &protocol_buffer[AHRS_UPDATE_QUAT_W_VALUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(quat_x, &protocol_buffer[AHRS_UPDATE_QUAT_X_VALUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(quat_y, &protocol_buffer[AHRS_UPDATE_QUAT_Y_VALUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(quat_z, &protocol_buffer[AHRS_UPDATE_QUAT_Z_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(baro_pressure, &protocol_buffer[AHRS_UPDATE_BARO_PRESSURE_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedHundredthsFloat(baro_temp_c, &protocol_buffer[AHRS_UPDATE_BARO_TEMP_VAUE_INDEX]);

//         protocol_buffer[AHRS_UPDATE_OPSTATUS_VALUE_INDEX] = op_status;
//         protocol_buffer[AHRS_UPDATE_SENSOR_STATUS_VALUE_INDEX] = sensor_status;
//         protocol_buffer[AHRS_UPDATE_CAL_STATUS_VALUE_INDEX] = cal_status;
//         protocol_buffer[AHRS_UPDATE_SELFTEST_STATUS_VALUE_INDEX] = selftest_status;
//         // Footer
//         encodeTermination( protocol_buffer, AHRS_UPDATE_MESSAGE_LENGTH, AHRS_UPDATE_MESSAGE_LENGTH - 4 );
//         return AHRS_UPDATE_MESSAGE_LENGTH;
//     }

//     static int decodeAHRSUpdate( char *buffer, int length, pub struct AHRSUpdate& update)
//     {
//         if ( length < AHRS_UPDATE_MESSAGE_LENGTH ) return 0;
//         if ( ( buffer[0] == PACKET_START_CHAR ) &&
//                 ( buffer[1] == BINARY_PACKET_INDICATOR_CHAR ) &&
//                 ( buffer[2] == AHRS_UPDATE_MESSAGE_LENGTH - 2) &&
//                 ( buffer[3] == MSGID_AHRS_UPDATE ) ) {

//             if ( !verifyChecksum( buffer, AHRS_UPDATE_MESSAGE_CHECKSUM_INDEX ) ) return 0;

//             update.yaw = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[AHRS_UPDATE_YAW_VALUE_INDEX]);
//             update.pitch = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[AHRS_UPDATE_PITCH_VALUE_INDEX]);
//             update.roll = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[AHRS_UPDATE_ROLL_VALUE_INDEX]);
//             update.compass_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(&buffer[AHRS_UPDATE_HEADING_VALUE_INDEX]);
//             update.altitude = IMURegisters::decodeProtocol1616Float(&buffer[AHRS_UPDATE_ALTITUDE_VALUE_INDEX]);
//             update.fused_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(&buffer[AHRS_UPDATE_FUSED_HEADING_VALUE_INDEX]);
//             update.linear_accel_x = IMURegisters::decodeProtocolSignedThousandthsFloat(&buffer[AHRS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX]);
//             update.linear_accel_y = IMURegisters::decodeProtocolSignedThousandthsFloat(&buffer[AHRS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX]);
//             update.linear_accel_z = IMURegisters::decodeProtocolSignedThousandthsFloat(&buffer[AHRS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX]);
//             update.cal_mag_x = IMURegisters::decodeProtocolInt16(&buffer[AHRS_UPDATE_CAL_MAG_X_VALUE_INDEX]);
//             update.cal_mag_y = IMURegisters::decodeProtocolInt16(&buffer[AHRS_UPDATE_CAL_MAG_Y_VALUE_INDEX]);
//             update.cal_mag_z = IMURegisters::decodeProtocolInt16(&buffer[AHRS_UPDATE_CAL_MAG_Z_VALUE_INDEX]);
//             update.mag_field_norm_ratio = IMURegisters::decodeProtocolUnsignedHundredthsFloat(&buffer[AHRS_UPDATE_CAL_MAG_NORM_RATIO_VALUE_INDEX]);
//             update.mag_field_norm_scalar = IMURegisters::decodeProtocol1616Float(&buffer[AHRS_UPDATE_CAL_MAG_SCALAR_VALUE_INDEX]);
//             update.mpu_temp = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[AHRS_UPDATE_MPU_TEMP_VAUE_INDEX]);
//             update.raw_mag_x = IMURegisters::decodeProtocolInt16(&buffer[AHRS_UPDATE_RAW_MAG_X_VALUE_INDEX]);
//             update.raw_mag_y = IMURegisters::decodeProtocolInt16(&buffer[AHRS_UPDATE_RAW_MAG_Y_VALUE_INDEX]);
//             update.raw_mag_z = IMURegisters::decodeProtocolInt16(&buffer[AHRS_UPDATE_RAW_MAG_Z_VALUE_INDEX]);
// 			/* AHRSPosUpdate:  Quaternions are signed int (16-bit resolution); divide by 16384 to yield +/- 2 radians */
//             update.quat_w = ((float)IMURegisters::decodeProtocolInt16(&buffer[AHRS_UPDATE_QUAT_W_VALUE_INDEX]) / 16384.0f);
//             update.quat_x = ((float)IMURegisters::decodeProtocolInt16(&buffer[AHRS_UPDATE_QUAT_X_VALUE_INDEX]) / 16384.0f);
//             update.quat_y = ((float)IMURegisters::decodeProtocolInt16(&buffer[AHRS_UPDATE_QUAT_Y_VALUE_INDEX]) / 16384.0f);
//             update.quat_z = ((float)IMURegisters::decodeProtocolInt16(&buffer[AHRS_UPDATE_QUAT_Z_VALUE_INDEX]) / 16384.0f);
//             update.barometric_pressure = IMURegisters::decodeProtocol1616Float(&buffer[AHRS_UPDATE_BARO_PRESSURE_VALUE_INDEX]);
//             update.baro_temp = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[AHRS_UPDATE_BARO_TEMP_VAUE_INDEX]);
//             update.op_status = buffer[AHRS_UPDATE_OPSTATUS_VALUE_INDEX];
//             update.sensor_status = buffer[AHRS_UPDATE_SENSOR_STATUS_VALUE_INDEX];
//             update.cal_status = buffer[AHRS_UPDATE_CAL_STATUS_VALUE_INDEX];
//             update.selftest_status = buffer[AHRS_UPDATE_SELFTEST_STATUS_VALUE_INDEX];

//             return AHRS_UPDATE_MESSAGE_LENGTH;
//         }
//         return 0;
//     }

//     static int encodeAHRSPosUpdate( char *protocol_buffer,
//             float yaw, float pitch, float roll,
//             float compass_heading, float altitude, float fused_heading,
//             float linear_accel_x, float linear_accel_y, float linear_accel_z,
//             float mpu_temp_c,
//             int16_t quat_w, int16_t quat_x, int16_t quat_y, int16_t quat_z,
//             float vel_x, float vel_y, float vel_z,
//             float disp_x, float disp_y, float disp_z,
//             uint8_t op_status, uint8_t sensor_status,
//             uint8_t cal_status, uint8_t selftest_status )
//     {
//         // Header
//         protocol_buffer[0] = PACKET_START_CHAR;
//         protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
//         protocol_buffer[2] = AHRSPOS_UPDATE_MESSAGE_LENGTH - 2;
//         protocol_buffer[3] = MSGID_AHRSPOS_UPDATE;
//         // data
//         IMURegisters::encodeProtocolSignedHundredthsFloat(yaw, &protocol_buffer[AHRSPOS_UPDATE_YAW_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedHundredthsFloat(pitch, &protocol_buffer[AHRSPOS_UPDATE_PITCH_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedHundredthsFloat(roll, &protocol_buffer[AHRSPOS_UPDATE_ROLL_VALUE_INDEX]);
//         IMURegisters::encodeProtocolUnsignedHundredthsFloat(compass_heading, &protocol_buffer[AHRSPOS_UPDATE_HEADING_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(altitude,&protocol_buffer[AHRSPOS_UPDATE_ALTITUDE_VALUE_INDEX]);
//         IMURegisters::encodeProtocolUnsignedHundredthsFloat(fused_heading, &protocol_buffer[AHRSPOS_UPDATE_FUSED_HEADING_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedThousandthsFloat(linear_accel_x,&protocol_buffer[AHRSPOS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedThousandthsFloat(linear_accel_y,&protocol_buffer[AHRSPOS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedThousandthsFloat(linear_accel_z,&protocol_buffer[AHRSPOS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(vel_x,&protocol_buffer[AHRSPOS_UPDATE_VEL_X_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(vel_y,&protocol_buffer[AHRSPOS_UPDATE_VEL_Y_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(vel_z,&protocol_buffer[AHRSPOS_UPDATE_VEL_Z_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(disp_x,&protocol_buffer[AHRSPOS_UPDATE_DISP_X_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(disp_y,&protocol_buffer[AHRSPOS_UPDATE_DISP_Y_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(disp_z,&protocol_buffer[AHRSPOS_UPDATE_DISP_Z_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedHundredthsFloat(mpu_temp_c, &protocol_buffer[AHRSPOS_UPDATE_MPU_TEMP_VAUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(quat_w, &protocol_buffer[AHRSPOS_UPDATE_QUAT_W_VALUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(quat_x, &protocol_buffer[AHRSPOS_UPDATE_QUAT_X_VALUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(quat_y, &protocol_buffer[AHRSPOS_UPDATE_QUAT_Y_VALUE_INDEX]);
//         IMURegisters::encodeProtocolInt16(quat_z, &protocol_buffer[AHRSPOS_UPDATE_QUAT_Z_VALUE_INDEX]);

//         protocol_buffer[AHRSPOS_UPDATE_OPSTATUS_VALUE_INDEX] = op_status;
//         protocol_buffer[AHRSPOS_UPDATE_SENSOR_STATUS_VALUE_INDEX] = sensor_status;
//         protocol_buffer[AHRSPOS_UPDATE_CAL_STATUS_VALUE_INDEX] = cal_status;
//         protocol_buffer[AHRSPOS_UPDATE_SELFTEST_STATUS_VALUE_INDEX] = selftest_status;
//         // Footer
//         encodeTermination( protocol_buffer, AHRSPOS_UPDATE_MESSAGE_LENGTH, AHRSPOS_UPDATE_MESSAGE_LENGTH - 4 );
//         return AHRSPOS_UPDATE_MESSAGE_LENGTH;
//     }

//     static int decodeAHRSPosUpdate( char *buffer, int length, pub struct AHRSPosUpdate& update)
//     {
//         if ( length < AHRSPOS_UPDATE_MESSAGE_LENGTH ) return 0;
//         if ( ( buffer[0] == PACKET_START_CHAR ) &&
//                 ( buffer[1] == BINARY_PACKET_INDICATOR_CHAR ) &&
//                 ( buffer[2] == AHRSPOS_UPDATE_MESSAGE_LENGTH - 2) &&
//                 ( buffer[3] == MSGID_AHRSPOS_UPDATE ) ) {

//             if ( !verifyChecksum( buffer, AHRSPOS_UPDATE_MESSAGE_CHECKSUM_INDEX ) ) return 0;

//             update.yaw = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[AHRSPOS_UPDATE_YAW_VALUE_INDEX]);
//             update.pitch = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[AHRSPOS_UPDATE_PITCH_VALUE_INDEX]);
//             update.roll = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[AHRSPOS_UPDATE_ROLL_VALUE_INDEX]);
//             update.compass_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(&buffer[AHRSPOS_UPDATE_HEADING_VALUE_INDEX]);
//             update.altitude = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_UPDATE_ALTITUDE_VALUE_INDEX]);
//             update.fused_heading = IMURegisters::decodeProtocolUnsignedHundredthsFloat(&buffer[AHRSPOS_UPDATE_FUSED_HEADING_VALUE_INDEX]);
//             update.linear_accel_x = IMURegisters::decodeProtocolSignedThousandthsFloat(&buffer[AHRSPOS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX]);
//             update.linear_accel_y = IMURegisters::decodeProtocolSignedThousandthsFloat(&buffer[AHRSPOS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX]);
//             update.linear_accel_z = IMURegisters::decodeProtocolSignedThousandthsFloat(&buffer[AHRSPOS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX]);
//             update.vel_x = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_UPDATE_VEL_X_VALUE_INDEX]);
//             update.vel_y = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_UPDATE_VEL_Y_VALUE_INDEX]);
//             update.vel_z = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_UPDATE_VEL_Z_VALUE_INDEX]);
//             update.disp_x = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_UPDATE_DISP_X_VALUE_INDEX]);
//             update.disp_y = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_UPDATE_DISP_Y_VALUE_INDEX]);
//             update.disp_z = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_UPDATE_DISP_Z_VALUE_INDEX]);
//             update.mpu_temp = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[AHRSPOS_UPDATE_MPU_TEMP_VAUE_INDEX]);
// 			/* AHRSPosUpdate:  Quaternions are signed int (16-bit resolution); divide by 16384 to yield +/- 2 radians */
//             update.quat_w = ((float)IMURegisters::decodeProtocolInt16(&buffer[AHRSPOS_UPDATE_QUAT_W_VALUE_INDEX]) / 16384.0f);
//             update.quat_x = ((float)IMURegisters::decodeProtocolInt16(&buffer[AHRSPOS_UPDATE_QUAT_X_VALUE_INDEX]) / 16384.0f);
//             update.quat_y = ((float)IMURegisters::decodeProtocolInt16(&buffer[AHRSPOS_UPDATE_QUAT_Y_VALUE_INDEX]) / 16384.0f);
//             update.quat_z = ((float)IMURegisters::decodeProtocolInt16(&buffer[AHRSPOS_UPDATE_QUAT_Z_VALUE_INDEX]) / 16384.0f);
//             update.op_status = buffer[AHRSPOS_UPDATE_OPSTATUS_VALUE_INDEX];
//             update.sensor_status = buffer[AHRSPOS_UPDATE_SENSOR_STATUS_VALUE_INDEX];
//             update.cal_status = buffer[AHRSPOS_UPDATE_CAL_STATUS_VALUE_INDEX];
//             update.selftest_status = buffer[AHRSPOS_UPDATE_SELFTEST_STATUS_VALUE_INDEX];

//             return AHRSPOS_UPDATE_MESSAGE_LENGTH;
//         }
//         return 0;
//     }

//     static int encodeAHRSPosTSUpdate( char *protocol_buffer,
//             float yaw, float pitch, float roll,
//             float compass_heading, float altitude, float fused_heading,
//             float linear_accel_x, float linear_accel_y, float linear_accel_z,
//             float mpu_temp_c,
//             float quat_w, float quat_x, float quat_y, float quat_z,
//             float vel_x, float vel_y, float vel_z,
//             float disp_x, float disp_y, float disp_z,
//             uint8_t op_status, uint8_t sensor_status,
//             uint8_t cal_status, uint8_t selftest_status,
//             uint32_t timestamp)
//     {
//         // Header
//         protocol_buffer[0] = PACKET_START_CHAR;
//         protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
//         protocol_buffer[2] = AHRSPOS_TS_UPDATE_MESSAGE_LENGTH - 2;
//         protocol_buffer[3] = MSGID_AHRSPOS_TS_UPDATE;

//         // data
//         IMURegisters::encodeProtocol1616Float(yaw, &protocol_buffer[AHRSPOS_TS_UPDATE_YAW_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(pitch, &protocol_buffer[AHRSPOS_TS_UPDATE_PITCH_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(roll, &protocol_buffer[AHRSPOS_TS_UPDATE_ROLL_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(compass_heading, &protocol_buffer[AHRSPOS_TS_UPDATE_HEADING_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(altitude,&protocol_buffer[AHRSPOS_TS_UPDATE_ALTITUDE_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(fused_heading, &protocol_buffer[AHRSPOS_TS_UPDATE_FUSED_HEADING_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(linear_accel_x,&protocol_buffer[AHRSPOS_TS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(linear_accel_y,&protocol_buffer[AHRSPOS_TS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(linear_accel_z,&protocol_buffer[AHRSPOS_TS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(vel_x,&protocol_buffer[AHRSPOS_TS_UPDATE_VEL_X_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(vel_y,&protocol_buffer[AHRSPOS_TS_UPDATE_VEL_Y_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(vel_z,&protocol_buffer[AHRSPOS_TS_UPDATE_VEL_Z_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(disp_x,&protocol_buffer[AHRSPOS_TS_UPDATE_DISP_X_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(disp_y,&protocol_buffer[AHRSPOS_TS_UPDATE_DISP_Y_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(disp_z,&protocol_buffer[AHRSPOS_TS_UPDATE_DISP_Z_VALUE_INDEX]);
//         IMURegisters::encodeProtocolSignedHundredthsFloat(mpu_temp_c, &protocol_buffer[AHRSPOS_TS_UPDATE_MPU_TEMP_VAUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(quat_w, &protocol_buffer[AHRSPOS_TS_UPDATE_QUAT_W_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(quat_x, &protocol_buffer[AHRSPOS_TS_UPDATE_QUAT_X_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(quat_y, &protocol_buffer[AHRSPOS_TS_UPDATE_QUAT_Y_VALUE_INDEX]);
//         IMURegisters::encodeProtocol1616Float(quat_z, &protocol_buffer[AHRSPOS_TS_UPDATE_QUAT_Z_VALUE_INDEX]);

//         protocol_buffer[AHRSPOS_TS_UPDATE_OPSTATUS_VALUE_INDEX] = op_status;
//         protocol_buffer[AHRSPOS_TS_UPDATE_SENSOR_STATUS_VALUE_INDEX] = sensor_status;
//         protocol_buffer[AHRSPOS_TS_UPDATE_CAL_STATUS_VALUE_INDEX] = cal_status;
//         protocol_buffer[AHRSPOS_TS_UPDATE_SELFTEST_STATUS_VALUE_INDEX] = selftest_status;
//         IMURegisters::encodeProtocolInt32((int32_t)timestamp, &protocol_buffer[AHRSPOS_TS_UPDATE_TIMESTAMP_INDEX]);

//         // Footer
//         encodeTermination( protocol_buffer, AHRSPOS_TS_UPDATE_MESSAGE_LENGTH, AHRSPOS_TS_UPDATE_MESSAGE_LENGTH - 4 );
//         return AHRSPOS_TS_UPDATE_MESSAGE_LENGTH;
//     }

//     static int decodeAHRSPosTSUpdate( char *buffer, int length, pub struct AHRSPosTSUpdate& update)
//     {
//         if ( length < AHRSPOS_TS_UPDATE_MESSAGE_LENGTH ) return 0;
//         if ( ( buffer[0] == PACKET_START_CHAR ) &&
//                 ( buffer[1] == BINARY_PACKET_INDICATOR_CHAR ) &&
//                 ( buffer[2] == AHRSPOS_TS_UPDATE_MESSAGE_LENGTH - 2) &&
//                 ( buffer[3] == MSGID_AHRSPOS_TS_UPDATE ) ) {

//             if ( !verifyChecksum( buffer, AHRSPOS_TS_UPDATE_MESSAGE_CHECKSUM_INDEX ) ) return 0;

//             update.yaw = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_YAW_VALUE_INDEX]);
//             update.pitch = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_PITCH_VALUE_INDEX]);
//             update.roll = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_ROLL_VALUE_INDEX]);
//             update.compass_heading = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_HEADING_VALUE_INDEX]);
//             update.altitude = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_ALTITUDE_VALUE_INDEX]);
//             update.fused_heading = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_FUSED_HEADING_VALUE_INDEX]);
//             update.linear_accel_x = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_LINEAR_ACCEL_X_VALUE_INDEX]);
//             update.linear_accel_y = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_LINEAR_ACCEL_Y_VALUE_INDEX]);
//             update.linear_accel_z = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_LINEAR_ACCEL_Z_VALUE_INDEX]);
//             update.vel_x = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_VEL_X_VALUE_INDEX]);
//             update.vel_y = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_VEL_Y_VALUE_INDEX]);
//             update.vel_z = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_VEL_Z_VALUE_INDEX]);
//             update.disp_x = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_DISP_X_VALUE_INDEX]);
//             update.disp_y = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_DISP_Y_VALUE_INDEX]);
//             update.disp_z = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_DISP_Z_VALUE_INDEX]);
//             update.mpu_temp = IMURegisters::decodeProtocolSignedHundredthsFloat(&buffer[AHRSPOS_TS_UPDATE_MPU_TEMP_VAUE_INDEX]);
//             update.quat_w = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_QUAT_W_VALUE_INDEX]) / 16384.0f;
//             update.quat_x = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_QUAT_X_VALUE_INDEX]) / 16384.0f;
//             update.quat_y = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_QUAT_Y_VALUE_INDEX]) / 16384.0f;
//             update.quat_z = IMURegisters::decodeProtocol1616Float(&buffer[AHRSPOS_TS_UPDATE_QUAT_Z_VALUE_INDEX]) / 16384.0f;
//             update.op_status = buffer[AHRSPOS_TS_UPDATE_OPSTATUS_VALUE_INDEX];
//             update.sensor_status = buffer[AHRSPOS_TS_UPDATE_SENSOR_STATUS_VALUE_INDEX];
//             update.cal_status = buffer[AHRSPOS_TS_UPDATE_CAL_STATUS_VALUE_INDEX];
//             update.selftest_status = buffer[AHRSPOS_TS_UPDATE_SELFTEST_STATUS_VALUE_INDEX];
//             update.timestamp = (uint32_t)IMURegisters::decodeProtocolInt32(&buffer[AHRSPOS_TS_UPDATE_TIMESTAMP_INDEX]);

//             return AHRSPOS_TS_UPDATE_MESSAGE_LENGTH;
//         }
//         return 0;
//     }

//     static int encodeMagCalCommand( char *protocol_buffer, AHRS_DATA_ACTION action, int16_t *bias, float *matrix, float earth_mag_field_norm )
//     {
//         // Header
//         protocol_buffer[0] = PACKET_START_CHAR;
//         protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
//         protocol_buffer[2] = MAG_CAL_CMD_MESSAGE_LENGTH - 2;
//         protocol_buffer[3] = MSGID_MAG_CAL_CMD;

//         // Data
//         protocol_buffer[MAG_CAL_DATA_ACTION_VALUE_INDEX] = action;
//         for ( int i = 0; i < 3; i++ ) {
//             IMURegisters::encodeProtocolInt16(	bias[i],
//                     &protocol_buffer[MAG_X_BIAS_VALUE_INDEX + (i * sizeof(int16_t))]);
//         }
//         for ( int i = 0; i < 9; i++ ) {
//             IMURegisters::encodeProtocol1616Float( matrix[i], &protocol_buffer[MAG_XFORM_1_1_VALUE_INDEX + (i * sizeof(s_1616_float))]);
//         }
//         IMURegisters::encodeProtocol1616Float( earth_mag_field_norm, &protocol_buffer[MAG_CAL_EARTH_MAG_FIELD_NORM_VALUE_INDEX]);
//         // Footer
//         encodeTermination( protocol_buffer, MAG_CAL_CMD_MESSAGE_LENGTH, MAG_CAL_CMD_MESSAGE_LENGTH - 4 );
//         return MAG_CAL_CMD_MESSAGE_LENGTH;
//     }

//     static int decodeMagCalCommand( char *buffer, int length,
//             AHRS_DATA_ACTION& action,
//             int16_t *bias,
//             float *matrix,
//             float& earth_mag_field_norm)
//     {
//         if ( length < MAG_CAL_CMD_MESSAGE_LENGTH ) return 0;
//         if ( ( buffer[0] == PACKET_START_CHAR ) &&
//                 ( buffer[1] == BINARY_PACKET_INDICATOR_CHAR ) &&
//                 ( buffer[2] == MAG_CAL_CMD_MESSAGE_LENGTH - 2) &&
//                 ( buffer[3] == MSGID_MAG_CAL_CMD ) ) {

//             if ( !verifyChecksum( buffer, MAG_CAL_CMD_MESSAGE_CHECKSUM_INDEX ) ) return 0;

//             action = (AHRS_DATA_ACTION)buffer[MAG_CAL_DATA_ACTION_VALUE_INDEX];
//             for ( int i = 0; i < 3; i++ ) {
//                 bias[i] = IMURegisters::decodeProtocolInt16(&buffer[MAG_X_BIAS_VALUE_INDEX + (i * sizeof(int16_t))]);
//             }
//             for ( int i = 0; i < 9; i++ ) {
//                 matrix[i] = IMURegisters::decodeProtocol1616Float(&buffer[MAG_XFORM_1_1_VALUE_INDEX + (i * sizeof(s_1616_float))]);
//             }
//             earth_mag_field_norm = IMURegisters::decodeProtocol1616Float(&buffer[MAG_CAL_EARTH_MAG_FIELD_NORM_VALUE_INDEX]);
//             return MAG_CAL_CMD_MESSAGE_LENGTH;
//         }
//         return 0;
//     }

//     static int encodeDataSetResponse( char *protocol_buffer, AHRS_DATA_TYPE type, AHRS_TUNING_VAR_ID subtype, uint8_t status )
//     {
//         // Header
//         protocol_buffer[0] = PACKET_START_CHAR;
//         protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
//         protocol_buffer[2] = DATA_SET_RESPONSE_MESSAGE_LENGTH - 2;
//         protocol_buffer[3] = MSGID_DATA_SET_RESPONSE;
//         // Data
//         protocol_buffer[DATA_SET_RESPONSE_DATATYPE_VALUE_INDEX] = type;
//         protocol_buffer[DATA_SET_RESPONSE_VARID_VALUE_INDEX] = subtype;
//         protocol_buffer[DATA_SET_RESPONSE_STATUS_VALUE_INDEX] = status;
//         // Footer
//         encodeTermination( protocol_buffer, DATA_SET_RESPONSE_MESSAGE_LENGTH, DATA_SET_RESPONSE_MESSAGE_LENGTH - 4 );
//         return DATA_SET_RESPONSE_MESSAGE_LENGTH;
//     }

//     static int decodeDataSetResponse( char *buffer, int length, AHRS_DATA_TYPE type, AHRS_TUNING_VAR_ID subtype, uint8_t& status )
//     {
//         if ( length < DATA_SET_RESPONSE_MESSAGE_LENGTH ) return 0;
//         if ( ( buffer[0] == PACKET_START_CHAR ) &&
//                 ( buffer[1] == BINARY_PACKET_INDICATOR_CHAR ) &&
//                 ( buffer[2] == DATA_SET_RESPONSE_MESSAGE_LENGTH - 2) &&
//                 ( buffer[3] == MSGID_DATA_SET_RESPONSE ) ) {

//             if ( !verifyChecksum( buffer, DATA_SET_RESPONSE_MESSAGE_CHECKSUM_INDEX ) ) return 0;

//             type = (AHRS_DATA_TYPE)buffer[DATA_SET_RESPONSE_DATATYPE_VALUE_INDEX];
//             subtype = (AHRS_TUNING_VAR_ID)buffer[DATA_SET_RESPONSE_VARID_VALUE_INDEX];
//             status = buffer[DATA_SET_RESPONSE_STATUS_VALUE_INDEX];
//             return DATA_SET_RESPONSE_MESSAGE_LENGTH;
//         }
//         return 0;
//     }

//     static int encodeDataGetRequest( char *protocol_buffer, AHRS_DATA_TYPE type, AHRS_TUNING_VAR_ID subtype )
//     {
//         // Header
//         protocol_buffer[0] = PACKET_START_CHAR;
//         protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
//         protocol_buffer[2] = DATA_REQUEST_MESSAGE_LENGTH - 2;
//         protocol_buffer[3] = MSGID_DATA_REQUEST;
//         // Data
//         protocol_buffer[DATA_REQUEST_DATATYPE_VALUE_INDEX] = type;
//         protocol_buffer[DATA_REQUEST_VARIABLEID_VALUE_INDEX] = subtype;
//         // Footer
//         encodeTermination( protocol_buffer, DATA_REQUEST_MESSAGE_LENGTH, DATA_REQUEST_MESSAGE_LENGTH - 4 );
//         return DATA_REQUEST_MESSAGE_LENGTH;
//     }

//     static int decodeDataGetRequest( char *buffer, int length, AHRS_DATA_TYPE& type, AHRS_TUNING_VAR_ID& subtype )
//     {
//         if ( length < DATA_REQUEST_MESSAGE_LENGTH ) return 0;
//         if ( ( buffer[0] == PACKET_START_CHAR ) &&
//                 ( buffer[1] == BINARY_PACKET_INDICATOR_CHAR ) &&
//                 ( buffer[2] == DATA_REQUEST_MESSAGE_LENGTH - 2) &&
//                 ( buffer[3] == MSGID_DATA_REQUEST ) ) {

//             if ( !verifyChecksum( buffer, DATA_REQUEST_CHECKSUM_INDEX ) ) return 0;

//             type = (AHRS_DATA_TYPE)buffer[DATA_REQUEST_DATATYPE_VALUE_INDEX];
//             subtype = (AHRS_TUNING_VAR_ID)buffer[DATA_REQUEST_VARIABLEID_VALUE_INDEX];

//             return DATA_REQUEST_MESSAGE_LENGTH;
//         }
//         return 0;
//     }

//     static int encodeBoardIdentityResponse( char *protocol_buffer, uint8_t type, uint8_t fw_rev,
//             uint8_t fw_ver_major, uint8_t fw_ver_minor, uint16_t fw_revision,
//             uint8_t *unique_id)
//     {
//         // Header
//         protocol_buffer[0] = PACKET_START_CHAR;
//         protocol_buffer[1] = BINARY_PACKET_INDICATOR_CHAR;
//         protocol_buffer[2] = BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH - 2;
//         protocol_buffer[3] = MSGID_BOARD_IDENTITY_RESPONSE;
//         // Data
//         protocol_buffer[BOARD_IDENTITY_BOARDTYPE_VALUE_INDEX] = type;
//         protocol_buffer[BOARD_IDENTITY_HWREV_VALUE_INDEX] = fw_rev;
//         protocol_buffer[BOARD_IDENTITY_FW_VER_MAJOR] = fw_ver_major;
//         protocol_buffer[BOARD_IDENTITY_FW_VER_MINOR] = fw_ver_minor;
//         IMURegisters::encodeProtocolUint16(fw_revision,&protocol_buffer[BOARD_IDENTITY_FW_VER_REVISION_VALUE_INDEX]);
//         for ( int i = 0; i < 12; i++ ) {
//             protocol_buffer[BOARD_IDENTITY_UNIQUE_ID_0 + i] = unique_id[i];
//         }
//         // Footer
//         encodeTermination( protocol_buffer, BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH, BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH - 4 );
//         return BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH;
//     }

//     static int decodeBoardIdentityResponse( char *buffer, int length,  pub struct BoardID& update )
//     {
//         if ( length < BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH ) return 0;
//         if ( ( buffer[0] == PACKET_START_CHAR ) &&
//                 ( buffer[1] == BINARY_PACKET_INDICATOR_CHAR ) &&
//                 ( buffer[2] == BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH - 2) &&
//                 ( buffer[3] == MSGID_BOARD_IDENTITY_RESPONSE ) ) {
//             if ( !verifyChecksum( buffer, BOARD_IDENTITY_RESPONSE_CHECKSUM_INDEX ) ) return 0;

//             update.type = buffer[BOARD_IDENTITY_BOARDTYPE_VALUE_INDEX];
//             update.hw_rev = buffer[BOARD_IDENTITY_HWREV_VALUE_INDEX];
//             update.fw_ver_major = buffer[BOARD_IDENTITY_FW_VER_MAJOR];
//             update.fw_ver_minor = buffer[BOARD_IDENTITY_FW_VER_MINOR];
//             update.fw_revision = IMURegisters::decodeProtocolUint16(&buffer[BOARD_IDENTITY_FW_VER_REVISION_VALUE_INDEX]);
//             for ( int i = 0; i < 12; i++ ) {
//                 update.unique_id[i] = buffer[BOARD_IDENTITY_UNIQUE_ID_0 + i];
//             }
//             return BOARD_IDENTITY_RESPONSE_MESSAGE_LENGTH;
//         }
//         return 0;
//     }

// };
