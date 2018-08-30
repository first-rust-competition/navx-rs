/*

   Deprecated
   (idk how to mark it in rust)

*/





pub const PACKET_START_CHAR: u8 = '!' as u8;
pub const CHECKSUM_LENGTH: i8 = 2;   /* 8-bit checksump, all bytes before checksum */
pub const TERMINATOR_LENGTH: i8 = 2;   /* Carriage Return, Line Feed */

pub const PROTOCOL_FLOAT_LENGTH: u16 = 7;


// Yaw/Pitch/Roll (YPR) Update Packet - e.g., !y[yaw][pitch][roll][compass]
// (All values as floats)

pub const MSGID_YPR_UPDATE: u8 = 'y' as u8;
pub const YPR_UPDATE_YAW_VALUE_INDEX: i32 = 2;
pub const YPR_UPDATE_ROLL_VALUE_INDEX: i32 = 9;
pub const YPR_UPDATE_PITCH_VALUE_INDEX: i32 = 16;
pub const YPR_UPDATE_COMPASS_VALUE_INDEX: i32 = 23;
pub const YPR_UPDATE_CHECKSUM_INDEX: i32 = 30;
pub const YPR_UPDATE_TERMINATOR_INDEX: i32 = 32;
pub const YPR_UPDATE_MESSAGE_LENGTH: i32 = 34;

// Quaternion Update Packet - e.g., !r[q1][q2][q3][q4][accelx][accely][accelz][magx][magy][magz]

pub const MSGID_QUATERNION_UPDATE: u8 = 'q' as u8;
pub const QUATERNION_UPDATE_QUAT1_VALUE_INDEX: i32 = 2;
pub const QUATERNION_UPDATE_QUAT2_VALUE_INDEX: i32 = 6;
pub const QUATERNION_UPDATE_QUAT3_VALUE_INDEX: i32 = 10;
pub const QUATERNION_UPDATE_QUAT4_VALUE_INDEX: i32 = 14;
pub const QUATERNION_UPDATE_ACCEL_X_VALUE_INDEX: i32 = 18;
pub const QUATERNION_UPDATE_ACCEL_Y_VALUE_INDEX: i32 = 22;
pub const QUATERNION_UPDATE_ACCEL_Z_VALUE_INDEX: i32 = 26;
pub const QUATERNION_UPDATE_MAG_X_VALUE_INDEX: i32 = 30;
pub const QUATERNION_UPDATE_MAG_Y_VALUE_INDEX: i32 = 34;
pub const QUATERNION_UPDATE_MAG_Z_VALUE_INDEX: i32 = 38;
pub const QUATERNION_UPDATE_TEMP_VALUE_INDEX: i32 = 42;
pub const QUATERNION_UPDATE_CHECKSUM_INDEX: i32 = 49;
pub const QUATERNION_UPDATE_TERMINATOR_INDEX: i32 = 51;
pub const QUATERNION_UPDATE_MESSAGE_LENGTH: i32 = 53;

// Gyro/Raw Data Update packet - e.g., !g[gx][gy][gz][accelx][accely][accelz][magx][magy][magz][temp_c]

pub const MSGID_GYRO_UPDATE: u8 = 'g' as u8;
pub const GYRO_UPDATE_MESSAGE_LENGTH: i32 = 46;
pub const GYRO_UPDATE_GYRO_X_VALUE_INDEX: i32 = 2;
pub const GYRO_UPDATE_GYRO_Y_VALUE_INDEX: i32 = 6;
pub const GYRO_UPDATE_GYRO_Z_VALUE_INDEX: i32 = 10;
pub const GYRO_UPDATE_ACCEL_X_VALUE_INDEX: i32 = 14;
pub const GYRO_UPDATE_ACCEL_Y_VALUE_INDEX: i32 = 18;
pub const GYRO_UPDATE_ACCEL_Z_VALUE_INDEX: i32 = 22;
pub const GYRO_UPDATE_MAG_X_VALUE_INDEX: i32 = 26;
pub const GYRO_UPDATE_MAG_Y_VALUE_INDEX: i32 = 30;
pub const GYRO_UPDATE_MAG_Z_VALUE_INDEX: i32 = 34;
pub const GYRO_UPDATE_TEMP_VALUE_INDEX: i32 = 38;
pub const GYRO_UPDATE_CHECKSUM_INDEX: i32 = 42;
pub const GYRO_UPDATE_TERMINATOR_INDEX: i32 = 44;

// EnableStream Command Packet - e.g., !S[stream type]

pub const MSGID_STREAM_CMD: u8 = 'S' as u8;
pub const STREAM_CMD_STREAM_TYPE_YPR: i32 = MSGID_YPR_UPDATE as i32;
pub const STREAM_CMD_STREAM_TYPE_QUATERNION: i32 = MSGID_QUATERNION_UPDATE as i32;
pub const STREAM_CMD_STREAM_TYPE_RAW: i32 = MSGID_RAW_UPDATE;
pub const STREAM_CMD_STREAM_TYPE_INDEX: i32 = 2;
pub const STREAM_CMD_UPDATE_RATE_HZ_INDEX: i32 = 3;
pub const STREAM_CMD_CHECKSUM_INDEX: i32 = 5;
pub const STREAM_CMD_TERMINATOR_INDEX: i32 = 7;
pub const STREAM_CMD_MESSAGE_LENGTH: i32 = 9;

// EnableStream Response Packet - e.g., !s[stream type][gyro full scale range][accel full scale range][update rate hz][yaw_offset_degrees][q1/2/3/4 offsets][flags][checksum][cr][lf]
pub const MSG_ID_STREAM_RESPONSE: u8 = 's' as u8;
pub const STREAM_RESPONSE_STREAM_TYPE_INDEX: i32 = 2;
pub const STREAM_RESPONSE_GYRO_FULL_SCALE_DPS_RANGE: i32 = 3;
pub const STREAM_RESPONSE_ACCEL_FULL_SCALE_G_RANGE: i32 = 7;
pub const STREAM_RESPONSE_UPDATE_RATE_HZ: i32 = 11;
pub const STREAM_RESPONSE_YAW_OFFSET_DEGREES: i32 = 15;
pub const STREAM_RESPONSE_FLAGS: i32 = 38;
pub const STREAM_RESPONSE_CHECKSUM_INDEX: i32 = 42;
pub const STREAM_RESPONSE_TERMINATOR_INDEX: i32 = 44;
pub const STREAM_RESPONSE_MESSAGE_LENGTH: i32 = 46;

pub const NAV6_FLAG_MASK_CALIBRATION_STATE: u8 = 0x03;
pub const NAV6_CALIBRATION_STATE_WAIT: u8 = 0x00;
pub const NAV6_CALIBRATION_STATE_ACCUMULATE: u8 = 0x01;
pub const NAV6_CALIBRATION_STATE_COMPLETE: u8 = 0x02;


pub const IMU_PROTOCOL_MAX_MESSAGE_LENGTH: i32 = QUATERNION_UPDATE_MESSAGE_LENGTH;


struct YPRUpdate {
    yaw: f32,
    pitch: f32,
    roll: f32,
    compass_heading: f32,
}

struct GyroUpdate {
    gyro_x: u16,
    gyro_y: u16,
    gyro_z: u16,
    accel_x: u16,
    accel_y: u16,
    accel_z: u16,
    mag_x: u16,
    mag_y: u16,
    mag_z: u16,
    temp_c: f32,
}

struct QuaternionUpdate {
    q1: i16,
    q2: i16,
    q3: i16,
    q4: i16,
    accel_x: i16,
    accel_y: i16,
    accel_z: i16,
    mag_x: i16,
    mag_y: i16,
    mag_z: i16,
    temp_c: f32,
}

struct StreamResponse {
    stream_type: u8,
    gyro_fsr_dps: u16,
    accel_fsr_g: u16,
    update_rate_hz: u16,
    yaw_offset_degrees: f32,
    q1_offset: u16,
    q2_offset: u16,
    q3_offset: u16,
    q4_offset: u16,
    flags: u16,
}

pub fn encodeYPRUpdate(protocol_buffer: &mut [u8; 256], yaw: f32, pitch: f32, roll: f32, compass_heading: f32) -> i32 {
// Header
    protocol_buffer[0] = PACKET_START_CHAR;
    protocol_buffer[1] = MSGID_YPR_UPDATE;

// Data
    encodeProtocolFloat(yaw, &protocol_buffer[YPR_UPDATE_YAW_VALUE_INDEX]);
    encodeProtocolFloat(pitch, &protocol_buffer[YPR_UPDATE_PITCH_VALUE_INDEX]);
    encodeProtocolFloat(roll, &protocol_buffer[YPR_UPDATE_ROLL_VALUE_INDEX]);
    encodeProtocolFloat(compass_heading, &protocol_buffer[YPR_UPDATE_COMPASS_VALUE_INDEX]);

// Footer
    encodeTermination(protocol_buffer, YPR_UPDATE_MESSAGE_LENGTH, YPR_UPDATE_MESSAGE_LENGTH - 4);
    Ok(YPR_UPDATE_MESSAGE_LENGTH)
}


pub fn encodeQuaternionUpdate(protocol_buffer: &mut [u8; 256], q1: i16, q2: i16, q3: i16, q4: i16, accel_x: i16, accel_y: i16, accel_z: i16, mag_x: i16, mag_y: i16, mag_z: i16, temp_c: f32) -> i32 {
// Header
    protocol_buffer[0] = PACKET_START_CHAR;
    protocol_buffer[1] = MSGID_QUATERNION_UPDATE;

// Data
    encodeProtocolUint16(q1, &protocol_buffer[QUATERNION_UPDATE_QUAT1_VALUE_INDEX]);
    encodeProtocolUint16(q2, &protocol_buffer[QUATERNION_UPDATE_QUAT2_VALUE_INDEX]);
    encodeProtocolUint16(q3, &protocol_buffer[QUATERNION_UPDATE_QUAT3_VALUE_INDEX]);
    encodeProtocolUint16(q4, &protocol_buffer[QUATERNION_UPDATE_QUAT4_VALUE_INDEX]);
    encodeProtocolUint16(accel_x, &protocol_buffer[QUATERNION_UPDATE_ACCEL_X_VALUE_INDEX]);
    encodeProtocolUint16(accel_y, &protocol_buffer[QUATERNION_UPDATE_ACCEL_Y_VALUE_INDEX]);
    encodeProtocolUint16(accel_z, &protocol_buffer[QUATERNION_UPDATE_ACCEL_Z_VALUE_INDEX]);
    encodeProtocolUint16(mag_x as u16, &protocol_buffer[QUATERNION_UPDATE_MAG_X_VALUE_INDEX]);
    encodeProtocolUint16(mag_y as u16, &protocol_buffer[QUATERNION_UPDATE_MAG_Y_VALUE_INDEX]);
    encodeProtocolUint16(mag_z as u16, &protocol_buffer[QUATERNION_UPDATE_MAG_Z_VALUE_INDEX]);
    encodeProtocolFloat(temp_c, &protocol_buffer[QUATERNION_UPDATE_TEMP_VALUE_INDEX]);

// Footer
    encodeTermination(protocol_buffer, QUATERNION_UPDATE_MESSAGE_LENGTH, QUATERNION_UPDATE_MESSAGE_LENGTH - 4);

    Ok(QUATERNION_UPDATE_MESSAGE_LENGTH)
}

pub fn encodeGyroUpdate(protocol_buffer: &mut [u8; 256], gyro_x: u16, gyro_y: u16, gyro_z: u16, accel_x: u16, accel_y: u16, accel_z: u16, mag_x: i16, mag_y: i16, mag_z: i16, temp_c: f32) -> i32 {
// Header
    protocol_buffer[0] = PACKET_START_CHAR;
    protocol_buffer[1] = MSGID_GYRO_UPDATE;

// Data
    encodeProtocolUint16(gyro_x, &protocol_buffer[GYRO_UPDATE_GYRO_X_VALUE_INDEX]);
    encodeProtocolUint16(gyro_y, &protocol_buffer[GYRO_UPDATE_GYRO_Y_VALUE_INDEX]);
    encodeProtocolUint16(gyro_z, &protocol_buffer[GYRO_UPDATE_GYRO_Z_VALUE_INDEX]);
    encodeProtocolUint16(accel_x, &protocol_buffer[GYRO_UPDATE_ACCEL_X_VALUE_INDEX]);
    encodeProtocolUint16(accel_y, &protocol_buffer[GYRO_UPDATE_ACCEL_Y_VALUE_INDEX]);
    encodeProtocolUint16(accel_z, &protocol_buffer[GYRO_UPDATE_ACCEL_Z_VALUE_INDEX]);
    encodeProtocolUint16(mag_x as u16, &protocol_buffer[GYRO_UPDATE_MAG_X_VALUE_INDEX]);
    encodeProtocolUint16(mag_y as u16, &protocol_buffer[GYRO_UPDATE_MAG_Y_VALUE_INDEX]);
    encodeProtocolUint16(mag_z as u16, &protocol_buffer[GYRO_UPDATE_MAG_Z_VALUE_INDEX]);
    encodeProtocolUnsignedHundredthsFloat(temp_c, &protocol_buffer[GYRO_UPDATE_TEMP_VALUE_INDEX]);

// Footer
    encodeTermination(protocol_buffer, GYRO_UPDATE_MESSAGE_LENGTH, GYRO_UPDATE_MESSAGE_LENGTH - 4);

    Ok(GYRO_UPDATE_MESSAGE_LENGTH)
}

pub fn encodeStreamCommand(protocol_buffer: &mut [u8; 256], stream_type: u8, update_rate_hz: u8) -> i32 {
// Header
    protocol_buffer[0] = PACKET_START_CHAR;
    protocol_buffer[1] = MSGID_STREAM_CMD;

// Data
    protocol_buffer[STREAM_CMD_STREAM_TYPE_INDEX] = stream_type;
// convert update_rate_hz to two ascii bytes
    sprintf(&protocol_buffer[STREAM_CMD_UPDATE_RATE_HZ_INDEX], "%02X", update_rate_hz);

// Footer
    encodeTermination(protocol_buffer, STREAM_CMD_MESSAGE_LENGTH, STREAM_CMD_MESSAGE_LENGTH - 4);

    Ok(STREAM_CMD_MESSAGE_LENGTH)
}

pub fn encodeStreamResponse(protocol_buffer: &mut [u8; 256], stream_type: u8, gyro_fsr_dps: u16, accel_fsr_g: u16, update_rate_hz: u16, yaw_offset_degrees: f32, q1_offset: u16, q2_offset: u16, q3_offset: u16, q4_offset: u16, flags: u16) -> i32 {
// Header
    protocol_buffer[0] = PACKET_START_CHAR;
    protocol_buffer[1] = MSG_ID_STREAM_RESPONSE;

// Data
    protocol_buffer[STREAM_RESPONSE_STREAM_TYPE_INDEX] = stream_type;
    encodeProtocolUint16(gyro_fsr_dps, &protocol_buffer[STREAM_RESPONSE_GYRO_FULL_SCALE_DPS_RANGE]);
    encodeProtocolUint16(accel_fsr_g, &protocol_buffer[STREAM_RESPONSE_ACCEL_FULL_SCALE_G_RANGE]);
    encodeProtocolUint16(update_rate_hz, &protocol_buffer[STREAM_RESPONSE_UPDATE_RATE_HZ]);
    encodeProtocolFloat(yaw_offset_degrees, &protocol_buffer[STREAM_RESPONSE_YAW_OFFSET_DEGREES]);
    encodeProtocolUint16(q1_offset, &protocol_buffer[STREAM_RESPONSE_QUAT1_OFFSET]);
    encodeProtocolUint16(q2_offset, &protocol_buffer[STREAM_RESPONSE_QUAT2_OFFSET]);
    encodeProtocolUint16(q3_offset, &protocol_buffer[STREAM_RESPONSE_QUAT3_OFFSET]);
    encodeProtocolUint16(q4_offset, &protocol_buffer[STREAM_RESPONSE_QUAT4_OFFSET]);
    encodeProtocolUint16(flags, &protocol_buffer[STREAM_RESPONSE_FLAGS]);

// Footer
    encodeTermination(protocol_buffer, STREAM_RESPONSE_MESSAGE_LENGTH, STREAM_RESPONSE_MESSAGE_LENGTH - 4);

    Ok(STREAM_RESPONSE_MESSAGE_LENGTH)
}

pub fn decodeStreamResponse(buffer: [u8; 256], length: i32, rsp: &mut StreamResponse) -> i32 {
    if (length < STREAM_RESPONSE_MESSAGE_LENGTH) {
        Ok(0)
    } else if ((buffer[0] == PACKET_START_CHAR) && (buffer[1] == MSG_ID_STREAM_RESPONSE)) {
        if (!verifyChecksum(buffer, STREAM_RESPONSE_CHECKSUM_INDEX)) {
            Ok(0)
        }

        rsp.stream_type = buffer[2];
        rsp.gyro_fsr_dps = decodeProtocolUint16(&buffer[STREAM_RESPONSE_GYRO_FULL_SCALE_DPS_RANGE]);
        rsp.accel_fsr_g = decodeProtocolUint16(&buffer[STREAM_RESPONSE_ACCEL_FULL_SCALE_G_RANGE]);
        rsp.update_rate_hz = decodeProtocolUint16(&buffer[STREAM_RESPONSE_UPDATE_RATE_HZ]);
        rsp.yaw_offset_degrees = decodeProtocolFloat(&buffer[STREAM_RESPONSE_YAW_OFFSET_DEGREES]);
        rsp.q1_offset = decodeProtocolUint16(&buffer[STREAM_RESPONSE_QUAT1_OFFSET]);
        rsp.q2_offset = decodeProtocolUint16(&buffer[STREAM_RESPONSE_QUAT2_OFFSET]);
        rsp.q3_offset = decodeProtocolUint16(&buffer[STREAM_RESPONSE_QUAT3_OFFSET]);
        rsp.q4_offset = decodeProtocolUint16(&buffer[STREAM_RESPONSE_QUAT4_OFFSET]);
        rsp.flags = decodeProtocolUint16(&buffer[STREAM_RESPONSE_FLAGS]);
        Ok(STREAM_RESPONSE_MESSAGE_LENGTH)
    }
    Ok(0)
}

pub fn decodeStreamCommand(buffer: [u8; 256], length: i32, stream_type: &mut u8, update_rate_hz: &mut u8) -> i32 {
    if length < STREAM_CMD_MESSAGE_LENGTH {
        Ok(0)
    } else if buffer[0] == '!' && buffer[1] == MSGID_STREAM_CMD {
        if !verifyChecksum(buffer, STREAM_CMD_CHECKSUM_INDEX) {
            Ok(0)
        }

        &stream_type = buffer[STREAM_CMD_STREAM_TYPE_INDEX];
        &update_rate_hz = decodeUint8(&buffer[STREAM_CMD_UPDATE_RATE_HZ_INDEX]);

        return STREAM_CMD_MESSAGE_LENGTH;
    }
    Ok(0)
}

pub fn decodeYPRUpdate(buffer: [u8; 256], length: i32, update: &mut YPRUpdate) -> i32 {
    if (length < YPR_UPDATE_MESSAGE_LENGTH) {
        Ok(0)
    } else if buffer[0] == '!' && buffer[1] == 'y' {
        if !verifyChecksum(buffer, YPR_UPDATE_CHECKSUM_INDEX) {
            Ok(0)
        }

        update.yaw = decodeProtocolFloat(&buffer[YPR_UPDATE_YAW_VALUE_INDEX]);
        update.pitch = decodeProtocolFloat(&buffer[YPR_UPDATE_PITCH_VALUE_INDEX]);
        update.roll = decodeProtocolFloat(&buffer[YPR_UPDATE_ROLL_VALUE_INDEX]);
        update.compass_heading = decodeProtocolFloat(&buffer[YPR_UPDATE_COMPASS_VALUE_INDEX]);
        return YPR_UPDATE_MESSAGE_LENGTH;
    }
    Ok(0)
}

pub fn decodeQuaternionUpdate(buffer: [u8; 256], length: i32, update: &mut QuaternionUpdate) -> i32 {
    if length < QUATERNION_UPDATE_MESSAGE_LENGTH {
        Ok(0)
    } else if (buffer[0] == PACKET_START_CHAR) & &buffer[1] == MSGID_QUATERNION_UPDATE {
        if (!verifyChecksum(buffer, QUATERNION_UPDATE_CHECKSUM_INDEX)) {
            Ok(0)
        }

        update.q1 = decodeProtocolUint16(&buffer[QUATERNION_UPDATE_QUAT1_VALUE_INDEX]) as i16;
        update.q2 = decodeProtocolUint16(&buffer[QUATERNION_UPDATE_QUAT2_VALUE_INDEX]) as i16;
        update.q3 = decodeProtocolUint16(&buffer[QUATERNION_UPDATE_QUAT3_VALUE_INDEX]) as i16;
        update.q4 = decodeProtocolUint16(&buffer[QUATERNION_UPDATE_QUAT4_VALUE_INDEX]) as i16;
        update.accel_x = decodeProtocolUint16(&buffer[QUATERNION_UPDATE_ACCEL_X_VALUE_INDEX]) as i16;
        update.accel_y = decodeProtocolUint16(&buffer[QUATERNION_UPDATE_ACCEL_Y_VALUE_INDEX]) as i16;
        update.accel_z = decodeProtocolUint16(&buffer[QUATERNION_UPDATE_ACCEL_Z_VALUE_INDEX]) as i16;
        update.mag_x = decodeProtocolUint16(&buffer[QUATERNION_UPDATE_MAG_X_VALUE_INDEX]) as i16;
        update.mag_y = decodeProtocolUint16(&buffer[QUATERNION_UPDATE_MAG_Y_VALUE_INDEX]) as i16;
        update.mag_z = decodeProtocolUint16(&buffer[QUATERNION_UPDATE_MAG_Z_VALUE_INDEX]) as i16;
        update.temp_c = decodeProtocolFloat(&buffer[QUATERNION_UPDATE_TEMP_VALUE_INDEX]);
        return QUATERNION_UPDATE_MESSAGE_LENGTH;
    }
    Ok(0)
}

pub fn decodeGyroUpdate(buffer: [u8; 256], length: i32, update: &mut GyroUpdate) -> i32 {
    if length < GYRO_UPDATE_MESSAGE_LENGTH {
        Ok(0)
    } else if buffer[0] == PACKET_START_CHAR && buffer[1] == MSGID_GYRO_UPDATE {
        if !verifyChecksum(buffer, GYRO_UPDATE_CHECKSUM_INDEX) {
            Ok(0)
        }

        update.gyro_x = decodeProtocolUint16(&buffer[GYRO_UPDATE_GYRO_X_VALUE_INDEX]);
        update.gyro_y = decodeProtocolUint16(&buffer[GYRO_UPDATE_GYRO_Y_VALUE_INDEX]);
        update.gyro_z = decodeProtocolUint16(&buffer[GYRO_UPDATE_GYRO_Z_VALUE_INDEX]);
        update.accel_x = decodeProtocolUint16(&buffer[GYRO_UPDATE_ACCEL_X_VALUE_INDEX]);
        update.accel_y = decodeProtocolUint16(&buffer[GYRO_UPDATE_ACCEL_Y_VALUE_INDEX]);
        update.accel_z = decodeProtocolUint16(&buffer[GYRO_UPDATE_ACCEL_Z_VALUE_INDEX]);
        update.mag_x = decodeProtocolUint16(&buffer[GYRO_UPDATE_MAG_X_VALUE_INDEX]) as u16;
        update.mag_y = decodeProtocolUint16(&buffer[GYRO_UPDATE_MAG_Y_VALUE_INDEX]) as u16;
        update.mag_z = decodeProtocolUint16(&buffer[GYRO_UPDATE_MAG_Z_VALUE_INDEX]) as u16;
        update.temp_c = decodeProtocolUnsignedHundredthsFloat(&buffer[GYRO_UPDATE_TEMP_VALUE_INDEX]);
        return GYRO_UPDATE_MESSAGE_LENGTH;
    }
    Ok(0)
}

fn encodeTermination(buffer: &mut [u8; 256], length: i32, content_length: i32) {
    if total_length >= CHECKSUM_LENGTH + TERMINATOR_LENGTH && total_length >= content_length + CHECKSUM_LENGTH + TERMINATOR_LENGTH {
        // Checksum
        let mut checksum: u8 = 0;
        for i in 0..(content_length - 1) {
            checksum += buffer[i];
        }

        // convert checksum to two ascii bytes
        sprintf(&buffer[content_length], "%02X", checksum);
        // Message Terminator
        sprintf(&buffer[content_length + CHECKSUM_LENGTH], "%s", "\r\n");
    }
}

// Formats a float as follows
//
// e.g., -129.235
//
// "-129.24"
//
// e.g., 23.4
//
// "+023.40"

fn encodeProtocolFloat(f: f32, buffer: &mut [u8; 256]) {
    let work_buffer: [u8; PROTOCOL_FLOAT_LENGTH + 1 as usize];
    let i;
    let temp1 = i32::abs(((f - f as i32) * 100) as i32);
    if f < 0 {
        buff[0] = '-';
    } else {
        buff[0] = ' ';
    }
    sprintf(work_buffer, "%03d.%02d", i32::abs(f as i32), temp1);
    for i in 0..(PROTOCOL_FLOAT_LENGTH - 2) {
        buff[1 + i] = work_buffer[i];
    }
}

fn encodeProtocolUint16(value: u8, buffer: &mut [u8; 2]) {
    sprintf(&buff[0], "%04X", value);
}

fn decodeProtocolUint16(uint16_string: [u16; 2]) -> u16 {
    let mut decoded_uint16: u16 = 0;
    let mut shift_left: u32 = 12;
    for i in 0..4 {
        let digit: u8 = {
            if uint16_string[i] <= '9' { uint16_string[i] - '0' }
            ((uint16_string[i] - 'A') + 10)
        };
        decoded_uint16 += ((digit as u16) << shift_left);
        shift_left -= 4;
    }
    return decoded_uint16;
}

/* 0 to 655.35 */
fn decodeProtocolUnsignedHundredthsFloat(uint8_unsigned_hundredths_float: [u16; 2]) -> f32 {
    let mut unsigned_float = decodeProtocolUint16(uint8_unsigned_hundredths_float) as f32;
    unsigned_float /= 100;
    return unsigned_float;
}

fn encodeProtocolUnsignedHundredthsFloat(input: f32, uint8_unsigned_hundredths_float: &mut [u8; 2]) {
    let input_as_uint = (uint16_t)(input * 100.0) as u8;
    encodeProtocolUint16(input_as_uint, uint8_unsigned_hundredths_float);
}

fn verifyChecksum(buffer: &mut [u8; 256], content_length: i32) -> bool {
// Calculate Checksum
    let mut checksum: u8 = 0;
    for i in 0..content_length {
        checksum += buffer[i];
    }

    return checksum == decodeUint8(&buffer[content_length]);
}

static unsigned char decodeUint8( char *checksum )
{
unsigned char first_digit = checksum[0] < = '9' ? checksum[0] - '0': ((checksum[0] - 'A') + 10);
unsigned char second_digit = checksum[1] < = '9' ? checksum[1] - '0' : ((checksum[1] - 'A') + 10);
unsigned char decoded_checksum = (first_digit * 16) + second_digit;
return decoded_checksum;
}

static float decodeProtocolFloat( char *buffer )
{
char temp[PROTOCOL_FLOAT_LENGTH + 1];
for ( int i = 0; i < PROTOCOL_FLOAT_LENGTH; i+ + )
{
temp[i] = buffer[i];
}
temp[PROTOCOL_FLOAT_LENGTH] = 0;
return atof(temp);
}

};