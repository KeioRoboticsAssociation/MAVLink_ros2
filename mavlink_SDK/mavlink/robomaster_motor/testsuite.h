/** @file
 *    @brief MAVLink comm protocol testsuite generated from robomaster_motor.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef ROBOMASTER_MOTOR_TESTSUITE_H
#define ROBOMASTER_MOTOR_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_robomaster_motor(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_robomaster_motor(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_robomaster_motor_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_robomaster_motor_control_t packet_in = {
        17.0,45.0,73.0,963498088,53,120,187,254
    };
    mavlink_robomaster_motor_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.duty_cycle = packet_in.duty_cycle;
        packet1.target_position_rad = packet_in.target_position_rad;
        packet1.target_speed_rad_s = packet_in.target_speed_rad_s;
        packet1.timeout_ms = packet_in.timeout_ms;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.motor_id = packet_in.motor_id;
        packet1.control_mode = packet_in.control_mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_robomaster_motor_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_control_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.motor_id , packet1.control_mode , packet1.duty_cycle , packet1.target_position_rad , packet1.target_speed_rad_s , packet1.timeout_ms );
    mavlink_msg_robomaster_motor_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.motor_id , packet1.control_mode , packet1.duty_cycle , packet1.target_position_rad , packet1.target_speed_rad_s , packet1.timeout_ms );
    mavlink_msg_robomaster_motor_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_robomaster_motor_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_control_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.motor_id , packet1.control_mode , packet1.duty_cycle , packet1.target_position_rad , packet1.target_speed_rad_s , packet1.timeout_ms );
    mavlink_msg_robomaster_motor_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ROBOMASTER_MOTOR_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ROBOMASTER_MOTOR_CONTROL) != NULL);
#endif
}

static void mavlink_test_robomaster_motor_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_robomaster_motor_status_t packet_in = {
        17.0,45.0,73.0,101.0,963498296,65,132,199
    };
    mavlink_robomaster_motor_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.current_position_rad = packet_in.current_position_rad;
        packet1.current_speed_rad_s = packet_in.current_speed_rad_s;
        packet1.current_duty_cycle = packet_in.current_duty_cycle;
        packet1.position_error_rad = packet_in.position_error_rad;
        packet1.timestamp_ms = packet_in.timestamp_ms;
        packet1.motor_id = packet_in.motor_id;
        packet1.control_mode = packet_in.control_mode;
        packet1.status = packet_in.status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_robomaster_motor_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_status_pack(system_id, component_id, &msg , packet1.motor_id , packet1.control_mode , packet1.status , packet1.current_position_rad , packet1.current_speed_rad_s , packet1.current_duty_cycle , packet1.position_error_rad , packet1.timestamp_ms );
    mavlink_msg_robomaster_motor_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_id , packet1.control_mode , packet1.status , packet1.current_position_rad , packet1.current_speed_rad_s , packet1.current_duty_cycle , packet1.position_error_rad , packet1.timestamp_ms );
    mavlink_msg_robomaster_motor_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_robomaster_motor_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_robomaster_motor_status_send(MAVLINK_COMM_1 , packet1.motor_id , packet1.control_mode , packet1.status , packet1.current_position_rad , packet1.current_speed_rad_s , packet1.current_duty_cycle , packet1.position_error_rad , packet1.timestamp_ms );
    mavlink_msg_robomaster_motor_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ROBOMASTER_MOTOR_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ROBOMASTER_MOTOR_STATUS) != NULL);
#endif
}

static void mavlink_test_dc_motor_control(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DC_MOTOR_CONTROL >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_dc_motor_control_t packet_in = {
        17.0,45.0,73.0,41,108,175,242
    };
    mavlink_dc_motor_control_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_value = packet_in.target_value;
        packet1.speed_limit_rad_s = packet_in.speed_limit_rad_s;
        packet1.acceleration_limit_rad_s2 = packet_in.acceleration_limit_rad_s2;
        packet1.target_system = packet_in.target_system;
        packet1.target_component = packet_in.target_component;
        packet1.motor_id = packet_in.motor_id;
        packet1.control_mode = packet_in.control_mode;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DC_MOTOR_CONTROL_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DC_MOTOR_CONTROL_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_dc_motor_control_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_dc_motor_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_dc_motor_control_pack(system_id, component_id, &msg , packet1.target_system , packet1.target_component , packet1.motor_id , packet1.control_mode , packet1.target_value , packet1.speed_limit_rad_s , packet1.acceleration_limit_rad_s2 );
    mavlink_msg_dc_motor_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_dc_motor_control_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.target_system , packet1.target_component , packet1.motor_id , packet1.control_mode , packet1.target_value , packet1.speed_limit_rad_s , packet1.acceleration_limit_rad_s2 );
    mavlink_msg_dc_motor_control_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_dc_motor_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_dc_motor_control_send(MAVLINK_COMM_1 , packet1.target_system , packet1.target_component , packet1.motor_id , packet1.control_mode , packet1.target_value , packet1.speed_limit_rad_s , packet1.acceleration_limit_rad_s2 );
    mavlink_msg_dc_motor_control_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DC_MOTOR_CONTROL") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DC_MOTOR_CONTROL) != NULL);
#endif
}

static void mavlink_test_dc_motor_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DC_MOTOR_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_dc_motor_status_t packet_in = {
        17.0,45.0,73.0,101.0,129.0,963498504,77,144,211
    };
    mavlink_dc_motor_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.position_rad = packet_in.position_rad;
        packet1.speed_rad_s = packet_in.speed_rad_s;
        packet1.duty_cycle = packet_in.duty_cycle;
        packet1.position_error_rad = packet_in.position_error_rad;
        packet1.target_value = packet_in.target_value;
        packet1.timestamp_ms = packet_in.timestamp_ms;
        packet1.motor_id = packet_in.motor_id;
        packet1.control_mode = packet_in.control_mode;
        packet1.status = packet_in.status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DC_MOTOR_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DC_MOTOR_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_dc_motor_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_dc_motor_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_dc_motor_status_pack(system_id, component_id, &msg , packet1.motor_id , packet1.control_mode , packet1.status , packet1.position_rad , packet1.speed_rad_s , packet1.duty_cycle , packet1.position_error_rad , packet1.target_value , packet1.timestamp_ms );
    mavlink_msg_dc_motor_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_dc_motor_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_id , packet1.control_mode , packet1.status , packet1.position_rad , packet1.speed_rad_s , packet1.duty_cycle , packet1.position_error_rad , packet1.target_value , packet1.timestamp_ms );
    mavlink_msg_dc_motor_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_dc_motor_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_dc_motor_status_send(MAVLINK_COMM_1 , packet1.motor_id , packet1.control_mode , packet1.status , packet1.position_rad , packet1.speed_rad_s , packet1.duty_cycle , packet1.position_error_rad , packet1.target_value , packet1.timestamp_ms );
    mavlink_msg_dc_motor_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("DC_MOTOR_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_DC_MOTOR_STATUS) != NULL);
#endif
}

static void mavlink_test_motor_command(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_MOTOR_COMMAND >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_motor_command_t packet_in = {
        17.0,17,84,151
    };
    mavlink_motor_command_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.target_value = packet_in.target_value;
        packet1.motor_id = packet_in.motor_id;
        packet1.control_mode = packet_in.control_mode;
        packet1.enable = packet_in.enable;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_MOTOR_COMMAND_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_MOTOR_COMMAND_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_command_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_motor_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_command_pack(system_id, component_id, &msg , packet1.motor_id , packet1.control_mode , packet1.target_value , packet1.enable );
    mavlink_msg_motor_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_command_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_id , packet1.control_mode , packet1.target_value , packet1.enable );
    mavlink_msg_motor_command_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_motor_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_motor_command_send(MAVLINK_COMM_1 , packet1.motor_id , packet1.control_mode , packet1.target_value , packet1.enable );
    mavlink_msg_motor_command_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("MOTOR_COMMAND") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_MOTOR_COMMAND) != NULL);
#endif
}

static void mavlink_test_rs485_motor_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RS485_MOTOR_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rs485_motor_status_t packet_in = {
        17.0,45.0,73.0,101.0,963498296,65,132,199,10,77,144
    };
    mavlink_rs485_motor_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.current_position_rotations = packet_in.current_position_rotations;
        packet1.current_velocity_rps = packet_in.current_velocity_rps;
        packet1.target_velocity_rps = packet_in.target_velocity_rps;
        packet1.acceleration_rps2 = packet_in.acceleration_rps2;
        packet1.timestamp_ms = packet_in.timestamp_ms;
        packet1.motor_id = packet_in.motor_id;
        packet1.device_id = packet_in.device_id;
        packet1.motor_index = packet_in.motor_index;
        packet1.control_mode = packet_in.control_mode;
        packet1.status = packet_in.status;
        packet1.error_code = packet_in.error_code;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RS485_MOTOR_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RS485_MOTOR_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_motor_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rs485_motor_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_motor_status_pack(system_id, component_id, &msg , packet1.motor_id , packet1.device_id , packet1.motor_index , packet1.control_mode , packet1.status , packet1.error_code , packet1.current_position_rotations , packet1.current_velocity_rps , packet1.target_velocity_rps , packet1.acceleration_rps2 , packet1.timestamp_ms );
    mavlink_msg_rs485_motor_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_motor_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_id , packet1.device_id , packet1.motor_index , packet1.control_mode , packet1.status , packet1.error_code , packet1.current_position_rotations , packet1.current_velocity_rps , packet1.target_velocity_rps , packet1.acceleration_rps2 , packet1.timestamp_ms );
    mavlink_msg_rs485_motor_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rs485_motor_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_motor_status_send(MAVLINK_COMM_1 , packet1.motor_id , packet1.device_id , packet1.motor_index , packet1.control_mode , packet1.status , packet1.error_code , packet1.current_position_rotations , packet1.current_velocity_rps , packet1.target_velocity_rps , packet1.acceleration_rps2 , packet1.timestamp_ms );
    mavlink_msg_rs485_motor_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RS485_MOTOR_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RS485_MOTOR_STATUS) != NULL);
#endif
}

static void mavlink_test_encoder_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_ENCODER_STATUS >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_encoder_status_t packet_in = {
        963497464,45.0,73.0,963498088,129.0,963498504,77,144,211
    };
    mavlink_encoder_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.raw_count = packet_in.raw_count;
        packet1.position_rad = packet_in.position_rad;
        packet1.velocity_rad_s = packet_in.velocity_rad_s;
        packet1.resolution = packet_in.resolution;
        packet1.gear_ratio = packet_in.gear_ratio;
        packet1.timestamp_ms = packet_in.timestamp_ms;
        packet1.encoder_id = packet_in.encoder_id;
        packet1.encoder_type = packet_in.encoder_type;
        packet1.status = packet_in.status;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_ENCODER_STATUS_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_ENCODER_STATUS_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_encoder_status_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_encoder_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_encoder_status_pack(system_id, component_id, &msg , packet1.encoder_id , packet1.encoder_type , packet1.status , packet1.raw_count , packet1.position_rad , packet1.velocity_rad_s , packet1.resolution , packet1.gear_ratio , packet1.timestamp_ms );
    mavlink_msg_encoder_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_encoder_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.encoder_id , packet1.encoder_type , packet1.status , packet1.raw_count , packet1.position_rad , packet1.velocity_rad_s , packet1.resolution , packet1.gear_ratio , packet1.timestamp_ms );
    mavlink_msg_encoder_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_encoder_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_encoder_status_send(MAVLINK_COMM_1 , packet1.encoder_id , packet1.encoder_type , packet1.status , packet1.raw_count , packet1.position_rad , packet1.velocity_rad_s , packet1.resolution , packet1.gear_ratio , packet1.timestamp_ms );
    mavlink_msg_encoder_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("ENCODER_STATUS") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_ENCODER_STATUS) != NULL);
#endif
}

static void mavlink_test_rs485_read_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RS485_READ_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rs485_read_request_t packet_in = {
        17235,139,206
    };
    mavlink_rs485_read_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.address = packet_in.address;
        packet1.motor_id = packet_in.motor_id;
        packet1.length = packet_in.length;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RS485_READ_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RS485_READ_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_read_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rs485_read_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_read_request_pack(system_id, component_id, &msg , packet1.motor_id , packet1.address , packet1.length );
    mavlink_msg_rs485_read_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_read_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_id , packet1.address , packet1.length );
    mavlink_msg_rs485_read_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rs485_read_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_read_request_send(MAVLINK_COMM_1 , packet1.motor_id , packet1.address , packet1.length );
    mavlink_msg_rs485_read_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RS485_READ_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RS485_READ_REQUEST) != NULL);
#endif
}

static void mavlink_test_rs485_read_response(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RS485_READ_RESPONSE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rs485_read_response_t packet_in = {
        17235,139,206,{ 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80 },209,20
    };
    mavlink_rs485_read_response_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.address = packet_in.address;
        packet1.motor_id = packet_in.motor_id;
        packet1.length = packet_in.length;
        packet1.status = packet_in.status;
        packet1.rs485_error = packet_in.rs485_error;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*64);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RS485_READ_RESPONSE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RS485_READ_RESPONSE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_read_response_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rs485_read_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_read_response_pack(system_id, component_id, &msg , packet1.motor_id , packet1.address , packet1.length , packet1.data , packet1.status , packet1.rs485_error );
    mavlink_msg_rs485_read_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_read_response_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_id , packet1.address , packet1.length , packet1.data , packet1.status , packet1.rs485_error );
    mavlink_msg_rs485_read_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rs485_read_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_read_response_send(MAVLINK_COMM_1 , packet1.motor_id , packet1.address , packet1.length , packet1.data , packet1.status , packet1.rs485_error );
    mavlink_msg_rs485_read_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RS485_READ_RESPONSE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RS485_READ_RESPONSE) != NULL);
#endif
}

static void mavlink_test_rs485_write_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RS485_WRITE_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rs485_write_request_t packet_in = {
        17235,139,206,{ 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80 }
    };
    mavlink_rs485_write_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.address = packet_in.address;
        packet1.motor_id = packet_in.motor_id;
        packet1.length = packet_in.length;
        
        mav_array_memcpy(packet1.data, packet_in.data, sizeof(uint8_t)*64);
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RS485_WRITE_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RS485_WRITE_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_write_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rs485_write_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_write_request_pack(system_id, component_id, &msg , packet1.motor_id , packet1.address , packet1.length , packet1.data );
    mavlink_msg_rs485_write_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_write_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_id , packet1.address , packet1.length , packet1.data );
    mavlink_msg_rs485_write_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rs485_write_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_write_request_send(MAVLINK_COMM_1 , packet1.motor_id , packet1.address , packet1.length , packet1.data );
    mavlink_msg_rs485_write_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RS485_WRITE_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RS485_WRITE_REQUEST) != NULL);
#endif
}

static void mavlink_test_rs485_write_response(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RS485_WRITE_RESPONSE >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rs485_write_response_t packet_in = {
        17235,139,206,17,84
    };
    mavlink_rs485_write_response_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.address = packet_in.address;
        packet1.motor_id = packet_in.motor_id;
        packet1.length = packet_in.length;
        packet1.status = packet_in.status;
        packet1.rs485_error = packet_in.rs485_error;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RS485_WRITE_RESPONSE_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_write_response_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rs485_write_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_write_response_pack(system_id, component_id, &msg , packet1.motor_id , packet1.address , packet1.length , packet1.status , packet1.rs485_error );
    mavlink_msg_rs485_write_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_write_response_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_id , packet1.address , packet1.length , packet1.status , packet1.rs485_error );
    mavlink_msg_rs485_write_response_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rs485_write_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_write_response_send(MAVLINK_COMM_1 , packet1.motor_id , packet1.address , packet1.length , packet1.status , packet1.rs485_error );
    mavlink_msg_rs485_write_response_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RS485_WRITE_RESPONSE") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RS485_WRITE_RESPONSE) != NULL);
#endif
}

static void mavlink_test_rs485_flash_save_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
    mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_RS485_FLASH_SAVE_REQUEST >= 256) {
            return;
        }
#endif
    mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
    mavlink_rs485_flash_save_request_t packet_in = {
        5
    };
    mavlink_rs485_flash_save_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.motor_id = packet_in.motor_id;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_RS485_FLASH_SAVE_REQUEST_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_RS485_FLASH_SAVE_REQUEST_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_flash_save_request_encode(system_id, component_id, &msg, &packet1);
    mavlink_msg_rs485_flash_save_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_flash_save_request_pack(system_id, component_id, &msg , packet1.motor_id );
    mavlink_msg_rs485_flash_save_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_flash_save_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.motor_id );
    mavlink_msg_rs485_flash_save_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
            comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
    mavlink_msg_rs485_flash_save_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
    mavlink_msg_rs485_flash_save_request_send(MAVLINK_COMM_1 , packet1.motor_id );
    mavlink_msg_rs485_flash_save_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

#ifdef MAVLINK_HAVE_GET_MESSAGE_INFO
    MAVLINK_ASSERT(mavlink_get_message_info_by_name("RS485_FLASH_SAVE_REQUEST") != NULL);
    MAVLINK_ASSERT(mavlink_get_message_info_by_id(MAVLINK_MSG_ID_RS485_FLASH_SAVE_REQUEST) != NULL);
#endif
}

static void mavlink_test_robomaster_motor(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_robomaster_motor_control(system_id, component_id, last_msg);
    mavlink_test_robomaster_motor_status(system_id, component_id, last_msg);
    mavlink_test_dc_motor_control(system_id, component_id, last_msg);
    mavlink_test_dc_motor_status(system_id, component_id, last_msg);
    mavlink_test_motor_command(system_id, component_id, last_msg);
    mavlink_test_rs485_motor_status(system_id, component_id, last_msg);
    mavlink_test_encoder_status(system_id, component_id, last_msg);
    mavlink_test_rs485_read_request(system_id, component_id, last_msg);
    mavlink_test_rs485_read_response(system_id, component_id, last_msg);
    mavlink_test_rs485_write_request(system_id, component_id, last_msg);
    mavlink_test_rs485_write_response(system_id, component_id, last_msg);
    mavlink_test_rs485_flash_save_request(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ROBOMASTER_MOTOR_TESTSUITE_H
