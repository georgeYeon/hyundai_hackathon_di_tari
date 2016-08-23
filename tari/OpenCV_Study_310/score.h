#pragma once

#ifndef _SCORE_
#define _SCORE_

#include <iostream>
#include <string>
#include <time.h>
#include <windows.h>
#include "csvparser.h"
using namespace std;

// ���� ����ġ
#define DISTANCE_PLUS 2
#define DISTANCE_MINUS -5
#define DURATION_PLUS 2
#define DURATION_MINUS -5
#define DRIVE_SPEED_MINUS -5
#define DRIVE_SPEED_PLUS 2
#define BURST_7_10 -0
#define BURST_11_13 -1
#define BURST_14_17 -2
#define BURST_18 -5
#define REDUCE_18_20 -5
#define REDUCE_14_17 -2
#define REDUCE_11_13 -1
#define REDUCE_7_10 -0

#define ROAD_TYPE_MINUS -1
#define STEERING_MINUS -1
#define ACCER_MINUS -1

// ��� ������
typedef struct SUMMARY_DATA
{
	string key; // �Ǿհ�
	string group_key; // �׷�Ű��
	string fuel_type; // ����Ÿ��
	string produce_date; // ��������
	string drive_number; // �������� ������ȣ
	string start_date; // �õ������
	string start_time; // �õ��ú���
	string distance; // ����Ÿ�
	string duration; // ����ð���
	string idle_running; // ��ȸ���ð���
	string heat_time;// �����ð���
	string avg_velocity; // ��ռӵ���
	string max_velocity; // �ְ�ӵ���
	string low_drive_time; // ���ӿ����ʼ�
	string middle_low_drive_time; // �����ӿ����ʼ�
	string middle_drive_time; // �߼ӿ����ʼ�
	string middle_high_drive_time; // �߰�ӿ����ʼ�
	string high_drive_time; // ��ӿ����ʼ�
	string pure_drive_time; // �������ʼ�
	string burst_7to10; // �ް��� 7~10
	string burst_11to13; // �ް��� 11~13
	string burst_14to17; // �ް��� 14~17
	string burst_18; // �ް��� 18 �̻�
	string reduce_21;// �ް��� -21 ����
	string reduce_18to20; // �ް��� -18 ~ -20
	string reduce_14to17; // �ް��� -14 ~ -17
	string reduce_11to13; // �ް��� -11 ~ -13
	string reduce_7to10; // �ް��� -7 ~ -10
	string destination;// ������ POI
} summ_data;

void check_distance(string dist, int& point);
void check_duration(string time, int& point);
void check_drive_speed(string low, string middle_low, string middle, string middle_high, string high, int& point);
void check_burst(string b_7_10, string b_11_13, string b_14_17, string b_18, int& point);
void check_reduce(string r_18_20, string r_14_17, string r_11_13, string r_7_10, int& point);

// �α� ������
typedef struct LOG_DATA
{
	string key; // �Ǿհ�
	string group_key; // �׷�Ű��
	string start_date; // �õ��Ͻ�
	string sequence_number; // ��������ȣ
	string x; // X��ǥ
	string y; // Y��ǥ
	string energy_consume; // ������ �Ҹ���
	string velocity; // �ӵ�
	string rpm; // RPM
	string airconditioner; // ������ ����
	string cooling_water_temperature;// �ð�����
	string break_status; // �극��ũ ����
	string accel_open_value; // �Ǽ�������
	string steering_wheel_ang; // ��Ƽ� �� ����
	string gear_level; // ���ܼ�
	string province; // �����õ�
	string city; // �ñ���
	string road_type; // ���� ����
	string road_slope; // ���� ��絵
	string road_direction; // ���� ����
	string model_year; // �� ����
	string model_type; // ���� �ڵ�
} log_data;

void check_road_type(string velocity, string road_type, int& point, int& count);
void check_steering_wheel(string previous_v, string current_v, string previous_angle, string current_angle, int& point);
void check_accer(string previous_v, string current_v, int& point);

#define ONE_SECOND 1000

log_data previous;
log_data temp;
int speeding_count = 0;

CsvParser *csvparser = CsvParser_new("log.csv", ",", 1);

int getLogData(char* filename, boolean& IsEnd);
int getSummData(char* filename);



#endif