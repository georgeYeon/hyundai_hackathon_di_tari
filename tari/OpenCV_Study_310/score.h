#pragma once

#ifndef _SCORE_
#define _SCORE_

#include <iostream>
#include <string>
#include <time.h>
#include <windows.h>
#include "csvparser.h"
using namespace std;

// 점수 가중치
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

// 요약 데이터
typedef struct SUMMARY_DATA
{
	string key; // 맨앞값
	string group_key; // 그룹키값
	string fuel_type; // 연료타입
	string produce_date; // 생산년월일
	string drive_number; // 주행정보 고유번호
	string start_date; // 시동년월일
	string start_time; // 시동시분초
	string distance; // 주행거리
	string duration; // 운행시간값
	string idle_running; // 공회전시간수
	string heat_time;// 예열시간수
	string avg_velocity; // 평균속도값
	string max_velocity; // 최고속도값
	string low_drive_time; // 저속운행초수
	string middle_low_drive_time; // 중저속운행초수
	string middle_drive_time; // 중속운행초수
	string middle_high_drive_time; // 중고속운행초수
	string high_drive_time; // 고속운행초수
	string pure_drive_time; // 순주행초수
	string burst_7to10; // 급가속 7~10
	string burst_11to13; // 급가속 11~13
	string burst_14to17; // 급가속 14~17
	string burst_18; // 급가속 18 이상
	string reduce_21;// 급감속 -21 이하
	string reduce_18to20; // 급감속 -18 ~ -20
	string reduce_14to17; // 급감속 -14 ~ -17
	string reduce_11to13; // 급감속 -11 ~ -13
	string reduce_7to10; // 급감속 -7 ~ -10
	string destination;// 목적지 POI
} summ_data;

void check_distance(string dist, int& point);
void check_duration(string time, int& point);
void check_drive_speed(string low, string middle_low, string middle, string middle_high, string high, int& point);
void check_burst(string b_7_10, string b_11_13, string b_14_17, string b_18, int& point);
void check_reduce(string r_18_20, string r_14_17, string r_11_13, string r_7_10, int& point);

// 로그 데이터
typedef struct LOG_DATA
{
	string key; // 맨앞값
	string group_key; // 그룹키값
	string start_date; // 시동일시
	string sequence_number; // 시퀀스번호
	string x; // X좌표
	string y; // Y좌표
	string energy_consume; // 에너지 소모율
	string velocity; // 속도
	string rpm; // RPM
	string airconditioner; // 에어컨 상태
	string cooling_water_temperature;// 냉각수온
	string break_status; // 브레이크 상태
	string accel_open_value; // 악셀개도량
	string steering_wheel_ang; // 스티어링 휠 각도
	string gear_level; // 기어단수
	string province; // 광역시도
	string city; // 시군구
	string road_type; // 도로 종류
	string road_slope; // 도로 경사도
	string road_direction; // 도로 방향
	string model_year; // 모델 연식
	string model_type; // 차종 코드
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