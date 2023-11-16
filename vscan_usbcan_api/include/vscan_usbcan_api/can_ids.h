/*
 * can_ids.h
 *
 *  Created on: 15 апр. 2022 г.
 *      Author: doodu
 */

#ifndef INC_CAN_IDS_H_
#define INC_CAN_IDS_H_

// ПРИОРИТЕТЫ фрэймов:
// 1 - экстренная остановка					0x000
// 1 - sync, heartbeat          		0x00f
// 	2 - ошибка											0x0nm, где n - код уст-ва, а m - код ошибки
// 	2 - переключение режима работы  0x1fm, где m - код режима работы
// 		3 - настройка									0x2nm, где n - код уст-ва, а m - код параметра
// 		3 - команды траектории				0x3nm, где n - код уст-ва, m - код данных
//  			4 - фрэймы состояния 			0x4nm, где n - код уст-ва, m - код данных

#define STANDART_ID
#ifndef STANDART_ID
	#define EXTENDED_ID
#endif

enum MAIN_PUPOSE_ID_CODES
{
	CAN_EMERGENCY_ID = 	0x000,  // фрэйм экстренной остановки, останавливает робота
	CAN_HEARTBEAT_ID = 	0x00f,  // фрэйм синхронизации

#ifdef STANDART_ID
	DRV_ERROR_ID = 			0x100,  // фрэйм ошибки драйвера
	DRV_MODE_ID = 			0x200,  // фрэйм смена режимов работы драйверов (всех)
	DRV_CFG_ID  =       0x300,  // фрэйм настройки параметров драйвера
	DRV_CMD_ID = 			  0x400,  // фрэйм команды
	DRV_STATE_ID =      0x500   // фрэйм состояния драйвера
#else
	DRV_MODE_ID = 			0x0800,  // фрэйм смена режимов работы драйверов (всех)
	DRV_CFG_ID  =       0x1000,  // фрэйм настройки параметров драйвера
	DRV_CMD_ID = 			  0x1800,  // фрэйм команды
	DRV_STATE_ID =      0x2000   // фрэйм состояния драйвера
#endif
};


// пример: error_id = DRV_ERROR_ID | DRV_ERROR_CODE | DEV_ID
// error_id = 0x001 - 0x00e
enum DRV_ERROR_CODES
{
	NO_ERROR = 							0x000,	// НЕ ИСПОЛЬЗОВАТЬ для Id сообщений, т.к. CAN_EMERGENCY_ID = 0x000
	NO_ERROR_STOPPED = 			0x001,  // код ошибки: нет ошибки, драйвер остановлен
	NO_ERROR_CALIBR = 			0x002,  // код ошибки: нет ошибки, происходит калибровка энкодера (или возврат в начало)
	SOFTWARE_ERROR = 		    0x004,  // код ошибки: ошибка в работе программы
};


// пример: change_mode_id = DRV_MODE_ID | MODE_CODE
enum MODE_CODES
{
	//MODE_STOP = 0x000,   // код режима: остановка <-- НЕ НУЖНО использовать, т.к. есть CAN_STOP_ID
	MODE_CALIBR = 0x001,   // код режима: калибровка
	MODE_NORMAL = 0x002    // код режима: норм
};


// коды команды настройки драйверов
// пример: config_id = DRV_CFG_ID | CFG_CODE | DEV_CODE
enum CFG_CODES
{
	CFG_PID_P = 0x001,   // код параметров: П-коэффициент
	CFG_PID_I = 0x002,   // код параметров: И-коэффициент
	CFG_PID_D = 0x003    // код параметров: Д-коэффициент
};

// код фрэймов состояния
// пример: state_id = DRV_STATE_ID | STATE_CODE | DEV_CODE
// пример: trajectory_cmd_id = DRV_CMD_ID | STATE_CODE | DEV_CODE
// коды для фрэймов состояния или фрэймов с командами траектории
enum STATE_CODES
{
	MOTOR_POS = 				0x000, // код фрэйма: положение
	MOTOR_VEL = 				0x001, // код фрэйма: скорость
	MOTOR_POS_VEL = 			0x002,  // код фрэйма: положение и скорость
	MOTOR_TAU = 				0x003 // код фрэйма: момент, ток или напряжение
};

// DEV_CODE, коды устройства, которое должно принять или отправляет фрэйм
enum DEV_CODES
{
	TO_ALL_CODE = 0x000,   // 0000 код фрэйма: всем, ПК
	DRV_1_CODE = 	0x020,   // 0010 код фрэйма: драйвер 1
	DRV_2_CODE = 	0x040,   // 0100 код фрэйма: драйвер 2
	DRV_3_CODE = 	0x080,   // 1000 код фрэйма: драйвер 3
#ifdef EXTENDED_ID
	DRV_4_CODE = 	0x100,   // 0010 код фрэйма: драйвер 4
	DRV_5_CODE = 	0x200,   // 0100 код фрэйма: драйвер 5
	DRV_6_CODE = 	0x400    // 1000 код фрэйма: драйвер 6
#endif
};

// маски для аппаратной фильтрации CAN-кадров
enum CAN_FILTER_ID_MASKS
{
	DRV_1_MASK_ID = 0x0c0,
	DRV_2_MASK_ID	= 0x0a0,
	DRV_3_MASK_ID	= 0x060
};


#endif /* INC_CAN_IDS_H_ */
