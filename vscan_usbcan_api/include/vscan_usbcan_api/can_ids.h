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
// 	2 - остановка										0x100
// 	2 - переключение режима работы  0x1fm, где m - код режима работы
// 		3 - настройка									0x2nm, где n - код уст-ва, а m - код параметра
// 		3 - команды траектории				0x3nm, где n - код уст-ва, m - код данных
//  			4 - фрэймы состояния 			0x4nm, где n - код уст-ва, m - код данных


#define CAN_EMERGENCY_ID    0x000   // фрэйм экстренной остановки, включает тормоз, выключает регуляторы?
#define CAN_HEARTBEAT_ID    0x00f   // фрэйм синхронизации

// пример: error_id = ERROR_CODE | DEV_ID
// error_id = 0x001 - 0x00e
#define NO_ERROR					0x000	 // НЕ ИСПОЛЬЗОВАТЬ для Id сообщений, т.к. CAN_EMERGENCY_ID = 0x000
#define PID_CMD_ERROR				0x001  // код ошибки: слишком большая ошибка положения
#define INIT_POS_CALIBR_ERROR		0x002  // код ошибки: слишком большое отклонение от нуля для выполнения автокалибровки энкодера (POT_diff > MAX_POT_DIFF)
#define ENC_LIMIT_ERROR				0x003	 // код ошибки: попытка движения за угловые пределы сустава
#define NO_ERROR_STOPPED			0x004  // код ошибки: нет ошибки, драйвер остановлен

#define CAN_STOP_ID         0x100   // фрэйм обычной остановки, НЕ включает тормоз и НЕ отключает регуляторы

// команда смена режимов работы драйверов (всех!!!)
// пример: change_mode_id = DRV_MODE_ID | MODE_CODE
#define DRV_MODE_ID     	0x100   // фрэйм смены режима работы драйверов
//#define MODE_STOP	 		0x000   // код режима: остановка <-- НЕ НУЖНО использовать, т.к. есть CAN_STOP_ID
#define MODE_CALIBR	 		0x001   // код режима: калибровка
#define MODE_NORMAL	        0x002   // код режима: норм

// коды команды настройки драйверов
// пример: config_id = DRV_CFG_ID | CFG_CODE | DEV_CODE
#define DRV_CFG_ID          0x200   // фрэйм параметров драйверов
#define CFG_PID_P       		0x001   // код параметров: П-коэффициент
#define CFG_PID_I       		0x002   // код параметров: И-коэффициент
#define CFG_PID_D       		0x003   // код параметров: Д-коэффициент

// код фрэйма с командой(ами) траектории
// пример: trajectory_cmd_id = TRAJ_CMD_ID | POS_VEL_ID | DEV_CODE
#define TRAJ_CMD_ID         0x300   // фрэйм задаваемой траектории

// код фрэймов состояния
// пример: state_id = DRV_STATE_ID | POS_VEL_ | DEV_CODE
#define DRV_STATE_ID        0x400   // фрэйм состояния

// коды для фрэймов состояния или фрэймов с командами траектории
#define MOTOR_POS						0x000
#define MOTOR_VEL						0x001
#define MOTOR_POT_ENC_CUR		0x002   // код фрэйма: данные АЦП потенциометра, энкодера и АЦП датчика тока
#define DRV_IS_CALIBR       0x003   // код фрэйма: драйвер калибруется

// DEV_CODE, коды устройства, которое должно принять или отправляет фрэйм
#define TO_ALL_CODE        	 		0x000   // код фрэйма: всем, ПК
#define DRV_1_CODE           		0x020   // код фрэйма: драйвер 1
#define DRV_2_CODE           		0x040   // код фрэйма: драйвер 2
#define DRV_3_CODE           		0x080   // код фрэйма: драйвер 3

#define DRV_1_MASK_ID						0x0c0
#define DRV_2_MASK_ID						0x0a0
#define DRV_3_MASK_ID						0x060

#endif /* INC_CAN_IDS_H_ */
