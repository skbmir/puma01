#pragma once

// ПРИОРИТЕТЫ фрэймов: 
// 1 - экстренная остановка			0x000
// 1 - sync, heartbeat              0x00f
// 	2 - ошибка						0x0nm, где n - код уст-ва (0 = ПК, 1-6 = драйверы), а m - код ошибки
// 	2 - остановка					0x100
// 	2 - переключение режима работы  0x1fm, где m - код режима работы
// 		3 - параметры ПИД 			0x2nm, где n - код уст-ва (1-6 = драйверы), а m - код параметра (0-f = П, И, Д и др.)
// 		3 - команды траектории		0x3nm, где n - код уст-ва (1-6 = драйверы), m - код данных (1 = положение и скорость, 0 = момент и ускорение)
//  			4 - телеметрия      0x4nm, где n - код уст-ва (1-6 = драйверы), m - код данных (2 = положение энк и скорость, 1 = положение пот, 0 = данные датчика тока)
 			

#define EMERGENCY_ID            0x000   // фрэйм экстренной остановки
#define HEARTBEAT_ID            0x00f   // фрэйм синхронизации

// пример: error_id = ERROR_CODE_ID | DEV_ID
// #define ERROR_ID             0x000   // фрэйм ошибки, не имеет значения
#define ERROR_CODE_ID           0x001   // код ошибки, пример

#define STOP_ID                 0x100   // фрэйм обычной остановки

// пример: change_mode_id = CHANGE_MODE_ID | MODE_CODE_ID
#define CHANGE_MODE_ID          0x1f0   // фрэйм смены режима работы драйверов
#define MODE_CODE_ID            0x001   // код режима работы, пример

// пример: config_id = CONFIG_ID | CONFIG_CODE_ID | DEV_ID
#define CONFIG_ID               0x200   // фрэйм параметров драйверов
#define CONFIG_CODE_ID          0x001   // код параметра, пример 

// пример: trajectory_id = TRAJECTORY_ID | POS_VEL_ID | MOTOR_1_ID
#define TRAJECTORY_ID           0x300   // фрэйм задаваемой траектории

// пример: elemetry_id = TELEMETRY_ID | POS_VEL_ID | MOTOR_1_ID
#define TELEMETRY_ID            0x700   // фрэйм телеметрии

#define POS_VEL_ID              0x000   // код команды: положение и скорость (энкодер)
#define ACC_TORQ_ID             0x001   // код команды: ускорение и момент
#define POT_ID                  0x002   // код команды: положение (потенциометр)
#define MOTOR_CURRENT_ID        0x003   // код команды: ток в обмотке мотора

// DEV_ID = ID устройства, которое отправляет команду
#define TO_ALL_ID               0x000   // код устройства: всем
#define DRIVER_1_ID             0x010   // код устройства: драйвер 1
#define DRIVER_2_ID             0x020   // код устройства: драйвер 2
#define DRIVER_3_ID             0x030   // код устройства: драйвер 3
#define DRIVER_4_ID             0x040   // код устройства: драйвер 4 
#define DRIVER_5_ID             0x050   // код устройства: драйвер 5
#define DRIVER_6_ID             0x060   // код устройства: драйвер 6
