/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the impliednaub/
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <errno.h>
#include <time.h>

#include "common/maths.h"

#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/serial.h"
#include "drivers/serial_tcp.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
const timerHardware_t timerHardware[1]; // unused

#include "drivers/accgyro/accgyro_fake.h"
#include "flight/imu.h"

#include "config/feature.h"
#include "fc/config.h"
#include "scheduler/scheduler.h"

#include "pg/rx.h"

#include "rx/rx.h"

#include "dyad.h"
#include "target/SITL/udplink.h"
#include "fcl_sim_proxy.h"

uint32_t SystemCoreClock;

//struct sitl_motor_t sitl_motor;
fcl_motor_t data_motor = {0};

static struct timespec start_time;
static double simRate = 1.0;
static pthread_t tcpWorker;
static bool workerRunning = true;
//static pthread_mutex_t updateLock;
static pthread_mutex_t mainLoopLock;
extern uint32_t do_reset;

int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y);

int lockMainPID(void)
{
    return pthread_mutex_trylock(&mainLoopLock);
}

const int UPDATE_STATE_Y = 1;
const int SEND_MOTOR_Y = 2;
const int SONAR_Y = 3;
const int SIMRATE_Y = 4;

void printfxy(int x, int y, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    printf("\033[%d;%dH", 15 + y, x);
    vprintf(format, args);
    va_end(args);
    fflush(stdout);
}

#define RAD2DEG (180.0 / M_PI)
#define ACC_SCALE (256 / 9.80665)
#define GYRO_SCALE (16.4)

static bool last_reset = false;
static int cc=0;

void sendMotorUpdate()
{
//    static uint32_t i = 0;
    //actuator_state.flags = do_reset;

    if (do_reset != last_reset) {
        printf("\n\n\nRESET %d!!!!!!!!!!!!!!!!!!!\n\n", cc++);
        //if(do_reset)
          //  sitl_reset_world();
    }
    last_reset = do_reset;
    
    //sitl_set_motor(&sitl_motor);
    fcl_send_to_sim(eMotor, &data_motor);

//    printfxy(0, SEND_MOTOR_Y, "<< ros sendMotorUpdate(#%u, %f, %f, %f, %f, %u)\n", i++, sitl_motor.motor[0], sitl_motor.motor[1], sitl_motor.motor[2], sitl_motor.motor[3], 0);
}

void fcl_callback(void* ptr, fclCmd_t data)
{
    if (eFcstate != data) {
        return;
    }

    fcl_fcstate_t state;
    fcl_get_from_sim(eFcstate, &state);

    static double last_timestamp = 0;  // in seconds
    static uint64_t last_realtime = 0; // in uS
    static struct timespec last_ts;    // last packet

    struct timespec now_ts;
    clock_gettime(CLOCK_MONOTONIC, &now_ts);

    //static uint32_t i = 0;


    const uint64_t realtime_now = micros64_real();
    if (realtime_now > last_realtime + 500 * 1e3)
    { // 500ms timeout
        last_timestamp = state.sim_time;
        last_realtime = realtime_now;
        sendMotorUpdate();
        return;
    }

    const double deltaSim = state.sim_time - last_timestamp; // in ms, max step size in gazebo
 //   printfxy(0, UPDATE_STATE_Y, ">> sitl_state_callback(#%u, delta=%f, sim_time=%f)", i++, deltaSim, imu.sim_time);
 //   printfxy(0, SONAR_Y, "> sonar %f", state->sonar.distance);


    if (deltaSim < 0)
    { // don't use old packet
        return;
    }

    int16_t x, y, z;
    x = constrain(-state.imu.linear_acceleration_x * ACC_SCALE, -32767, 32767);
    y = constrain(-state.imu.linear_acceleration_y * ACC_SCALE, -32767, 32767);
    z = constrain(-state.imu.linear_acceleration_z * ACC_SCALE, -32767, 32767);
    fakeAccSet(fakeAccDev, x, y, z);
    //    printf("[acc]%lf,%lf,%lf\n", pkt->imu_linear_acceleration_xyz[0], pkt->imu_linear_acceleration_xyz[1], pkt->imu_linear_acceleration_xyz[2]);

    x = constrain(state.imu.angular_velocity_r * GYRO_SCALE * RAD2DEG, -32767, 32767);
    y = constrain(-state.imu.angular_velocity_p * GYRO_SCALE * RAD2DEG, -32767, 32767);
    z = constrain(-state.imu.angular_velocity_y * GYRO_SCALE * RAD2DEG, -32767, 32767);
    fakeGyroSet(fakeGyroDev, x, y, z);
    //    printf("[gyr]%lf,%lf,%lf\n", pkt->imu_angular_velocity_rpy[0], pkt->imu_angular_velocity_rpy[1], pkt->imu_angular_velocity_rpy[2]);

#if !defined(USE_IMU_CALC)
#if defined(SET_IMU_FROM_EULER)
    // set from Euler
    double qw = get_imu_orientation_quat_w();
    double qx = get_imu_orientation_quat_x();
    double qy = get_imu_orientation_quat_y();
    double qz = get_imu_orientation_quat_z();
    double ysqr = qy * qy;
    double xf, yf, zf;

    // roll (x-axis rotation)
    double t0 = +2.0 * (qw * qx + qy * qz);
    double t1 = +1.0 - 2.0 * (qx * qx + ysqr);
    xf = atan2(t0, t1) * RAD2DEG;

    // pitch (y-axis rotation)
    double t2 = +2.0 * (qw * qy - qz * qx);
    t2 = t2 > 1.0 ? 1.0 : t2;
    t2 = t2 < -1.0 ? -1.0 : t2;
    yf = asin(t2) * RAD2DEG; // from wiki

    // yaw (z-axis rotation)
    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (ysqr + qz * qz);
    zf = atan2(t3, t4) * RAD2DEG;
    imuSetAttitudeRPY(xf, -yf, zf); // yes! pitch was inverted!!
#else
    imuSetAttitudeQuat(
        state.imu.orientation_quat_w
      , state.imu.orientation_quat_x
      , state.imu.orientation_quat_y
      , state.imu.orientation_quat_z);
#endif
#endif

#if defined(SIMULATOR_IMU_SYNC)
    imuSetHasNewData(deltaSim * 1e6);
    imuUpdateAttitude(micros());
#endif


    if (deltaSim < 0.02 && deltaSim > 0)
    {   // simulator should run faster than 50Hz
        //        simRate = simRate * 0.5 + (1e6 * deltaSim / (realtime_now - last_realtime)) * 0.5;
        struct timespec out_ts;
        timeval_sub(&out_ts, &now_ts, &last_ts);
        simRate = deltaSim / (out_ts.tv_sec + 1e-9 * out_ts.tv_nsec);
    }
    
    //    printf("simRate = %lf, millis64 = %lu, millis64_real = %lu, deltaSim = %lf\n", simRate, millis64(), millis64_real(), deltaSim*1e6);
    //printfxy(0, SIMRATE_Y, "simRate=%lf, deltaSim=%lf", simRate, deltaSim);

    last_timestamp = state.sim_time;
    last_realtime = micros64_real();

    last_ts.tv_sec = now_ts.tv_sec;
    last_ts.tv_nsec = now_ts.tv_nsec;

//    pthread_mutex_unlock(&updateLock); // can send PWM output now

#if defined(SIMULATOR_GYROPID_SYNC)
    pthread_mutex_unlock(&mainLoopLock); // can run main loop
#endif
}


static void *tcpThread(void *data)
{
    UNUSED(data);

    dyad_init();
    dyad_setTickInterval(0.2f);
    dyad_setUpdateTimeout(0.5f);

    while (workerRunning)
    {
        dyad_update();
    }

    dyad_shutdown();
    printf("tcpThread end!!\n");
    return NULL;
}

// system
void systemInit(void)
{
    int ret;

    clock_gettime(CLOCK_MONOTONIC, &start_time);
    printf("[system]Init...\n");

    SystemCoreClock = 500 * 1e6; // fake 500MHz
    FLASH_Unlock();

    /*if (pthread_mutex_init(&updateLock, NULL) != 0)
    {
        printf("Create updateLock error!\n");
        exit(1);
    }*/

    if (pthread_mutex_init(&mainLoopLock, NULL) != 0)
    {
        printf("Create mainLoopLock error!\n");
        exit(1);
    }

    ret = pthread_create(&tcpWorker, NULL, tcpThread, NULL);
    if (ret != 0)
    {
        printf("Create tcpWorker error!\n");
        exit(1);
    }

    fcl_init_sim_proxy(&fcl_callback);

    // serial can't been slow down
    rescheduleTask(TASK_SERIAL, 1);
}

void systemReset(void)
{
    printf("[system]Reset!\n");
    workerRunning = false;
    pthread_join(tcpWorker, NULL);
    fcl_deinit_sim_proxy();
    exit(0);
}
void systemResetToBootloader(void)
{
    printf("[system]ResetToBootloader!\n");
    workerRunning = false;
    pthread_join(tcpWorker, NULL);
    fcl_deinit_sim_proxy();
    exit(0);
}

void timerInit(void)
{
    printf("[timer]Init...\n");
}

void timerStart(void)
{
}

void failureMode(failureMode_e mode)
{
    printf("[failureMode]!!! %d\n", mode);
    while (1)
        ;
}

void indicateFailure(failureMode_e mode, int repeatCount)
{
    UNUSED(repeatCount);
    printf("Failure LED flash for: [failureMode]!!! %d\n", mode);
}

// Time part
// Thanks ArduPilot
uint64_t nanos64_real()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1e9 + ts.tv_nsec) - (start_time.tv_sec * 1e9 + start_time.tv_nsec);
}

uint64_t micros64_real()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e6 * ((ts.tv_sec + (ts.tv_nsec * 1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec * 1.0e-9)));
}

uint64_t millis64_real()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return 1.0e3 * ((ts.tv_sec + (ts.tv_nsec * 1.0e-9)) - (start_time.tv_sec + (start_time.tv_nsec * 1.0e-9)));
}

uint64_t micros64()
{
    static uint64_t last = 0;
    static uint64_t out = 0;
    uint64_t now = nanos64_real();

    out += (now - last) * simRate;
    last = now;

    return out * 1e-3;
    //    return micros64_real();
}

uint64_t millis64()
{
    static uint64_t last = 0;
    static uint64_t out = 0;
    uint64_t now = nanos64_real();

    out += (now - last) * simRate;
    last = now;

    return out * 1e-6;
    //    return millis64_real();
}

uint32_t micros(void)
{
    return micros64() & 0xFFFFFFFF;
}

uint32_t millis(void)
{
    return millis64() & 0xFFFFFFFF;
}

void microsleep(uint32_t usec)
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = usec * 1000UL;
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR)
        ;
}

void delayMicroseconds(uint32_t us)
{
    microsleep(us / simRate);
}

void delayMicroseconds_real(uint32_t us)
{
    microsleep(us);
}

void delay(uint32_t ms)
{
    uint64_t start = millis64();

    while ((millis64() - start) < ms)
    {
        microsleep(1000);
    }
}

// Subtract the ‘struct timespec’ values X and Y,  storing the result in RESULT.
// Return 1 if the difference is negative, otherwise 0.
// result = x - y
// from: http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
int timeval_sub(struct timespec *result, struct timespec *x, struct timespec *y)
{
    unsigned int s_carry = 0;
    unsigned int ns_carry = 0;
    // Perform the carry for the later subtraction by updating y.
    if (x->tv_nsec < y->tv_nsec)
    {
        int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
        ns_carry += 1000000000 * nsec;
        s_carry += nsec;
    }

    // Compute the time remaining to wait. tv_usec is certainly positive.
    result->tv_sec = x->tv_sec - y->tv_sec - s_carry;
    result->tv_nsec = x->tv_nsec - y->tv_nsec + ns_carry;

    // Return 1 if result is negative.
    return x->tv_sec < y->tv_sec;
}

// PWM part
static bool pwmMotorsEnabled = false;
static pwmOutputPort_t motors[MAX_SUPPORTED_MOTORS];
static pwmOutputPort_t servos[MAX_SUPPORTED_SERVOS];

// real value to send
static int16_t motorsPwm[MAX_SUPPORTED_MOTORS];
static int16_t servosPwm[MAX_SUPPORTED_SERVOS];
static int16_t idlePulse;

void motorDevInit(const motorDevConfig_t *motorConfig, uint16_t _idlePulse, uint8_t motorCount)
{
    UNUSED(motorConfig);
    UNUSED(motorCount);

    idlePulse = _idlePulse;

    for (int motorIndex = 0; motorIndex < MAX_SUPPORTED_MOTORS && motorIndex < motorCount; motorIndex++)
    {
        motors[motorIndex].enabled = true;
    }
    pwmMotorsEnabled = true;
}

void servoDevInit(const servoDevConfig_t *servoConfig)
{
    UNUSED(servoConfig);
    for (uint8_t servoIndex = 0; servoIndex < MAX_SUPPORTED_SERVOS; servoIndex++)
    {
        servos[servoIndex].enabled = true;
    }
}

pwmOutputPort_t *pwmGetMotors(void)
{
    return motors;
}

void pwmEnableMotors(void)
{
    pwmMotorsEnabled = true;
}

bool pwmAreMotorsEnabled(void)
{
    return pwmMotorsEnabled;
}

bool isMotorProtocolDshot(void)
{
    return false;
}

void pwmWriteMotor(uint8_t index, float value)
{
    motorsPwm[index] = value - idlePulse;
}

void pwmShutdownPulsesForAllMotors(uint8_t motorCount)
{
    UNUSED(motorCount);
    pwmMotorsEnabled = false;
}

void pwmCompleteMotorUpdate(uint8_t motorCount)
{
    UNUSED(motorCount);
    // send to simulator
    // for gazebo8 ArduCopterPlugin remap, normal range = [0.0, 1.0], 3D rang = [-1.0, 1.0]

    double outScale = 1000.0;
    if (featureIsEnabled(FEATURE_3D))
    {
        outScale = 500.0;
    }

    data_motor.motor[3] = motorsPwm[0] / outScale;
    data_motor.motor[0] = motorsPwm[1] / outScale;
    data_motor.motor[1] = motorsPwm[2] / outScale;
    data_motor.motor[2] = motorsPwm[3] / outScale;

    // get one "fdm_packet" can only send one "servo_packet"!!
    //if (pthread_mutex_trylock(&updateLock) != 0) return;
    sendMotorUpdate();
    //udpSend(&pwmLink, &pwmPkt, sizeof(servo_packet));
    //printf("[pwm]%u:%u,%u,%u,%u\n", idlePulse, motorsPwm[0], motorsPwm[1], motorsPwm[2], motorsPwm[3]);
    //printfxy(0,11,"[pwmPkt_a]%f,%f,%f,%f\n", actuator_state.motor[0], actuator_state.motor[1], actuator_state.motor[2], actuator_state.motor[3]);
}

void pwmWriteServo(uint8_t index, float value)
{
    servosPwm[index] = value;
}

// ADC part
uint16_t adcGetChannel(uint8_t channel)
{
    UNUSED(channel);
    return 0;
}

// stack part
char _estack;
char _Min_Stack_Size;

// fake EEPROM
static FILE *eepromFd = NULL;
uint8_t eepromData[EEPROM_SIZE];

void FLASH_Unlock(void)
{
    if (eepromFd != NULL)
    {
        fprintf(stderr, "[FLASH_Unlock] eepromFd != NULL\n");
        return;
    }

    // open or create
    eepromFd = fopen(EEPROM_FILENAME, "r+");
    if (eepromFd != NULL)
    {
        // obtain file size:
        fseek(eepromFd, 0, SEEK_END);
        size_t lSize = ftell(eepromFd);
        rewind(eepromFd);

        size_t n = fread(eepromData, 1, sizeof(eepromData), eepromFd);
        if (n == lSize)
        {
            printf("[FLASH_Unlock] loaded '%s', size = %ld / %ld\n", EEPROM_FILENAME, lSize, sizeof(eepromData));
        }
        else
        {
            fprintf(stderr, "[FLASH_Unlock] failed to load '%s'\n", EEPROM_FILENAME);
            return;
        }
    }
    else
    {
        printf("[FLASH_Unlock] created '%s', size = %ld\n", EEPROM_FILENAME, sizeof(eepromData));
        if ((eepromFd = fopen(EEPROM_FILENAME, "w+")) == NULL)
        {
            fprintf(stderr, "[FLASH_Unlock] failed to create '%s'\n", EEPROM_FILENAME);
            return;
        }
        if (fwrite(eepromData, sizeof(eepromData), 1, eepromFd) != 1)
        {
            fprintf(stderr, "[FLASH_Unlock] write failed: %s\n", strerror(errno));
        }
    }
}

void FLASH_Lock(void)
{
    // flush & close
    if (eepromFd != NULL)
    {
        fseek(eepromFd, 0, SEEK_SET);
        fwrite(eepromData, 1, sizeof(eepromData), eepromFd);
        fclose(eepromFd);
        eepromFd = NULL;
        printf("[FLASH_Lock] saved '%s'\n", EEPROM_FILENAME);
    }
    else
    {
        fprintf(stderr, "[FLASH_Lock] eeprom is not unlocked\n");
    }
}

FLASH_Status FLASH_ErasePage(uintptr_t Page_Address)
{
    UNUSED(Page_Address);
    //    printf("[FLASH_ErasePage]%x\n", Page_Address);
    return FLASH_COMPLETE;
}

FLASH_Status FLASH_ProgramWord(uintptr_t addr, uint32_t value)
{
    if ((addr >= (uintptr_t)eepromData) && (addr < (uintptr_t)ARRAYEND(eepromData)))
    {
        *((uint32_t *)addr) = value;
        printf("[FLASH_ProgramWord]%p = %08x\n", (void *)addr, *((uint32_t *)addr));
    }
    else
    {
        printf("[FLASH_ProgramWord]%p out of range!\n", (void *)addr);
    }
    return FLASH_COMPLETE;
}

void uartPinConfigure(const serialPinConfig_t *pSerialPinConfig)
{
    UNUSED(pSerialPinConfig);
    printf("uartPinConfigure");
}

void spektrumBind(rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);
    printf("spektrumBind");
}
