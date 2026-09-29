/* ******************************************************************************************************** */
// by Alfin Junaedy, Kubota Lab - TMU, June 2026
// make clean && make
    // i2cdetect -y 1
    // i2cdetect -y 3
    // sudo systemctl restart pigpiod
/* ******************************************************************************************************** */

#include <asm/termbits.h>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <linux/i2c-dev.h>
#include <pigpiod_if2.h>
#include <stdint.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <wiringPiSPI.h>
#include <wiringSerial.h>

#include "ground_effect.hpp"
#include "ndo_leso.hpp"

extern "C" {
#include "vl53l5cx_api.h"
}

using namespace	std;
using namespace	std::chrono;
using clock0 = std::chrono::steady_clock;
using us0 = std::chrono::microseconds;

/* --------------------------------------------------------------- */
#define PIN_WPI_BUTF        26
#define PIN_WPI_BUTR        24
#define PIN_WPI_BUZZ        29
#define PIN_WPI_LEDR        4
#define PIN_WPI_LEDG        5
#define PIN_WPI_LEDB        6
#define PIN_WPI_LEDFLASH1   27
#define PIN_BCM_RX_OF       6
#define I2C_DEV_3           "/dev/i2c-3"
#define MUX_I2C_ADDR        0x70
#define ToF8x8_ADDR         0x29
#define ToF1_MUX_CH         2
#define ToF_FPS             30
#define I2C_DEV_1           "/dev/i2c-1"
#define BNO055_ADDR         0x28
#define ADS1115_ADDR        0x48
static constexpr uint8_t  CRSF_ADDR                 = 0xC8;
static constexpr uint8_t  CRSF_TYPE                 = 0x16;
static constexpr uint32_t CRSF_BAUD                 = 420000;
static constexpr const char* FC_SER_PORT            = "/dev/serial0";
static constexpr uint16_t MSP2_SENSOR_RANGEFINDER   = 0x1F01;
static constexpr uint16_t MSP2_SENSOR_OPTIC_FLOW    = 0x1F02;
static constexpr int BAUD_SOFT_OF                   = 115200;
static constexpr uint16_t MAX_OF_PAYLOAD            = 256;
static const string LOG_DIR                         = (filesystem::current_path() / "log").string() + "/";
static const string LOG_FILE                        = "dctllog";
static const float OFF_ROLL_CMD                     = -45;      // roll+=right (1000~2000)
static const float OFF_PITCH_CMD                    = -40;      // pitch+=forward (1000~2000)
static const float PX_SCALE_OF                      = 0.02f;    // (1/scale_ori)*(mea_real/mea_out) 
static const float HOVER_ALT_M                      = 0.3f;
static const float EMGC_LAND_BATT                   = 20.0f;    // volt
static const float EMGC_LAND_ALT                    = 1.65f;    // m
static const float EMGC_STOP_LINVEL                 = 1.2f;     // m/s
static const float EMGC_STOP_TILT                   = 25.0f;    // deg
static const float EMGC_STOP_ALT                    = 1.95f;    // m
static const float EMGC_STOP_ANGVEL                 = 180.0f;   // deg/s
static const int N_MAX_ODOM                         = 100;
static const float ODOM_LINVEL_TO                   = 0.1f;     // m/s
static const float ODOM_ANGVEL_TO                   = 15.0f;    // deg/s
static const int GE_state                           = 4;        // 1=PID 2=PID+scalar 3=PID+NDO 4=PID+GE
/* --------------------------------------------------------------- */

VL53L5CX_Configuration DevToF;
int fd_crsf_fc, fd_msp2_of, fd_i2c_1, fd_i2c_3;

float rpy_enc_deg[3], pos_ned[3], pos_neu[3], vel_ned[3], vel_rpy[3], flightTime=0;
int16_t adc_raw; float adc_volt;
float min_batt_volt=999, alt_of_m, flow_of_x, flow_of_y;
float tof1_m[64]={0}, tof1_avg_m=0, rc_cmd[16]={0}, tof1_single_m = 0;
const float emaFPS=0.25f, emaSYScpu=0.1f; float emaSYScpuVal[3]={0}; int cpuRAMtmp[3]={0};
float fps_main_th[3]={0}, fps_imu=0, fps_alt=0, fps_tof1=0, fps_adc=0, fps_of=0; 
int cnt_main_th[3]={0}, cnt_imu=0, cnt_alt=0, cnt_tof1=0, cnt_adc=0;
bool init_ready=false, init_stop=false, print_debug=false, startDctlLog=false, read_of=false;
int nDctlLog=0, dctlStages=0; float dctlLog150[150]={0};
bool startMission=false, z_ready=false, xy_ready=false;
int emgcCode=0, nOdom=0, mOdom=0, mOdomCur=0;
float odomGoal_NEU_CCW[N_MAX_ODOM][7]={0}, posTgtNeuDctl[4]={0}, posErrNeuDctl[4]={0}, ge_TRPY[8]={0};
bool SLOPE_START = false, gestart = false;
static float last_applied_pwm_ndo = 1500.0f;

PI_THREAD(th1_100);
PI_THREAD(th2_15);
void sigint_handler(int sig) {
    printf("\nForced close ... %d\n", sig);
    vl53l5cx_stop_ranging(&DevToF);
    if (fd_i2c_1 >= 0) { close(fd_i2c_1); fd_i2c_1 = -1; }
    if (fd_i2c_3 >= 0) { close(fd_i2c_3); fd_i2c_3 = -1; }
    if (fd_msp2_of >= 0) { bb_serial_read_close(fd_msp2_of, PIN_BCM_RX_OF); pigpio_stop(fd_msp2_of); fd_msp2_of = -1; } 
    if (fd_crsf_fc >= 0) { close(fd_crsf_fc); fd_crsf_fc = -1; }
    digitalWrite(PIN_WPI_BUZZ, 0);
    digitalWrite(PIN_WPI_LEDR, 0);
    digitalWrite(PIN_WPI_LEDG, 0);
    digitalWrite(PIN_WPI_LEDB, 0);
    digitalWrite(PIN_WPI_LEDFLASH1, 0);
    std::exit(0);
}

void set_buz_led(int buzz=0, int ledr=0, int ledg=0, int ledb=0, int ledf=0) {
    if (buzz >= 0) digitalWrite(PIN_WPI_BUZZ, buzz);
    if (ledr >= 0) digitalWrite(PIN_WPI_LEDR, ledr);
    if (ledg >= 0) digitalWrite(PIN_WPI_LEDG, ledg);
    if (ledb >= 0) digitalWrite(PIN_WPI_LEDB, ledb);
    if (ledf >= 0) digitalWrite(PIN_WPI_LEDFLASH1, ledf);
}
void getCPUramTmp(void) {
	try { // 0.2-1 ms
		// CPU usage %
		static unsigned int lastTotalUser, lastTotalUserLow, lastTotalSys, lastTotalIdle;
		unsigned long long totalUser, totalUserLow, totalSys, totalIdle, total;
		double percent;
		FILE* file;
		file = fopen("/proc/stat", "r");
		fscanf(file, "cpu %llu %llu %llu %llu", &totalUser, &totalUserLow, &totalSys, &totalIdle);
		fclose(file);
		if (totalUser < lastTotalUser || totalUserLow < lastTotalUserLow || totalSys < lastTotalSys || totalIdle < lastTotalIdle) { percent = -1.0; }
		else {
			total = (totalUser - lastTotalUser) + (totalUserLow - lastTotalUserLow) + (totalSys - lastTotalSys);
			percent = total * 100;
			total += (totalIdle - lastTotalIdle);
			percent /= total;
		}
		lastTotalUser = totalUser;
		lastTotalUserLow = totalUserLow;
		lastTotalSys = totalSys;
		lastTotalIdle = totalIdle;
		if (isnanf(percent) || isinf(percent) || percent <= 0) percent = cpuRAMtmp[0];
		else if (percent > 100) percent = 100;
		emaSYScpuVal[0] = (emaSYScpu * (float)percent) + ((1 - emaSYScpu) * emaSYScpuVal[0]);
		cpuRAMtmp[0] = round(emaSYScpuVal[0]);
		// MEM usage %
		ifstream meminfoFile("/proc/meminfo");
		string lineMem;
		unsigned long long totalMemory = 0;
		unsigned long long freeMemory = 0;
		while (getline(meminfoFile, lineMem)) {
			if (lineMem.find("MemTotal:") != string::npos) {
				istringstream iss(lineMem);
				iss.ignore(256, ':');
				iss >> totalMemory;
			}
			else if (lineMem.find("MemFree:") != string::npos) {
				istringstream iss(lineMem);
				iss.ignore(256, ':');
				iss >> freeMemory;
			}
		}
		unsigned long long ram = (long)round((float)(totalMemory - freeMemory) / 1024);
		if (isnan(ram) || isinf(ram) || ram <= 0) ram = cpuRAMtmp[1];
		else if (ram > 512) ram = 512;
		emaSYScpuVal[2] = (emaSYScpu * (float)ram) + ((1 - emaSYScpu) * emaSYScpuVal[2]); //MB
		cpuRAMtmp[1] = round(emaSYScpuVal[2] / 425.0 * 100.0); //MB to percent
		//CPU temp [degC]
		ifstream tempFile("/sys/class/thermal/thermal_zone0/temp");
		double temperature = 0.0;
		if (tempFile.is_open()) {
			string line;
			getline(tempFile, line);
			temperature = stod(line) / 1000.0;
			tempFile.close();
		}
		if (isnanf(temperature) || isinf(temperature) || temperature <= 0) temperature = cpuRAMtmp[2];
		else if (temperature > 100) temperature = 100;
		emaSYScpuVal[1] = (emaSYScpu * (float)temperature) + ((1 - emaSYScpu) * emaSYScpuVal[1]);
		cpuRAMtmp[2] = round(emaSYScpuVal[1]);
	}
	catch (const std::exception&) {
		cout << "Err reading CPU stats" << endl;
	}
}

static int open_ser_crsf_fc(const char *dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) return -1;
    termios2 tty{};
    if (ioctl(fd, TCGETS2, &tty) < 0) return -1;
    tty.c_cflag &= ~CBAUD;
    tty.c_cflag |= BOTHER;
    tty.c_ispeed = CRSF_BAUD;
    tty.c_ospeed = CRSF_BAUD;
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_iflag = tty.c_oflag = tty.c_lflag = 0;
    if (ioctl(fd, TCSETS2, &tty) < 0) return -1;
    if (ioctl(fd, TCFLSH, TCIOFLUSH) < 0) return -1;
    return fd;
}
static int open_ser_msp2_of(void) {
    int fd = pigpio_start(nullptr, nullptr);
    if (fd < 0) return -1;
    if (bb_serial_read_open(fd, PIN_BCM_RX_OF, BAUD_SOFT_OF, 8) != 0) { pigpio_stop(fd); return -1; }
    return fd;
}
static int imu_init(int fd) {
    uint8_t config[2] = {0x3D, 0x0C};
    if (ioctl(fd, I2C_SLAVE, BNO055_ADDR) < 0) return -1;
    if (write(fd, config, 2) < 0) return -1;
    return 0;
}
static int adc_init(int fd) {
    uint8_t config[3] = {0x01,0xC2,0x83};
    if (ioctl(fd, I2C_SLAVE, ADS1115_ADDR) < 0) return -1;
    if (write(fd, config, 3) < 0) return -1;
    return 0;
}
static int tof_init(int fd) {
    DevToF.platform.fd = fd;
    DevToF.platform.address = ToF8x8_ADDR;
    // select mux ch
    uint8_t data = (1 << ToF1_MUX_CH); 
    if (ioctl(DevToF.platform.fd, I2C_SLAVE, MUX_I2C_ADDR) < 0) return -1;
    if (write(DevToF.platform.fd, &data, 1) != 1) return -1;
    usleep(5000);
    // init
    if (ioctl(DevToF.platform.fd, I2C_SLAVE, DevToF.platform.address) < 0) return -1;
    uint8_t isAlive = 0, status;
    status = vl53l5cx_is_alive(&DevToF, &isAlive);
    if (!isAlive || status) return -1;
    status = vl53l5cx_init(&DevToF);
    if (status) return -1;
    vl53l5cx_set_resolution(&DevToF, VL53L5CX_RESOLUTION_8X8);
    vl53l5cx_set_ranging_frequency_hz(&DevToF, ToF_FPS);
    vl53l5cx_start_ranging(&DevToF);
    return 0;
}

static int build_crsf_frame(uint8_t *out, uint16_t ch_us[16]) {
    uint8_t payload[22] = {};
    for (int i = 0; i < 16; i++) {
        if (ch_us[i] < 1000) ch_us[i] = 1000;
        if (ch_us[i] > 2000) ch_us[i] = 2000;
        ch_us[i] = 172 + (ch_us[i] - 1000) * (1811 - 172) / 1000;
        //---
        uint16_t v = ch_us[i] & 0x07FF;
        int bit = i * 11;
        int b = bit / 8;
        int s = bit % 8;
        payload[b]     |= (v << s);
        payload[b + 1] |= (v >> (8 - s));
        if (s > 5) payload[b + 2] |= (v >> (16 - s));
    }
    out[0] = CRSF_ADDR;
    out[1] = 24;
    out[2] = CRSF_TYPE;
    memcpy(out + 3, payload, 22);
    const uint8_t *buf = out+2; size_t len = 23; uint8_t crc = 0;
    while (len--) {
        crc ^= *buf++;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1);
    }
    out[25] = crc;
    return 26;
}
static uint8_t crc8_dvb_s2(uint8_t crc, uint8_t b) {
    crc ^= b;
    for (int i = 0; i < 8; i++) { crc = (crc & 0x80) ? (crc << 1) ^ 0xD5 : (crc << 1); }
    return crc;
}
struct MSPParser {
    enum State {
        S_IDLE, S_X, S_DIR, S_FLAG,
        S_FN1, S_FN2, S_LEN1, S_LEN2,
        S_PAYLOAD, S_CRC
    };
    State    state = S_IDLE;
    uint16_t fn    = 0;
    uint16_t len   = 0;
    uint16_t idx   = 0;
    uint8_t  crc   = 0;
    uint8_t  payload[MAX_OF_PAYLOAD];
    void reset() { state = S_IDLE; crc = 0; idx = 0; }
    bool feed(uint8_t b, uint16_t &out_fn, uint8_t *&out_payload, uint16_t &out_len) {
        switch (state) {
            case S_IDLE:
                if (b == '$') state = S_X;
                break;
            case S_X:
                if (b == 'X') state = S_DIR;
                else reset();
                break;
            case S_DIR:
                if (b == '<') { crc = 0; state = S_FLAG; }
                else reset();
                break;
            case S_FLAG:
                crc = crc8_dvb_s2(crc, b);
                state = S_FN1;
                break;
            case S_FN1:
                fn  = b;
                crc = crc8_dvb_s2(crc, b);
                state = S_FN2;
                break;
            case S_FN2:
                fn |= ((uint16_t)b << 8);
                crc = crc8_dvb_s2(crc, b);
                state = S_LEN1;
                break;
            case S_LEN1:
                len = b;
                crc = crc8_dvb_s2(crc, b);
                state = S_LEN2;
                break;
            case S_LEN2:
                len |= ((uint16_t)b << 8);
                crc  = crc8_dvb_s2(crc, b);
                if (len > MAX_OF_PAYLOAD) { reset(); break; }
                idx   = 0;
                state = (len == 0) ? S_CRC : S_PAYLOAD;
                break;
            case S_PAYLOAD:
                payload[idx++] = b;
                crc = crc8_dvb_s2(crc, b);
                if (idx >= len) state = S_CRC;
                break;
            case S_CRC:
                if (b != crc) { reset(); break; }
                out_fn      = fn;
                out_payload = payload;
                out_len     = len;
                reset();
                return true;
        }
        return false;
    }
};

static int odomHover(float this_delay_sec = 2) {
    if (nOdom < N_MAX_ODOM - 1) {
        odomGoal_NEU_CCW[nOdom][0] = 0; // x
        odomGoal_NEU_CCW[nOdom][1] = 0; // y
        odomGoal_NEU_CCW[nOdom][2] = 0; // z
        odomGoal_NEU_CCW[nOdom][3] = 0; // yaw
        odomGoal_NEU_CCW[nOdom][4] = 0; // flag
        odomGoal_NEU_CCW[nOdom][5] = -1; // timeout
        odomGoal_NEU_CCW[nOdom][6] = this_delay_sec; // delay
        nOdom++;
    }
    else return -1;
    return 0;
}
static int odomMoveIncWorldX(float mtrX = 0, float this_delay_sec = 2) {
    if (nOdom < N_MAX_ODOM - 1) {
        odomGoal_NEU_CCW[nOdom][0] = mtrX; // x
        odomGoal_NEU_CCW[nOdom][1] = 0; // y
        odomGoal_NEU_CCW[nOdom][2] = 0; // z
        odomGoal_NEU_CCW[nOdom][3] = 0; // yaw
        if (mtrX >= 0.0f) odomGoal_NEU_CCW[nOdom][4] = 1; // flag
        else odomGoal_NEU_CCW[nOdom][4] = 2; // flag
        odomGoal_NEU_CCW[nOdom][5] = abs(mtrX / ODOM_LINVEL_TO); // timeout
        odomGoal_NEU_CCW[nOdom][6] = this_delay_sec; // delay
        nOdom++;
    }
    else return -1;
    return 0;
}
static int odomMoveIncWorldY(float mtrY = 0, float this_delay_sec = 2) {
    if (nOdom < N_MAX_ODOM - 1) {
        odomGoal_NEU_CCW[nOdom][0] = 0; // x
        odomGoal_NEU_CCW[nOdom][1] = mtrY; // y
        odomGoal_NEU_CCW[nOdom][2] = 0; // z
        odomGoal_NEU_CCW[nOdom][3] = 0; // yaw
        if (mtrY >= 0.0f) odomGoal_NEU_CCW[nOdom][4] = 3; // flag
        else odomGoal_NEU_CCW[nOdom][4] = 4; // flag
        odomGoal_NEU_CCW[nOdom][5] = abs(mtrY / ODOM_LINVEL_TO); // timeout
        odomGoal_NEU_CCW[nOdom][6] = this_delay_sec; // delay
        nOdom++;
    }
    else return -1;
    return 0;
}
static int odomMoveIncWorldZ(float mtrZ = 0, float this_delay_sec = 2) {
    if (nOdom < N_MAX_ODOM - 1) {
        odomGoal_NEU_CCW[nOdom][0] = 0; // x
        odomGoal_NEU_CCW[nOdom][1] = 0; // y
        odomGoal_NEU_CCW[nOdom][2] = mtrZ; // z
        odomGoal_NEU_CCW[nOdom][3] = 0; // yaw
        if (mtrZ >= 0.0f) odomGoal_NEU_CCW[nOdom][4] = 5; // flag
        else odomGoal_NEU_CCW[nOdom][4] = 6; // flag
        odomGoal_NEU_CCW[nOdom][5] = abs(mtrZ / ODOM_LINVEL_TO); // timeout
        odomGoal_NEU_CCW[nOdom][6] = this_delay_sec; // delay
        nOdom++;
    }
    else return -1;
    return 0;
}
static int odomRotIncWorldYaw(float degYaw = 0, float this_delay_sec = 2) {
    if (nOdom < N_MAX_ODOM - 1) {
        odomGoal_NEU_CCW[nOdom][0] = 0; // x
        odomGoal_NEU_CCW[nOdom][1] = 0; // y
        odomGoal_NEU_CCW[nOdom][2] = 0; // z
        odomGoal_NEU_CCW[nOdom][3] = degYaw; // yaw
        if (degYaw >= 0.0f) odomGoal_NEU_CCW[nOdom][4] = 7; // flag
        else odomGoal_NEU_CCW[nOdom][4] = 8; // flag
        odomGoal_NEU_CCW[nOdom][5] = abs(degYaw / ODOM_ANGVEL_TO); // timeout
        odomGoal_NEU_CCW[nOdom][6] = this_delay_sec; // delay
        nOdom++;
    }
    else return -1;
    return 0;
}

PI_THREAD(th1_100) {
    const us0 PERIODus(10000); // 100Hz, sensors (IMU+ADC+OF-Alt) + posEst + log
    auto nextP = clock0::now() + PERIODus;
    auto readBytes_bno055 = [&](int fd, uint8_t reg, uint8_t* buf, int len) { write(fd, &reg, 1); read(fd, buf, len); };
    auto le32s = [](const uint8_t *p) -> int32_t { return (int32_t)(((uint32_t)p[0]) | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24)); };
    auto t0a = high_resolution_clock::now(), t1a=t0a;
    auto ticFPSsec = [&]() { t0a = high_resolution_clock::now(); };
    auto tocFPSsec = [&]() { return (float)(high_resolution_clock::now() - t0a).count() / 1000000000; };
    auto ticDTsec = [&]() { t1a = high_resolution_clock::now(); };
    auto tocDTsec = [&]() { return (float)(high_resolution_clock::now() - t1a).count() / 1000000000; };
    auto sindeg = [&](float deg) { return sin((deg * M_PI) / 180.0); };
	auto cosdeg = [&](float deg) { return cos((deg * M_PI) / 180.0); };
	auto tandeg = [&](float deg) { return tan((deg * M_PI) / 180.0); };
	auto atandeg = [&](float y, float x) { return 180.0 * atan2(y, x) / M_PI; }; //+-180deg
    auto angleDiff = [](float now, float prev) -> float {
        float d = now - prev;
        if (d >  180.f) d -= 360.f;
        if (d < -180.f) d += 360.f;
        return d;
    };
    auto exportDctlLog = [&](bool isNew) {
        while (isNew) {
			string filename = LOG_DIR + LOG_FILE + to_string(nDctlLog) + ".bin";
			ifstream fileCheck(filename, ios::binary | ios::ate);
			if (fileCheck.is_open()) {
				if (fileCheck.tellg() > 0) {
					fileCheck.close();
					nDctlLog++;
				}
				else {
					fileCheck.close();
					ofstream binFile(filename, std::ios::binary);
					binFile.write(reinterpret_cast<const char*>(dctlLog150), sizeof(dctlLog150));
					binFile.close();
					break;
				}
			}
			else {
				fileCheck.close();
				ofstream binFile(filename, std::ios::binary);
				binFile.write(reinterpret_cast<const char*>(dctlLog150), sizeof(dctlLog150));
				binFile.close();
				break;
			}
		}
		if (!isNew) {
			string filename = LOG_DIR + LOG_FILE + to_string(nDctlLog) + ".bin";
			ofstream binFile(filename, ios::binary | ios::app);
			if (binFile.is_open()) {
				binFile.write(reinterpret_cast<const char*>(dctlLog150), sizeof(dctlLog150));
				binFile.close();
			}
			else { cout << "Error writing [" << filename << "]" << endl; }
		}
    };

    const float emaRPY[2]={0.75f,0.55f}, emaRPYn=0.0001f, filtIMU=7.5f; float emaRPYval[6]={0}, emaRPYval2[6]={0};
    const float emaFLOW[2]={0.85f,0.95f}; float emaFLOWval[4]={0};
    const float emaADC[2]={0.1f,0.075f}; float emaADCval[2]={24.2f,24.2f};
    const float emaXY[2]={0.7f,1.0f}; float emaXYbval[4]={0}, emaXYwval[4]={0};
    const float emaVRPY[2]={0.45f,0.15f}; float emaVRPYval[6]={0}, prev_vrpy[3]={0};
    const float emaVNED[2]={0.75f,0.85f}; float emaVNEDval[4]={0}, prev_ned[2]={0};
    bool firstLog = true, firstIMU = true; 
    float prev_rpy[3], prev0_rpy[3], offset_rpy[3]={0}, rpy_tmp_deg[3];
    float tempFPS, tmpXYbody[2]={0}, tmpXYworld[2]={0}, prevXYoff[2]={0}, prev_rpy_out[3]={0};
    float dt0_of, dt1_of, dt0_vrpy, dt1_vrpy;
    int badIMUn = 0;
    uint8_t reg_adc = 0x00, data_adc[2];

    MSPParser parser_of;
    uint8_t buf_of[4096], *payload_of, quality_of;
    uint16_t fn_of, len_of;
    int count_of = 0;

    while (!init_ready || init_stop) { if (!digitalRead(PIN_WPI_BUTF) || !digitalRead(PIN_WPI_BUTR)) { init_stop = true; } delay(100); }
    cout << ">>> [OK] th1 (100)" << endl;
    ticFPSsec(); ticDTsec();
    dt0_of = tocDTsec(); dt0_vrpy = tocDTsec();

    while (true) {
        // IMU BNO055
        ioctl(fd_i2c_1, I2C_SLAVE, BNO055_ADDR);
        uint8_t eul[6]; readBytes_bno055(fd_i2c_1, 0x1A, eul, 6);
        float yaw0   = -(float)((int16_t)(eul[1] << 8 | eul[0])) / 16.0f, yaw=yaw0;
        float roll0  = -(float)((int16_t)(eul[3] << 8 | eul[2])) / 16.0f, roll=roll0;
        float pitch0 = (float)((int16_t)(eul[5] << 8 | eul[4])) / 16.0f, pitch=pitch0;
        if (abs(roll - prev0_rpy[0]) > filtIMU) roll = roll = (emaRPYn*roll) + ((1 - emaRPYn) * prev_rpy[0]);
        if (abs(pitch - prev0_rpy[1]) > filtIMU) pitch = (emaRPYn*pitch) + ((1 - emaRPYn) * prev_rpy[1]);
        if (abs(yaw - prev0_rpy[2]) > 360 || (abs(yaw - prev0_rpy[2]) > filtIMU && abs(yaw - prev0_rpy[2]) < (360-filtIMU))) yaw = (emaRPYn*yaw) + ((1 - emaRPYn) * prev_rpy[2]);
        prev0_rpy[0] = roll0;   prev0_rpy[1] = pitch0;  prev0_rpy[2] = yaw0;
        prev_rpy[0] = roll;     prev_rpy[1] = pitch;    prev_rpy[2] = yaw;
        if (roll > 180.0f) roll -= 360; else if (roll < -180.0f) roll += 360;
        if (pitch > 180.0f) pitch -= 360; else if (pitch < -180.0f) pitch += 360;
        if (yaw > 180.0f) yaw -= 360; else if (yaw < -180.0f) yaw += 360;
        emaRPYval[0] = (emaRPY[0] * roll) +         ((1 - emaRPY[0]) * emaRPYval[0]);
        emaRPYval[1] = (emaRPY[1] * emaRPYval[0]) + ((1 - emaRPY[1]) * emaRPYval[1]);
        emaRPYval[2] = (emaRPY[0] * pitch) +        ((1 - emaRPY[0]) * emaRPYval[2]);
        emaRPYval[3] = (emaRPY[1] * emaRPYval[2]) + ((1 - emaRPY[1]) * emaRPYval[3]);
        emaRPYval[4] = (emaRPY[0] * yaw) +          ((1 - emaRPY[0]) * emaRPYval[4]);
        emaRPYval[5] = (emaRPY[1] * emaRPYval[4]) + ((1 - emaRPY[1]) * emaRPYval[5]);
        rpy_tmp_deg[0] = emaRPYval[1];// - offset_rpy[0];
        rpy_tmp_deg[1] = emaRPYval[3];// - offset_rpy[1];
        rpy_tmp_deg[2] = emaRPYval[5] - offset_rpy[2];
        cnt_imu++;
        if (fps_imu>50 && fps_imu<150 && abs(roll)<=360 && abs(pitch)<=360 && abs(yaw)<=360) {
            if (firstIMU) { 
                firstIMU = false; 
                memcpy(offset_rpy, rpy_tmp_deg, sizeof(offset_rpy));
            }
            else if (badIMUn < 10) {
                if (abs(rpy_tmp_deg[0])>1.0f || abs(rpy_tmp_deg[1])>1.0f || abs(rpy_tmp_deg[2])>1.0f) { firstIMU = true; badIMUn++; }
                else badIMUn = 999;
            }
        }
        if (rpy_tmp_deg[0] > 180.0f) rpy_tmp_deg[0] -= 360; else if (rpy_tmp_deg[0] < -180.0f) rpy_tmp_deg[0] += 360;
        if (rpy_tmp_deg[1] > 180.0f) rpy_tmp_deg[1] -= 360; else if (rpy_tmp_deg[1] < -180.0f) rpy_tmp_deg[1] += 360;
        if (rpy_tmp_deg[2] > 180.0f) rpy_tmp_deg[2] -= 360; else if (rpy_tmp_deg[2] < -180.0f) rpy_tmp_deg[2] += 360;
        emaRPYval2[0] = (emaRPY[0] * rpy_tmp_deg[0]) + ((1 - emaRPY[0]) * emaRPYval2[0]);
        emaRPYval2[1] = (emaRPY[1] * emaRPYval2[0]) +  ((1 - emaRPY[1]) * emaRPYval2[1]);
        emaRPYval2[2] = (emaRPY[0] * rpy_tmp_deg[1]) + ((1 - emaRPY[0]) * emaRPYval2[2]);
        emaRPYval2[3] = (emaRPY[1] * emaRPYval2[2]) +  ((1 - emaRPY[1]) * emaRPYval2[3]);
        emaRPYval2[4] = (emaRPY[0] * rpy_tmp_deg[2]) + ((1 - emaRPY[0]) * emaRPYval2[4]);
        emaRPYval2[5] = (emaRPY[1] * emaRPYval2[4]) +  ((1 - emaRPY[1]) * emaRPYval2[5]);
        if (emaRPYval2[0] > 180.0f) emaRPYval2[0] -= 360; else if (emaRPYval2[0] < -180.0f) emaRPYval2[0] += 360;
        if (emaRPYval2[1] > 180.0f) emaRPYval2[1] -= 360; else if (emaRPYval2[1] < -180.0f) emaRPYval2[1] += 360;
        if (emaRPYval2[2] > 180.0f) emaRPYval2[2] -= 360; else if (emaRPYval2[2] < -180.0f) emaRPYval2[2] += 360;
        if (emaRPYval2[3] > 180.0f) emaRPYval2[3] -= 360; else if (emaRPYval2[3] < -180.0f) emaRPYval2[3] += 360;
        if (emaRPYval2[4] > 180.0f) emaRPYval2[4] -= 360; else if (emaRPYval2[4] < -180.0f) emaRPYval2[4] += 360;
        if (emaRPYval2[5] > 180.0f) emaRPYval2[5] -= 360; else if (emaRPYval2[5] < -180.0f) emaRPYval2[5] += 360;
        rpy_enc_deg[0] = emaRPYval2[1]; //+=est
        rpy_enc_deg[1] = emaRPYval2[3]; //+=north
        rpy_enc_deg[2] = emaRPYval2[5]; //+=ccw
        dt1_vrpy = tocDTsec() - dt0_vrpy; dt0_vrpy = tocDTsec();
        float roll_diff = rpy_enc_deg[0] - prev_vrpy[0];
        float pitch_diff = rpy_enc_deg[1] - prev_vrpy[1];
        float yaw_diff = rpy_enc_deg[2] - prev_vrpy[2];
        memcpy(prev_vrpy, rpy_enc_deg, sizeof(prev_vrpy));
        for (int i=0;i<3;i++){
            if (roll_diff > 180.0f) roll_diff -= 360.0f;    else if (roll_diff < -180.0f) roll_diff += 360.0f;
            if (pitch_diff > 180.0f) pitch_diff -= 360.0f;  else if (pitch_diff < -180.0f) pitch_diff += 360.0f;
            if (yaw_diff > 180.0f) yaw_diff -= 360.0f;      else if (yaw_diff < -180.0f) yaw_diff += 360.0f;
            if (abs(roll_diff)<=180 && abs(pitch_diff)<=180 && abs(yaw_diff)<=180) break;
        }
        // yaw vel filter
        if (abs(yaw_diff) > 1 && abs(dt1_vrpy-0.01f) <= 0.01f) yaw_diff = 0.15;
        // ---
        float tmp_vr = roll_diff / dt1_vrpy;
        float tmp_vp = pitch_diff / dt1_vrpy;
        float tmp_vy = yaw_diff / dt1_vrpy;
        if (isnan(tmp_vr) || isinf(tmp_vr)) tmp_vr = 0.0f;
        if (isnan(tmp_vp) || isinf(tmp_vp)) tmp_vp = 0.0f;
        if (isnan(tmp_vy) || isinf(tmp_vy)) tmp_vy = 0.0f;
        emaVRPYval[0] = (emaVRPY[0] * tmp_vr) + ((1 - emaVRPY[0]) * emaVRPYval[0]);
        emaVRPYval[1] = (emaVRPY[0] * tmp_vp) + ((1 - emaVRPY[0]) * emaVRPYval[1]);
        emaVRPYval[2] = (emaVRPY[0] * tmp_vy) + ((1 - emaVRPY[0]) * emaVRPYval[2]);
        emaVRPYval[3] = (emaVRPY[1] * emaVRPYval[0]) + ((1 - emaVRPY[1]) * emaVRPYval[3]);
        emaVRPYval[4] = (emaVRPY[1] * emaVRPYval[1]) + ((1 - emaVRPY[1]) * emaVRPYval[4]);
        emaVRPYval[5] = (emaVRPY[1] * emaVRPYval[2]) + ((1 - emaVRPY[1]) * emaVRPYval[5]);
        if (tocDTsec()<0.5f && dctlStages==0) {
            if (abs(emaVRPYval[3])>5 || abs(emaVRPYval[4])>5 || abs(emaVRPYval[5])>5) {
                memset(emaVRPYval, 0, sizeof(emaVRPYval));
            }
        }
        vel_rpy[0] = emaVRPYval[3];
        vel_rpy[1] = emaVRPYval[4];
        vel_rpy[2] = emaVRPYval[5];

        // ADC
        ioctl(fd_i2c_1, I2C_SLAVE, ADS1115_ADDR);
        write(fd_i2c_1, &reg_adc, 1);
        read(fd_i2c_1, data_adc, 2);
        adc_raw = (data_adc[0] << 8) | data_adc[1];
        emaADCval[0] = (emaADC[0] * adc_raw * (4.096f / 32768.0f) * 11.015f) + ((1 - emaADC[0]) * emaADCval[0]);
        emaADCval[1] = (emaADC[1] * emaADCval[0]) + ((1 - emaADC[1]) * emaADCval[1]);
        adc_volt = emaADCval[1];
        if (adc_volt >= 12.0f && adc_volt <= 26.0f && adc_volt < min_batt_volt) min_batt_volt = adc_volt;
        cnt_adc++;

        // Optical flow + altitude + pose estimation
        count_of = bb_serial_read(fd_msp2_of, PIN_BCM_RX_OF, buf_of, sizeof(buf_of));
        if (count_of > 0) {
            for (int i = 0; i < count_of; i++) {
                if (!parser_of.feed(buf_of[i], fn_of, payload_of, len_of)) continue;
                if (fn_of == MSP2_SENSOR_RANGEFINDER && len_of >= 5) {
                    quality_of = payload_of[0];
                    alt_of_m = ((float)le32s(payload_of + 1) / 1000.0f) - 0.026f;
                    cnt_alt++;
                }
                else if (fn_of == MSP2_SENSOR_OPTIC_FLOW && len_of >= 9) {
                    tempFPS = 1.0f / tocFPSsec(); ticFPSsec();
                    if (isnanf(tempFPS) || isinff(tempFPS)) tempFPS = 0;
                    fps_of = round(((emaFPS * tempFPS) + ((1 - emaFPS) * fps_of)) * 10.0f) / 10.0f;
                    dt1_of = tocDTsec() - dt0_of; dt0_of = tocDTsec();
                    // ---
                    if (read_of) {
                        float gy_rps_roll = angleDiff(rpy_enc_deg[0], prev_rpy_out[0]) / dt1_of * M_PI / 180.0f;
                        float gy_rps_pitch = angleDiff(rpy_enc_deg[1], prev_rpy_out[1]) / dt1_of * M_PI / 180.0f;
                        memcpy(prev_rpy_out, rpy_enc_deg, sizeof(prev_rpy_out));
                        float flow_x0 = (float)le32s(payload_of + 1) - (gy_rps_roll / PX_SCALE_OF);
                        float flow_y0 = (float)le32s(payload_of + 5) - (gy_rps_pitch / PX_SCALE_OF);
                        // flow filter on vel (z[m/s])
                        if (abs(vel_ned[2]) > 0.2f) {
                            flow_x0 *= 0.1;
                            flow_y0 *= 0.1;
                        }
                        // ---
                        quality_of = payload_of[0];
                        emaFLOWval[0] = (emaFLOW[0] * flow_x0) + ((1 - emaFLOW[0]) * emaFLOWval[0]); // lft
                        emaFLOWval[1] = (emaFLOW[1] * emaFLOWval[0]) + ((1 - emaFLOW[1]) * emaFLOWval[1]);
                        emaFLOWval[2] = (emaFLOW[0] * flow_y0) + ((1 - emaFLOW[0]) * emaFLOWval[2]); // fwd
                        emaFLOWval[3] = (emaFLOW[1] * emaFLOWval[2]) + ((1 - emaFLOW[1]) * emaFLOWval[3]);
                        flow_of_x = emaFLOWval[1]; // lft
                        flow_of_y = emaFLOWval[3]; // fwd
                        // px-to-body (optical flow is already in current body axes)
                        float vx_body =  flow_of_y * PX_SCALE_OF * pos_neu[2];  // fwd m/s
                        float vy_body = -flow_of_x * PX_SCALE_OF * pos_neu[2];  // rgt m/s
                        float offX = pos_neu[2] * sindeg(rpy_enc_deg[1]);
                        float offY = pos_neu[2] * sindeg(rpy_enc_deg[0]);
                        float dOffX = offX - prevXYoff[0]; prevXYoff[0] = offX;
                        float dOffY = offY - prevXYoff[1]; prevXYoff[1] = offY;
                        if (fabsf(dOffX) < 0.0001f) dOffX = 0.0f;
                        if (fabsf(dOffY) < 0.0001f) dOffY = 0.0f;
                        float dx_body = (vx_body * dt1_of) + dOffX;
                        float dy_body = (vy_body * dt1_of) - dOffY;
                        // body-to-world
                        float c_yaw = cosdeg(-rpy_enc_deg[2]);
                        float s_yaw = sindeg(-rpy_enc_deg[2]);
                        float dx_world = (dx_body * c_yaw) - (dy_body * s_yaw);
                        float dy_world = (dx_body * s_yaw) + (dy_body * c_yaw);
                        tmpXYworld[0] += dx_world;
                        tmpXYworld[1] += dy_world;
                        emaXYwval[0] = (emaXY[0] * tmpXYworld[0]) + ((1 - emaXY[0]) * emaXYwval[0]);
                        emaXYwval[1] = (emaXY[1] * emaXYwval[0]) + ((1 - emaXY[1]) * emaXYwval[1]);
                        emaXYwval[2] = (emaXY[0] * tmpXYworld[1]) + ((1 - emaXY[0]) * emaXYwval[2]);
                        emaXYwval[3] = (emaXY[1] * emaXYwval[2]) + ((1 - emaXY[1]) * emaXYwval[3]);
                        pos_neu[0] = emaXYwval[1];
                        pos_neu[1] = emaXYwval[3];
                        // world-to-body: NED xy is the SAME global point in current heading
                        pos_ned[0] =  (pos_neu[0] * c_yaw) + (pos_neu[1] * s_yaw);  // fwd
                        pos_ned[1] = -(pos_neu[0] * s_yaw) + (pos_neu[1] * c_yaw);  // rgt
                        // body velocity = translational flow in body axes
                        float tmpvx = vx_body;
                        float tmpvy = vy_body;
                        if (isnan(tmpvx) || isinf(tmpvx)) tmpvx = 0.0f;
                        if (isnan(tmpvy) || isinf(tmpvy)) tmpvy = 0.0f;
                        emaVNEDval[0] = (emaVNED[0] * tmpvx)        + ((1 - emaVNED[0]) * emaVNEDval[0]);
                        emaVNEDval[1] = (emaVNED[0] * tmpvy)        + ((1 - emaVNED[0]) * emaVNEDval[1]);
                        emaVNEDval[2] = (emaVNED[1] * emaVNEDval[0]) + ((1 - emaVNED[1]) * emaVNEDval[2]);
                        emaVNEDval[3] = (emaVNED[1] * emaVNEDval[1]) + ((1 - emaVNED[1]) * emaVNEDval[3]);
                        vel_ned[0] = emaVNEDval[2];
                        vel_ned[1] = emaVNEDval[3];
                        //---
                        xy_ready = true;
                    }
                }
            }
        }
        if (!read_of) {
            memset(emaFLOWval, 0, sizeof(emaFLOWval));
            memset(prevXYoff, 0, sizeof(prevXYoff));
            memset(tmpXYbody, 0, sizeof(tmpXYbody));
            memset(tmpXYworld, 0, sizeof(tmpXYworld));
            memset(emaXYbval, 0, sizeof(emaXYbval));
            memset(emaXYwval, 0, sizeof(emaXYwval));
            memset(prev_ned, 0, sizeof(prev_ned));
            memset(emaVNEDval, 0, sizeof(emaVNEDval));
            pos_ned[0] = 0; pos_ned[1] = 0;
            pos_neu[0] = 0; pos_neu[1] = 0;
            vel_ned[0] = 0; vel_ned[1] = 0;
        }

        // Logging (bin)
        if (startDctlLog) {
            dctlLog150[0]=fps_main_th[0]; dctlLog150[1]=fps_main_th[1]; dctlLog150[2]=fps_main_th[2];
            dctlLog150[3]=fps_imu; dctlLog150[4]=fps_of; dctlLog150[5]=fps_alt; dctlLog150[6]=fps_tof1; dctlLog150[7]=fps_adc;
            dctlLog150[8]=cpuRAMtmp[0]; dctlLog150[9]=cpuRAMtmp[1]; dctlLog150[10]=cpuRAMtmp[2];
            dctlLog150[11]=rpy_enc_deg[0]; dctlLog150[12]=rpy_enc_deg[1]; dctlLog150[13]=rpy_enc_deg[2];
            dctlLog150[14]=flow_of_x; dctlLog150[15]=flow_of_y; dctlLog150[16]=alt_of_m;
            dctlLog150[17]=adc_volt;
            dctlLog150[18]=tof1_avg_m;
            dctlLog150[19]=tocDTsec();
            dctlLog150[20]=pos_ned[0]; dctlLog150[21]=pos_ned[1]; dctlLog150[22]=pos_ned[2];
            dctlLog150[23]=pos_neu[0]; dctlLog150[24]=pos_neu[1]; dctlLog150[25]=pos_neu[2];
            dctlLog150[26]=vel_ned[0]; dctlLog150[27]=vel_ned[1]; dctlLog150[28]=vel_ned[2];
            dctlLog150[29]=vel_rpy[0]; dctlLog150[30]=vel_rpy[1]; dctlLog150[31]=vel_rpy[2];
            dctlLog150[32]=dctlStages;
            dctlLog150[33]=flightTime;
            // 34~40 rc_cmd
            // 41~104 ToF8x8
            // 105~139 controller
            // 140~147 ground effect
            if (firstLog) { firstLog=false; exportDctlLog(1); }
            else { exportDctlLog(0); }
        }
        else { if (!firstIMU && badIMUn>=10 && flightTime==0) { memset(emaRPYval, 0, sizeof(emaRPYval)); startDctlLog = true; ticDTsec(); }}

        //------------------------------
        if (!digitalRead(PIN_WPI_BUTF) || !digitalRead(PIN_WPI_BUTR)) emgcCode = -15;
        this_thread::sleep_until(nextP);
        nextP += PERIODus;
        cnt_main_th[1]++;
    }
}
PI_THREAD(th2_15) {
    while (!init_ready || init_stop) {
        if (!init_stop) { set_buz_led(-1,1,0,0,-1); delay(200); if (!digitalRead(PIN_WPI_BUTF) || !digitalRead(PIN_WPI_BUTR)) { init_stop = true; } }
        if (!init_stop) { set_buz_led(-1,1,1,0,-1); delay(200); if (!digitalRead(PIN_WPI_BUTF) || !digitalRead(PIN_WPI_BUTR)) { init_stop = true; } }
        if (!init_stop) { set_buz_led(-1,1,1,1,-1); delay(200); if (!digitalRead(PIN_WPI_BUTF) || !digitalRead(PIN_WPI_BUTR)) { init_stop = true; } }
        if (!init_stop) { set_buz_led(-1,0,0,0,-1); delay(200); if (!digitalRead(PIN_WPI_BUTF) || !digitalRead(PIN_WPI_BUTR)) { init_stop = true; } }
        if (init_stop) {
            cout << ">>> Terminated ..." << endl;
            set_buz_led(1,0,0,0,0); delay(50); set_buz_led();
            while (true) { set_buz_led(0,1,1,1,0); delay(1000); set_buz_led(); delay(100); }
        }
        delay(100); 
    }
    set_buz_led(1,0,0,0,0); delay(50); set_buz_led();
    
    const us0 PERIODus(66666); // 15Hz, ToF8x8 + stages + emgc + misc (cpu+fps+debug)
    auto nextP = clock0::now() + PERIODus;
    auto t0a = high_resolution_clock::now(), t1a=t0a, t2a=t0a;
    auto ticFPSsec = [&]() { t0a = high_resolution_clock::now(); };
    auto tocFPSsec = [&]() { return (float)(high_resolution_clock::now() - t0a).count() / 1000000000; };
    auto ticSTGsec = [&]() { t1a = high_resolution_clock::now(); };
    auto tocSTGsec = [&]() { return (float)(high_resolution_clock::now() - t1a).count() / 1000000000; };
    auto ticDTsec = [&]() { t2a = high_resolution_clock::now(); };
    auto tocDTsec = [&]() { return (float)(high_resolution_clock::now() - t2a).count() / 1000000000; };
    auto cosdeg = [&](float deg) { return cos((deg * M_PI) / 180.0); };

    VL53L5CX_ResultsData ToF1_Results;
    const float emaALT[2]={0.95f,0.80f}; float emaALTval[2]={0};
    const float emaVNEDz[2]={0.75f,0.55f}; float emaVNEDzval[2]={0}, prev_nedz=0;
    float dtFps, tempFPS, batt0, dt_vz, prev_pos[4]; int emgc_blink=0; bool blink_state=true;

    cout << ">>> [OK] th2 (15)" << endl;
    ticFPSsec(); ticSTGsec(); ticDTsec();

    while (true) {
        // ToF1 down
        uint8_t isToF = 0;
        vl53l5cx_check_data_ready(&DevToF, &isToF);
        if (isToF) {
            vl53l5cx_get_ranging_data(&DevToF, &ToF1_Results);
            float tof1_tot = 0.0f;
            for (int i = 0; i < 64; i++) { 
                tof1_m[i] = (float)ToF1_Results.distance_mm[i] / 1000.0f;
                dctlLog150[41 + i] = tof1_m[i];
                tof1_tot += tof1_m[i];
            }
            tof1_avg_m = tof1_tot / 64.f;
            if (tof1_avg_m < 0) tof1_avg_m = 0.0f;
            tof1_single_m = (tof1_m[29] + tof1_m[30] + tof1_m[31] + tof1_m[32] + tof1_m[33]) / 5.0f;
            cnt_tof1++;
            // ned z with angle compensation ----------------
            emaALTval[0] = (emaALT[0] * tof1_avg_m * cosdeg(rpy_enc_deg[0]) * cosdeg(rpy_enc_deg[1])) + ((1 - emaALT[0]) * emaALTval[0]);
            emaALTval[1] = (emaALT[1] * emaALTval[0]) + ((1 - emaALT[1]) * emaALTval[1]);
            pos_neu[2] = emaALTval[1];
            pos_ned[2] = -pos_neu[2];
            // vel z
            dt_vz = tocDTsec(); ticDTsec();
            float tmpvz = (pos_ned[2] - prev_nedz) / dt_vz; prev_nedz = pos_ned[2];
            if (isnan(tmpvz) || isinf(tmpvz)) tmpvz = 0.0f;
            emaVNEDzval[0] = (emaVNEDz[0] * tmpvz) + ((1 - emaVNEDz[0]) * emaVNEDzval[0]);
            emaVNEDzval[1] = (emaVNEDz[1] * emaVNEDzval[0]) + ((1 - emaVNEDz[1]) * emaVNEDzval[1]);
            vel_ned[2] = emaVNEDzval[1];
            // Ground effect --------------------------------
            const float gainGE = 1400.0f; //1250
            if (dctlStages == 4) gestart = true;
            if (GE_state == 2) {
                float h_scalar = pos_neu[2];
                const float H_THRESHOLD = 0.150f;
                const float H_MIN = 0.050f;
                const float k1 = 2.5; // similar as GE4
                const float R_PROP = 0.045f;
                if (h_scalar < H_THRESHOLD) {
                    float h_c = std::max(h_scalar, H_MIN);
                    ge_TRPY[0] = gainGE * (-k1 * std::pow(R_PROP / (4.0 * h_c), 2.0));
                }
                else { memset(ge_TRPY, 0, sizeof(ge_TRPY)); }
            }
            else if (GE_state == 3) {
                static ndo::LESO leso;
                static bool leso_configured = false;
                static int prev_ge_state = -1;
                const float K_T   = 17.05f; //slope=17.75f higher=lessaggressive N per normalized-throttle-unit -- (mass*g)/throttle_hover
                const float MASS  = 0.877f;
                const float OMEGA_O = 6.0f;
                if (!leso_configured) {
                    leso.configure(/*pwm_min=*/1000.0f, /*pwm_max=*/2000.0f, K_T, MASS, OMEGA_O, /*d_hat_max=*/15.0f);
                    leso_configured = true;
                }
                if (prev_ge_state != 3) leso.reset();
                float correction_norm = leso.update(pos_neu[2], last_applied_pwm_ndo, 0.0666);

                if (correction_norm>0.12) correction_norm=0.12; else if (correction_norm<-0.12) correction_norm=-0.12;
                ge_TRPY[0] = gainGE * correction_norm;
                if (ge_TRPY[0]>0) ge_TRPY[0]=0; else if (ge_TRPY[0]<-150) ge_TRPY[0]=-150;
                ge_TRPY[1] = 0.0f;
                ge_TRPY[2] = 0.0f;
                ge_TRPY[3] = 0.0f;
                ge_TRPY[4] = leso.dHat();
                ge_TRPY[5] = leso.zHat();
                ge_TRPY[6] = leso.vzHat();
                ge_TRPY[7] = correction_norm;
                prev_ge_state = GE_state;
            }
            else if (GE_state == 4) {
                // // ablation1
                // float tof1_single[64]; for (int i=0;i<64;i++) {tof1_single[i] = tof1_single_m;}
                // ge::GEResult geResult = ge::computeGroundEffect(tof1_single, rpy_enc_deg[0], rpy_enc_deg[1]);
                
                ge::GEResult geResult = ge::computeGroundEffect(tof1_m, rpy_enc_deg[0], rpy_enc_deg[1]);
                if (geResult.valid) {
                    float tmpT = gainGE * (+geResult.dT_motor[0]+geResult.dT_motor[1]+geResult.dT_motor[2]+geResult.dT_motor[3])*0.25f; // invert motor mixer
                    float tmpR = gainGE * (-geResult.dT_motor[0]+geResult.dT_motor[1]-geResult.dT_motor[2]+geResult.dT_motor[3])*0.25f;
                    float tmpP = gainGE * (-geResult.dT_motor[0]-geResult.dT_motor[1]+geResult.dT_motor[2]+geResult.dT_motor[3])*0.25f;
                    float tmpY = gainGE * -(-geResult.dT_motor[0]+geResult.dT_motor[1]+geResult.dT_motor[2]-geResult.dT_motor[3])*0.25f;
                    ge_TRPY[0] = tmpT;
                    ge_TRPY[1] = tmpR; //ablation2 0
                    ge_TRPY[2] = tmpP; //ablation2 0
                    ge_TRPY[3] = tmpY; //ablation2 0
                    ge_TRPY[4] = (float)geResult.alpha_rad*180.0/M_PI; // deg
                    ge_TRPY[5] = (float)geResult.h_eff; // m
                    ge_TRPY[6] = (float)geResult.e_rms; // m
                    ge_TRPY[7] = (float)geResult.dT_ff * gainGE; // Thrust
                }
                else {} // keep previous ge_TRPY values
            }
            else { memset(ge_TRPY, 0, sizeof(ge_TRPY)); }
            dctlLog150[140] = ge_TRPY[0];
            dctlLog150[141] = ge_TRPY[1];
            dctlLog150[142] = ge_TRPY[2];
            dctlLog150[143] = ge_TRPY[3];
            dctlLog150[144] = ge_TRPY[4];
            dctlLog150[145] = ge_TRPY[5];
            dctlLog150[146] = ge_TRPY[6];
            dctlLog150[147] = ge_TRPY[7];
            // ---
            z_ready = true;
        }

        // drone stages | 0=landed 1=takeoff 2=hover 3=move 4=landing (-)=emergency
        if (startMission && !SLOPE_START) {
            if (dctlStages == 0) {
                if (flightTime == 0 && emgcCode == 0) {
                    dctlStages = 1; batt0 = adc_volt; set_buz_led(0,1,0,0,0); ticSTGsec();
                    cout << "------------------------------ (1) Takeoff ------------------------------" << endl;
                }
                else {
                    cout << endl;
                    cout << "**********************************************************" << endl;
					cout << " FlightTime: " << flightTime << "s" << endl;
					cout << " StartBat: " << batt0 << "V  endBat: " << adc_volt << "V  low: " << min_batt_volt << "V (e:" << EMGC_LAND_BATT << "V)" << endl;
                    cout << " FPS main-th: [" << fps_main_th[0] << " " << fps_main_th[1] << " " << fps_main_th[2] << "]" << endl;
                    cout << " FPS imu:" << fps_imu << " OF:" << fps_of << " alt:" << fps_alt << " tof1:" << fps_tof1 << " adc:" << fps_adc << endl;
                    cout << " CPU: " << cpuRAMtmp[0] << "% RAM: " << cpuRAMtmp[1] << "% temp: " << cpuRAMtmp[2] << "C" << endl;
					cout << "**********************************************************" << endl;
                    cout << endl;
                    if (emgcCode == -1) cout << " >>> [EMGC LANDING] Battery (-1)" << endl;
                    else if (emgcCode == -2) cout << " >>> [EMGC LANDING] Altitude (-2)" << endl;
                    else if (emgcCode == -3) cout << " >>> [EMGC LANDING] FPS (-3)" << endl;
                    else if (emgcCode == -11) cout << " >>> [EMGC STOP] Lin. Velocity (-11)" << endl;
                    else if (emgcCode == -12) cout << " >>> [EMGC STOP] Angle (-12)" << endl;
                    else if (emgcCode == -13) cout << " >>> [EMGC STOP] FPS (-13)" << endl;
                    else if (emgcCode == -14) cout << " >>> [EMGC STOP] Altitude (-14)" << endl;
                    else if (emgcCode == -15) cout << " >>> [EMGC STOP] Buttons (-15)" << endl;
                    else if (emgcCode == -16) cout << " >>> [EMGC STOP] Ang. Velocity (-16)" << endl;
                    if (emgcCode < 0) dctlStages=emgcCode;
                    nOdom=0; mOdom=0; startMission=false;
                }
            }
            else if (dctlStages == 1) {
                if (pos_neu[2] > 0.050) { set_buz_led(-1,-1,-1,-1,1); read_of = true; }
                if (pos_neu[2] > HOVER_ALT_M - 0.055f || tocSTGsec() > 6.0) {
                    if (pos_neu[2] < 0.1f) {
                        cout << "------------------------------ (-) Aborted ------------------------------ " << tocSTGsec() << "s / 6.0s" << endl;
                        flightTime = 1; dctlStages = 0; set_buz_led(0,0,0,1,-1); ticSTGsec();
                    }
                    else {
                        cout << "------------------------------- (2) Hover ------------------------------- " << tocSTGsec() << "s / 6.0s" << endl;
                        dctlStages = 2; set_buz_led(0,0,1,0,-1); ticSTGsec();
                    }
                }
            }
            else if (dctlStages == 2) {
                if (nOdom > mOdom && tocSTGsec() >= odomGoal_NEU_CCW[mOdom][6]) {
                    if (odomGoal_NEU_CCW[mOdom][4] == 0) { // hover
                        cout << " [" << mOdom+1 << "/" << nOdom << "] (2) Hover [" << odomGoal_NEU_CCW[mOdom][6] << "s]" << endl;
                    }
                    else { // move
                        cout << " [" << mOdom+1 << "/" << nOdom << "] (3) Move [" << odomGoal_NEU_CCW[mOdom][0] << "m " << odomGoal_NEU_CCW[mOdom][1] << "m " << odomGoal_NEU_CCW[mOdom][2] << "m " << odomGoal_NEU_CCW[mOdom][3] << "deg]" << endl;
                        memcpy(prev_pos, posTgtNeuDctl, sizeof(prev_pos));
                        posTgtNeuDctl[0] += odomGoal_NEU_CCW[mOdom][0]; //x
                        posTgtNeuDctl[1] += odomGoal_NEU_CCW[mOdom][1]; //y
                        posTgtNeuDctl[2] += odomGoal_NEU_CCW[mOdom][2]; //z
                        posTgtNeuDctl[3] += odomGoal_NEU_CCW[mOdom][3]; //yaw
                        if (posTgtNeuDctl[3] > 180.0f) posTgtNeuDctl[3]-=360; else if (posTgtNeuDctl[3] < -180.0f) posTgtNeuDctl[3]+=360;
                        if (posTgtNeuDctl[2] > 1.2f) posTgtNeuDctl[2]=1.2f; else if (posTgtNeuDctl[2] < 0.15f) posTgtNeuDctl[2]=0.15f;
                        posErrNeuDctl[0] = posTgtNeuDctl[0] - prev_pos[0];
                        posErrNeuDctl[1] = posTgtNeuDctl[1] - prev_pos[1];
                        posErrNeuDctl[2] = posTgtNeuDctl[2] - prev_pos[2];
                        posErrNeuDctl[3] = posTgtNeuDctl[3] - prev_pos[3];
                        if (posErrNeuDctl[3] > 180.0f) posErrNeuDctl[3]-=360; else if (posErrNeuDctl[3] < -180.0f) posErrNeuDctl[3]+=360;
                        dctlStages = 3; set_buz_led(0,1,0,0,-1);
                    }                  
                    mOdomCur = mOdom;
                    mOdom++; if (mOdom >= nOdom) { nOdom=0; mOdom=0; }
                    ticSTGsec();
                }
                else if (nOdom == 0 && tocSTGsec() > 1.0f) {
                    cout << "------------------------------ (4) Landing ------------------------------ " << tocSTGsec() << "s" << endl;
                    dctlStages = 4; set_buz_led(0,1,0,0,-1); ticSTGsec();
                }
            }
            else if (dctlStages == 3) {
                const float odom_done[3] = {0.055f, 0.025f, 6.0f}; //m, m, deg
                if (tocSTGsec() > odomGoal_NEU_CCW[mOdomCur][5] || 
                    (odomGoal_NEU_CCW[mOdomCur][4] == 1 && posErrNeuDctl[0] <= odom_done[0]) ||    //x fwd
                    (odomGoal_NEU_CCW[mOdomCur][4] == 2 && posErrNeuDctl[0] >= -odom_done[0]) ||   //x bck
                    (odomGoal_NEU_CCW[mOdomCur][4] == 3 && posErrNeuDctl[1] <= odom_done[0]) ||    //y rgt
                    (odomGoal_NEU_CCW[mOdomCur][4] == 4 && posErrNeuDctl[1] >= -odom_done[0]) ||   //y lft
                    (odomGoal_NEU_CCW[mOdomCur][4] == 5 && posErrNeuDctl[2] <= odom_done[1]) ||    //z up
                    (odomGoal_NEU_CCW[mOdomCur][4] == 6 && posErrNeuDctl[2] >= -odom_done[1]) ||   //z dwn
                    (odomGoal_NEU_CCW[mOdomCur][4] == 7 && posErrNeuDctl[3] <= odom_done[2]) ||    //yaw ccw
                    (odomGoal_NEU_CCW[mOdomCur][4] == 8 && posErrNeuDctl[3] >= -odom_done[2]) ) {  //yaw cw
                    if (tocSTGsec() > odomGoal_NEU_CCW[mOdomCur][5]) cout << " Timeout" << endl;
                    cout << "------------------------------- (2) Hover ------------------------------- " << tocSTGsec() << "s / " << odomGoal_NEU_CCW[mOdomCur][5] << "s" << endl;
                    dctlStages = 2; set_buz_led(0,0,1,0,-1); ticSTGsec();
                }
            }
            else if (dctlStages == 4) {
                if (pos_neu[2] <= 0.055f || tocSTGsec() > 5.0) {
                    cout << "------------------------------- (0) Landed ------------------------------ " << tocSTGsec() << "s / 5.0s" << endl;
                    dctlStages = 0; set_buz_led(); ticSTGsec();
                }
            }
            // emergency ---
            if (dctlStages > 0) {
                // emgc landing (2nd priority)
                if (adc_volt <= EMGC_LAND_BATT || min_batt_volt <= EMGC_LAND_BATT) emgcCode = -1;
                else if (pos_neu[2] > EMGC_LAND_ALT) emgcCode = -2;
                else if (fps_main_th[0] < 50 || fps_main_th[1] < 50 || fps_main_th[2] < 7) emgcCode = -3;
                else if (fps_imu < 50 || fps_of < 5 || fps_alt < 7 || fps_tof1 < 7 || fps_adc < 50) emgcCode = -3;
                // emgc stop (1st priority)
                if (abs(vel_ned[0]) > EMGC_STOP_LINVEL || abs(vel_ned[1]) > EMGC_STOP_LINVEL || (abs(vel_ned[2]) > EMGC_STOP_LINVEL && dctlStages != 1)) emgcCode = -11;
                else if (abs(rpy_enc_deg[0]) > EMGC_STOP_TILT || abs(rpy_enc_deg[1]) > EMGC_STOP_TILT) emgcCode = -12;
                else if (fps_main_th[0] <= 0 || fps_main_th[1] <= 0 || fps_main_th[2] <= 0) emgcCode = -13;
                else if (fps_imu <= 0 || fps_of <= 0 || fps_alt <= 0 || fps_tof1 <= 0 || fps_adc <= 0) emgcCode = -13;
                else if (pos_neu[2] > EMGC_STOP_ALT) emgcCode = -14;
                else if (!digitalRead(PIN_WPI_BUTF) || !digitalRead(PIN_WPI_BUTR)) emgcCode = -15;
                else if (abs(vel_rpy[0]) > EMGC_STOP_ANGVEL || abs(vel_rpy[1]) > EMGC_STOP_ANGVEL || abs(vel_rpy[2]) > EMGC_STOP_ANGVEL) emgcCode = -16;
                // stage updates
                if (emgcCode <= -11 && emgcCode >= -16) {
                    cout << "----------------------------- (0) EMGC STOP ----------------------------- " << tocSTGsec() << "s" << endl;
                    dctlStages = 0; set_buz_led(0,1,1,1,0); ticSTGsec();
                }
                else if (emgcCode <= -1 && emgcCode >= -3 && dctlStages != 4) {
                    cout << "---------------------------- (4) EMGC LANDING --------------------------- " << tocSTGsec() << "s" << endl;
                    if (dctlStages == 3) {
                        if (odomGoal_NEU_CCW[mOdomCur][4] >= 1 && odomGoal_NEU_CCW[mOdomCur][4] <= 4) {
                            posTgtNeuDctl[0] = pos_neu[0];
                            posTgtNeuDctl[1] = pos_neu[1];
                        }
                        else if (odomGoal_NEU_CCW[mOdomCur][4] == 5 || odomGoal_NEU_CCW[mOdomCur][4] == 6) posTgtNeuDctl[2] = pos_neu[2];
                        else if (odomGoal_NEU_CCW[mOdomCur][4] == 7 || odomGoal_NEU_CCW[mOdomCur][4] == 8) {
                            posTgtNeuDctl[3] = rpy_enc_deg[2];
                            if (posTgtNeuDctl[3] > 180.0f) posTgtNeuDctl[3]-=360; else if (posTgtNeuDctl[3] < -180.0f) posTgtNeuDctl[3]+=360;
                        }                        
                    }
                    if (dctlStages != 4) ticSTGsec();
                    dctlStages = 4; set_buz_led(0,1,0,1,-1);
                }
                // misc
                if (adc_volt <= EMGC_LAND_BATT+0.25) set_buz_led(-1,-1,1,1,-1);
                else if (adc_volt <= EMGC_LAND_BATT+0.5) set_buz_led(-1,-1,-1,1,-1);
            }
        }
        else if (blink_state && emgcCode <= -11 && emgcCode >= -16) {
            emgc_blink++;
            if (emgc_blink == 2) { set_buz_led(); }
            if (emgc_blink >= 4) { set_buz_led(0,1,1,1,0); emgc_blink=0; }
            if (!digitalRead(PIN_WPI_BUTF) || !digitalRead(PIN_WPI_BUTR)) { 
                blink_state = false;  emgc_blink = 0;
                set_buz_led(1,0,0,0,0); delay(50); set_buz_led();
            }
        }

        getCPUramTmp();
        //---
        dtFps = tocFPSsec(); ticFPSsec();
		tempFPS = (float)cnt_main_th[0] / dtFps; cnt_main_th[0] = 0; 
		if (isnanf(tempFPS) || isinff(tempFPS)) tempFPS = 0;
		fps_main_th[0] = round(((emaFPS * tempFPS) + ((1 - emaFPS) * fps_main_th[0])) * 10.0f) / 10.0f;
        tempFPS = (float)cnt_main_th[1] / dtFps; cnt_main_th[1] = 0;
		if (isnanf(tempFPS) || isinff(tempFPS)) tempFPS = 0;
		fps_main_th[1] = round(((emaFPS * tempFPS) + ((1 - emaFPS) * fps_main_th[1])) * 10.0f) / 10.0f;
        tempFPS = (float)cnt_main_th[2] / dtFps; cnt_main_th[2] = 0;
		if (isnanf(tempFPS) || isinff(tempFPS)) tempFPS = 0;
		fps_main_th[2] = round(((emaFPS * tempFPS) + ((1 - emaFPS) * fps_main_th[2])) * 10.0f) / 10.0f;
        //---
        tempFPS = (float)cnt_imu / dtFps; cnt_imu = 0;
		if (isnanf(tempFPS) || isinff(tempFPS)) tempFPS = 0;
		fps_imu = round(((emaFPS * tempFPS) + ((1 - emaFPS) * fps_imu)) * 10.0f) / 10.0f;
        tempFPS = (float)cnt_alt / dtFps; cnt_alt = 0;
		if (isnanf(tempFPS) || isinff(tempFPS)) tempFPS = 0;
		fps_alt = round(((emaFPS * tempFPS) + ((1 - emaFPS) * fps_alt)) * 10.0f) / 10.0f;
        tempFPS = (float)cnt_tof1 / dtFps; cnt_tof1 = 0;
		if (isnanf(tempFPS) || isinff(tempFPS)) tempFPS = 0;
		fps_tof1 = round(((emaFPS * tempFPS) + ((1 - emaFPS) * fps_tof1)) * 10.0f) / 10.0f;
        tempFPS = (float)cnt_adc / dtFps; cnt_adc = 0;
		if (isnanf(tempFPS) || isinff(tempFPS)) tempFPS = 0;
		fps_adc = round(((emaFPS * tempFPS) + ((1 - emaFPS) * fps_adc)) * 10.0f) / 10.0f;

        if (print_debug) {
            bool butf = digitalRead(PIN_WPI_BUTF);
            bool butr = digitalRead(PIN_WPI_BUTR);
            if (!butf || !butr) set_buz_led(-1,1,1,1,1); else set_buz_led(-1,0,0,0,0);
            cout << "======================================" << endl;
            cout << "FPS main-th: [" << fps_main_th[0] << " " << fps_main_th[1] << " " << fps_main_th[2] << "]" << endl;
            cout << "FPS imu:" << fps_imu << " OF:" << fps_of << " alt:" << fps_alt << " tof1:" << fps_tof1 << " adc:" << fps_adc << endl;
            cout << "CPU: " << cpuRAMtmp[0] << "% RAM: " << cpuRAMtmp[1] << "% temp: " << cpuRAMtmp[2] << "C" << endl;
            cout << "but-FR: [" << butf << " " << butr << "]" << endl;
            cout << "IMU" << endl;
            cout << "   rpy: [" << rpy_enc_deg[0] << " " << rpy_enc_deg[1] << " " << rpy_enc_deg[2] << "] deg" << endl;
            cout << "OF (raw): [" << flow_of_x << "px " << flow_of_y << "px " << alt_of_m <<"m]" <<endl;
            cout << "POS-NED: [" << pos_ned[0] << " " << pos_ned[1] << " " << pos_ned[2] << "] m" << endl;
            cout << "POS-NEU: [" << pos_neu[0] << " " << pos_neu[1] << " " << pos_neu[2] << "] m" << endl;
            cout << "ToF1 ave: " << tof1_avg_m << " m"<< endl;
            for (int y = 0; y < 8; y++) {
                cout << "   ";
                for (int x = 0; x < 8; x++) {
                    int idx = y * 8 + x;
                    cout << tof1_m[idx] << "\t";
                }
                if (y == 8-1) cout << "m" << endl; 
                else cout << endl;
            }
            cout << "ADC" << endl;
            cout << "   raw: " << adc_raw << endl;
            cout << "   volt: " << adc_volt << endl;
            cout << endl;
        }
        
        //------------------------------
        this_thread::sleep_until(nextP);
        nextP += PERIODus;
        cnt_main_th[2]++;
    }
}

int main(void) {
    signal(SIGINT, sigint_handler);
    wiringPiSetup();
    pinMode(PIN_WPI_BUTF, INPUT);
    pinMode(PIN_WPI_BUTR, INPUT);
    pinMode(PIN_WPI_BUZZ, OUTPUT);
    pinMode(PIN_WPI_LEDR, OUTPUT);
    pinMode(PIN_WPI_LEDG, OUTPUT);
    pinMode(PIN_WPI_LEDB, OUTPUT);
    pinMode(PIN_WPI_LEDFLASH1, OUTPUT);

    cout << ">>> Initializing ..." << endl;
    set_buz_led(1,0,0,0,0); delay(50); 
    set_buz_led(0,0,0,0,0); delay(50);
    piThreadCreate(th1_100);
    piThreadCreate(th2_15);

    fd_crsf_fc = open_ser_crsf_fc(FC_SER_PORT); 
    if (fd_crsf_fc < 0) {cout << "[Bad] Serial FC" << endl; return -1;}
    cout << ">>> [OK] Serial FC" << endl; delay(50);
    fd_msp2_of = open_ser_msp2_of(); 
    if (fd_msp2_of < 0) {cout << "[Bad] Serial OF" << endl; return -1;}
    cout << ">>> [OK] Serial OF" << endl; delay(50);
    fd_i2c_1 = open(I2C_DEV_1, O_RDWR);
    fd_i2c_3 = open(I2C_DEV_3, O_RDWR);
    if (fd_i2c_1 < 0) {cout << "[Bad] I2C-1" << endl; return -1;}
    if (fd_i2c_3 < 0) {cout << "[Bad] I2C-3" << endl; return -1;}
    cout << ">>> [OK] I2C-1 I2C-3" << endl; delay(50);
    if (imu_init(fd_i2c_1) < 0) {cout << "[Bad] IMU" << endl; return -1;}
    cout << ">>> [OK] IMU" << endl; delay(50);
    if (adc_init(fd_i2c_1) < 0) {cout << "[Bad] ADC" << endl; return -1;}
    cout << ">>> [OK] ADC" << endl; delay(50);
    if (tof_init(fd_i2c_3) < 0) {cout << "[Bad] ToF1" << endl; return -1;}
    cout << ">>> [OK] ToF1" << endl; delay(50);
    if (!init_stop) { init_ready = true; cout << ">>> Starting ..." << endl; }
    while (fps_main_th[1] < 50 || fps_main_th[2] < 7 || fps_imu < 50 || fps_of < 5 || fps_alt < 15 || fps_tof1 < 7 || fps_adc < 50 || init_stop) { 
        if (!digitalRead(PIN_WPI_BUTF) || !digitalRead(PIN_WPI_BUTR)) { init_stop = true; } 
        delay(100);
    }

    uint16_t ch_us_mot[16]; uint8_t frame_mot[26]; int mot_seq=0;
    float posPID[3][3]={0}, tmpIpos[3]={0}, posErrNeuDctlPrev[4]={0}, vel_tgt_NED[3]={0};
    float velPID[3][3]={0}, tmpIvel[3]={0}, velErrNedDctlPrev[3]={0}, velErrNedDctl[3]={0};
    float Fxyz_NEU[3]={0}, Fyaw_CW=0, vxy_world[2]={0};
    float yawPID[3]={0}, tmpYPID=0, flagYawPrev=0;
    bool frs_print=true, flagYawDone=false;
    float Throttle_Ft=0, RPY_ref[3]={0}, Throttle_prev=0;
    float mg_tmp=1000, vel_tgtZ_prev=0, Fu=0; 
    bool isHover = false, isXYreset = false;
    int dctlStages_prev = 0;
    float cmd_TRP_slope[3] = {0};

    const float mg_throttle     = 1400;
    const float maxThrottle     = 1600;
    const float maxFz           = 75;
    const float maxFxy          = 150;
    const float maxFyaw         = 100;
    const float RCcenter        = 1500;
    const float dt100Hz         = 1.0f / 100.0f;
    const float dt10Hz          = 1.0f / 10.0f;
    const float dt15Hz          = 1.0f / 15.0f;
    const float kPIDvelZ[3]     = {80,200,15};
    const float kPIDposZ[3]     = {0.8,0.15,0.10};
    const float kPIDvelXY[3]    = {115,30,50};
    const float kPIDposXY[3]    = {0.65,0.04,0.05};
    const float kPIDyaw[3]      = {6,0.1,0.05};

    auto t0a = high_resolution_clock::now();
    auto ticFTsec = [&]() { t0a = high_resolution_clock::now(); };
    auto tocFTsec = [&]() { return (float)(high_resolution_clock::now() - t0a).count() / 1000000000; };
    auto sindeg = [&](float deg) { return sin((deg * M_PI) / 180.0); };
	auto cosdeg = [&](float deg) { return cos((deg * M_PI) / 180.0); };
	auto tandeg = [&](float deg) { return tan((deg * M_PI) / 180.0); };
    auto motDisArm = [&]() {
        rc_cmd[0] = RCcenter; // roll
        rc_cmd[1] = RCcenter; // pitch
        rc_cmd[3] = RCcenter; // yaw
        rc_cmd[2] = 1000; // throttle (1000-2000)
        rc_cmd[4] = 2000; // AUX1: arm (on: 900-1200)
    };
    auto motArm = [&]() {
        rc_cmd[0] = RCcenter; // roll
        rc_cmd[1] = RCcenter; // pitch
        rc_cmd[3] = RCcenter; // yaw
        rc_cmd[2] = 1000; // throttle (1000-2000)
        rc_cmd[4] = 1000; // AUX1: arm (on: 900-1200)
    };
    auto motFlightPID = [&]() {
        if (z_ready) {
            z_ready = false;
            if (dctlStages == 1) {
                posTgtNeuDctl[2] += 0.025f; //0.050 0.01
                posTgtNeuDctl[2] = min(posTgtNeuDctl[2], HOVER_ALT_M);
            }
            else if (dctlStages == 4) {
                posTgtNeuDctl[2] -= 0.01f; //0.015 0.025
                posTgtNeuDctl[2] = max(posTgtNeuDctl[2], 0.0f);
            }
            // Error pos (NEU) -------------------------------------------------------------
            posErrNeuDctl[2] = posTgtNeuDctl[2] - pos_neu[2];
            float errSat = 0.200f; // m
            if (posErrNeuDctl[2] > errSat) posErrNeuDctl[2] = errSat; else if (posErrNeuDctl[2] < -errSat) posErrNeuDctl[2] = -errSat;            
            // PID z -----------------------------------------------------------------------
            errSat = 0.200f; // m/s
            tmpIpos[2] += posErrNeuDctl[2] * dt15Hz;            
            posPID[2][0] = kPIDposZ[0] * posErrNeuDctl[2];
            posPID[2][1] = kPIDposZ[1] * tmpIpos[2];
            posPID[2][2] = kPIDposZ[2] * (posErrNeuDctl[2] - posErrNeuDctlPrev[2]) / dt15Hz;
            posErrNeuDctlPrev[2] = posErrNeuDctl[2];
            if (isnan(posPID[2][0]) || isinf(posPID[2][0])) posPID[2][0] = 0.0f;
            if (isnan(posPID[2][1]) || isinf(posPID[2][1])) posPID[2][1] = 0.0f;
            if (isnan(posPID[2][2]) || isinf(posPID[2][2])) posPID[2][2] = 0.0f;
            vel_tgt_NED[2] = -(posPID[2][0] + posPID[2][1] + posPID[2][2]); // world(NEU) to body(NED)
            if (vel_tgt_NED[2] > errSat) vel_tgt_NED[2] = errSat; else if (vel_tgt_NED[2] < -errSat) vel_tgt_NED[2] = -errSat;
            if (dctlStages == 1) {
                if (vel_tgt_NED[2] > vel_tgtZ_prev) vel_tgt_NED[2] = vel_tgtZ_prev;
            }
            vel_tgtZ_prev = vel_tgt_NED[2];
            // Error vel (NED) -------------------------------------------------------------
            velErrNedDctl[2] = vel_tgt_NED[2] - vel_ned[2];
            if (velErrNedDctl[2] > errSat) velErrNedDctl[2] = errSat; else if (velErrNedDctl[2] < -errSat) velErrNedDctl[2] = -errSat;
            // PID vz ----------------------------------------------------------------------
            tmpIvel[2] += velErrNedDctl[2] * dt15Hz;
            velPID[2][0] = kPIDvelZ[0] * velErrNedDctl[2];
            velPID[2][1] = kPIDvelZ[1] * tmpIvel[2];
            velPID[2][2] = kPIDvelZ[2] * (velErrNedDctl[2] - velErrNedDctlPrev[2]) / dt15Hz;
            velErrNedDctlPrev[2] = velErrNedDctl[2];
            if (isnan(velPID[2][0]) || isinf(velPID[2][0])) velPID[2][0] = 0.0f;
            if (isnan(velPID[2][1]) || isinf(velPID[2][1])) velPID[2][1] = 0.0f;
            if (isnan(velPID[2][2]) || isinf(velPID[2][2])) velPID[2][2] = 0.0f;
            Fxyz_NEU[2] = -(velPID[2][0] + velPID[2][1] + velPID[2][2]); // Invert
            if (Fxyz_NEU[2] > maxFz) Fxyz_NEU[2] = maxFz; else if (Fxyz_NEU[2] < -maxFz) Fxyz_NEU[2] = -maxFz;
            // Conversion
            if (!isHover) {
                mg_tmp += 15; //35 50
                mg_tmp = min(mg_tmp, mg_throttle);
                if (dctlStages == 2) {
                    isHover = true;
                    mg_tmp = Fu;
                    tmpIpos[2] = 0; tmpIvel[2] = 0;
                }
            }
            Fu = Fxyz_NEU[2] + mg_tmp;
            Throttle_Ft = Fu / (cosdeg(rpy_enc_deg[0]) * cosdeg(rpy_enc_deg[1]));
            // Output saturation ----------------------------------------------------------           
            if (Throttle_Ft > maxThrottle) Throttle_Ft = maxThrottle;
            Throttle_prev = Throttle_Ft;
            // log 105~109 110~114
            dctlLog150[105] = -velPID[2][0]; dctlLog150[106] = -velPID[2][1]; dctlLog150[107] = -velPID[2][2]; dctlLog150[108] = Fxyz_NEU[2]; dctlLog150[109] = -vel_tgt_NED[2];    // NEU
            dctlLog150[110] = posPID[2][0]; dctlLog150[111] = posPID[2][1]; dctlLog150[112] = posPID[2][2]; dctlLog150[113] = -vel_tgt_NED[2]; dctlLog150[114] = posTgtNeuDctl[2];  // NEU
        }
        if (xy_ready) {
            xy_ready = false;
            if (!isXYreset && dctlStages == 3) {
                if (odomGoal_NEU_CCW[mOdomCur][4] >= 1 && odomGoal_NEU_CCW[mOdomCur][4] <= 4) {
                    isXYreset = true;
                    tmpIpos[0] = 0; tmpIpos[1] = 0;
                    tmpIvel[0] = 0; tmpIvel[1] = 0;
                }
            }
            if (dctlStages == 2) isXYreset = false;
            float rotPD[2] = {1,1};
            if (dctlStages == 3) {
                if (odomGoal_NEU_CCW[mOdomCur][4] >= 7 && odomGoal_NEU_CCW[mOdomCur][4] <= 8) {
                    tmpIpos[0] = 0; tmpIpos[1] = 0;
                    tmpIvel[0] = 0; tmpIvel[1] = 0;
                    rotPD[0] = 1.0;
                    rotPD[1] = 0.75;
                }
            }
            // Error pos (NEU) -------------------------------------------------------------
            posErrNeuDctl[0] = posTgtNeuDctl[0] - pos_neu[0];
            posErrNeuDctl[1] = posTgtNeuDctl[1] - pos_neu[1];
            float errSat = 0.250f; // 0.2m
            if (posErrNeuDctl[0] > errSat) posErrNeuDctl[0] = errSat; else if (posErrNeuDctl[0] < -errSat) posErrNeuDctl[0] = -errSat;
            if (posErrNeuDctl[1] > errSat) posErrNeuDctl[1] = errSat; else if (posErrNeuDctl[1] < -errSat) posErrNeuDctl[1] = -errSat;
            // PID xy ----------------------------------------------------------------------
            errSat = 0.25f; // 0.2m/s
            if (abs(vxy_world[0]) < errSat) { tmpIpos[0] += posErrNeuDctl[0] * dt10Hz; }
            if (abs(vxy_world[1]) < errSat) { tmpIpos[1] += posErrNeuDctl[1] * dt10Hz; }
            posPID[0][0] = kPIDposXY[0] * posErrNeuDctl[0] * rotPD[0];
            posPID[0][1] = kPIDposXY[1] * tmpIpos[0];
            posPID[0][2] = kPIDposXY[2] * (posErrNeuDctl[0] - posErrNeuDctlPrev[0]) / dt10Hz * rotPD[1];
            posPID[1][0] = kPIDposXY[0] * posErrNeuDctl[1] * rotPD[0];
            posPID[1][1] = kPIDposXY[1] * tmpIpos[1];
            posPID[1][2] = kPIDposXY[2] * (posErrNeuDctl[1] - posErrNeuDctlPrev[1]) / dt10Hz * rotPD[1];
            posErrNeuDctlPrev[0] = posErrNeuDctl[0];
            posErrNeuDctlPrev[1] = posErrNeuDctl[1];
            if (isnan(posPID[0][0]) || isinf(posPID[0][0])) posPID[0][0] = 0.0f;
            if (isnan(posPID[0][1]) || isinf(posPID[0][1])) posPID[0][1] = 0.0f;
            if (isnan(posPID[0][2]) || isinf(posPID[0][2])) posPID[0][2] = 0.0f;
            if (isnan(posPID[1][0]) || isinf(posPID[1][0])) posPID[1][0] = 0.0f;
            if (isnan(posPID[1][1]) || isinf(posPID[1][1])) posPID[1][1] = 0.0f;
            if (isnan(posPID[1][2]) || isinf(posPID[1][2])) posPID[1][2] = 0.0f;
            vxy_world[0] = posPID[0][0] + posPID[0][1] + posPID[0][2];
            vxy_world[1] = posPID[1][0] + posPID[1][1] + posPID[1][2];
            if (vxy_world[0] > errSat) vxy_world[0] = errSat; else if (vxy_world[0] < -errSat) vxy_world[0] = -errSat;
            if (vxy_world[1] > errSat) vxy_world[1] = errSat; else if (vxy_world[1] < -errSat) vxy_world[1] = -errSat;
            // World to body
            float c_yaw = cosdeg(rpy_enc_deg[2]);
            float s_yaw = sindeg(rpy_enc_deg[2]);
            vel_tgt_NED[0] = (vxy_world[0] * c_yaw) - (vxy_world[1] * s_yaw);
            vel_tgt_NED[1] = (vxy_world[0] * s_yaw) + (vxy_world[1] * c_yaw);
            // Error vel (NED) -------------------------------------------------------------
            velErrNedDctl[0] = vel_tgt_NED[0] - vel_ned[0];
            velErrNedDctl[1] = vel_tgt_NED[1] - vel_ned[1];
            if (velErrNedDctl[0] > errSat) velErrNedDctl[0] = errSat; else if (velErrNedDctl[0] < -errSat) velErrNedDctl[0] = -errSat;
            if (velErrNedDctl[1] > errSat) velErrNedDctl[1] = errSat; else if (velErrNedDctl[1] < -errSat) velErrNedDctl[1] = -errSat;
            // PID vx vy -------------------------------------------------------------------
            if (abs(Fxyz_NEU[0]) < maxFxy) { tmpIvel[0] += velErrNedDctl[0] * dt10Hz; }
            if (abs(Fxyz_NEU[1]) < maxFxy) { tmpIvel[1] += velErrNedDctl[1] * dt10Hz; }
            velPID[0][0] = kPIDvelXY[0] * velErrNedDctl[0] * rotPD[0];
            velPID[0][1] = kPIDvelXY[1] * tmpIvel[0];
            velPID[0][2] = kPIDvelXY[2] * (velErrNedDctl[0] - velErrNedDctlPrev[0]) / dt10Hz * rotPD[1];
            velPID[1][0] = kPIDvelXY[0] * velErrNedDctl[1] * rotPD[0];
            velPID[1][1] = kPIDvelXY[1] * tmpIvel[1];
            velPID[1][2] = kPIDvelXY[2] * (velErrNedDctl[1] - velErrNedDctlPrev[1]) / dt10Hz * rotPD[1];
            velErrNedDctlPrev[0] = velErrNedDctl[0];
            velErrNedDctlPrev[1] = velErrNedDctl[1];
            if (isnan(velPID[0][0]) || isinf(velPID[0][0])) velPID[0][0] = 0.0f;
            if (isnan(velPID[0][1]) || isinf(velPID[0][1])) velPID[0][1] = 0.0f;
            if (isnan(velPID[0][2]) || isinf(velPID[0][2])) velPID[0][2] = 0.0f;
            if (isnan(velPID[1][0]) || isinf(velPID[1][0])) velPID[1][0] = 0.0f;
            if (isnan(velPID[1][1]) || isinf(velPID[1][1])) velPID[1][1] = 0.0f;
            if (isnan(velPID[1][2]) || isinf(velPID[1][2])) velPID[1][2] = 0.0f;
            Fxyz_NEU[0] = velPID[0][0] + velPID[0][1] + velPID[0][2];
            Fxyz_NEU[1] = velPID[1][0] + velPID[1][1] + velPID[1][2];
            // Output saturation -----------------------------------------------------
            if (Fxyz_NEU[0] > maxFxy) Fxyz_NEU[0] = maxFxy; else if (Fxyz_NEU[0] < -maxFxy) Fxyz_NEU[0] = -maxFxy;
            if (Fxyz_NEU[1] > maxFxy) Fxyz_NEU[1] = maxFxy; else if (Fxyz_NEU[1] < -maxFxy) Fxyz_NEU[1] = -maxFxy;
            RPY_ref[0] = Fxyz_NEU[1]; // Roll = y
            RPY_ref[1] = Fxyz_NEU[0]; // Pitch = x
            // log 115~119 120~124 | 125~129 130~134
            dctlLog150[115] = velPID[0][0]; dctlLog150[116] = velPID[0][1]; dctlLog150[117] = velPID[0][2]; dctlLog150[118] = Fxyz_NEU[0]; dctlLog150[119] = vel_tgt_NED[0];
            dctlLog150[120] = velPID[1][0]; dctlLog150[121] = velPID[1][1]; dctlLog150[122] = velPID[1][2]; dctlLog150[123] = Fxyz_NEU[1]; dctlLog150[124] = vel_tgt_NED[1];
            dctlLog150[125] = posPID[0][0]; dctlLog150[126] = posPID[0][1]; dctlLog150[127] = posPID[0][2]; dctlLog150[128] = vel_tgt_NED[0]; dctlLog150[129] = posTgtNeuDctl[0];
            dctlLog150[130] = posPID[1][0]; dctlLog150[131] = posPID[1][1]; dctlLog150[132] = posPID[1][2]; dctlLog150[133] = vel_tgt_NED[1]; dctlLog150[134] = posTgtNeuDctl[1];
        }

        // Error yaw -------------------------------------------------------------
        float tmpYawErr = posTgtNeuDctl[3] - rpy_enc_deg[2];
        if (tmpYawErr > 180.0f) tmpYawErr -= 360; else if (tmpYawErr < -180.0f) tmpYawErr += 360;
        const float errSat = 30.0f; // deg
        if (tmpYawErr > errSat) tmpYawErr = errSat; else if (tmpYawErr < -errSat) tmpYawErr = -errSat;
        posErrNeuDctl[3] = tmpYawErr;
        // I reset on opposite direction
        if (dctlStages == 3) {
            if (!flagYawDone && (odomGoal_NEU_CCW[mOdomCur][4]==7 || odomGoal_NEU_CCW[mOdomCur][4]==8)) {
                if (flagYawPrev != 0 && odomGoal_NEU_CCW[mOdomCur][4] != flagYawPrev) { // 7=CCW 8=CW
                    tmpYPID = 0.0f;
                }
                flagYawPrev = odomGoal_NEU_CCW[mOdomCur][4];
                flagYawDone = true;
            }
        }
        else { flagYawDone = false; }
        // PID
        if (abs(Fyaw_CW) < maxFyaw) { tmpYPID += posErrNeuDctl[3] * dt100Hz; }
        yawPID[0] = kPIDyaw[0] * posErrNeuDctl[3];
        yawPID[1] = kPIDyaw[1] * tmpYPID;
        yawPID[2] = kPIDyaw[2] * (posErrNeuDctl[3] - posErrNeuDctlPrev[3]) / dt100Hz;
        posErrNeuDctlPrev[3] = posErrNeuDctl[3];
        if (isnan(yawPID[0]) || isinf(yawPID[0])) yawPID[0] = 0.0f;
        if (isnan(yawPID[1]) || isinf(yawPID[1])) yawPID[1] = 0.0f;
        if (isnan(yawPID[2]) || isinf(yawPID[2])) yawPID[2] = 0.0f;
        Fyaw_CW = -(yawPID[0] + yawPID[1] + yawPID[2]);
        if (Fyaw_CW > maxFyaw) Fyaw_CW = maxFyaw; else if (Fyaw_CW < -maxFyaw) Fyaw_CW = -maxFyaw;
        RPY_ref[2] = Fyaw_CW;
        // log 135~139
        dctlLog150[135] = yawPID[0]; dctlLog150[136] = yawPID[1]; dctlLog150[137] = yawPID[2]; dctlLog150[138] = -Fyaw_CW; dctlLog150[139] = posTgtNeuDctl[3]; // CCW
        
        // Set the output --------------------------------------------------------
        rc_cmd[0] = RCcenter + RPY_ref[0] + cmd_TRP_slope[1];   // roll+=right      (1500=mid)
        rc_cmd[1] = RCcenter + RPY_ref[1] + cmd_TRP_slope[2];   // pitch+=forward   (1500=mid)
        rc_cmd[3] = RCcenter + RPY_ref[2];                      // yaw+=CW          (1500=mid)
        rc_cmd[2] = Throttle_Ft + cmd_TRP_slope[0];             // throttle+=up     (1000-2000)
        rc_cmd[4] = 1000;                                       // AUX1: arm        (on: 900-1200)
        if (gestart) {                      // GE on landing
            rc_cmd[0] += ge_TRPY[1];        // roll
            rc_cmd[1] += ge_TRPY[2];        // pitch
            rc_cmd[3] += ge_TRPY[3];        // yaw
            rc_cmd[2] += ge_TRPY[0];        // throttle
        }
        rc_cmd[0] += OFF_ROLL_CMD;          // offset
        rc_cmd[1] += OFF_PITCH_CMD;         // offset
        dctlStages_prev = dctlStages;

        // GE
        last_applied_pwm_ndo = rc_cmd[2];
        const float incOff = 10.0f; //10
        if (abs(cmd_TRP_slope[0]) <= incOff) cmd_TRP_slope[0] = 0.0;
        else if (cmd_TRP_slope[0] > incOff) cmd_TRP_slope[0] -= incOff;
        else if (cmd_TRP_slope[0] < -incOff) cmd_TRP_slope[0] += incOff;
        if (abs(cmd_TRP_slope[1]) <= incOff) cmd_TRP_slope[1] = 0.0;
        else if (cmd_TRP_slope[1] > incOff) cmd_TRP_slope[1] -= incOff;
        else if (cmd_TRP_slope[1] < -incOff) cmd_TRP_slope[1] += incOff;
        if (abs(cmd_TRP_slope[2]) <= incOff) cmd_TRP_slope[2] = 0.0;
        else if (cmd_TRP_slope[2] > incOff) cmd_TRP_slope[2] -= incOff;
        else if (cmd_TRP_slope[2] < -incOff) cmd_TRP_slope[2] += incOff;
    };
    auto slopeStartFunc = [&]() {
        const float RPtol = 3.5f;
        const float maxRP = 300;
        const float maxT = 75;
        const float incStep = 1.0f;
        // ---
        cmd_TRP_slope[0] = 0.0f;
        if (rpy_enc_deg[0] > RPtol*3) cmd_TRP_slope[1] -= incStep*7.5;
        else if (rpy_enc_deg[0] > RPtol) cmd_TRP_slope[1] -= incStep;
        else if (rpy_enc_deg[0] < -RPtol*3) cmd_TRP_slope[1] += incStep*7.5;
        else if (rpy_enc_deg[0] < -RPtol) cmd_TRP_slope[1] += incStep;
        if (rpy_enc_deg[1] > RPtol*3) cmd_TRP_slope[2] -= incStep*7.5;
        else if (rpy_enc_deg[1] > RPtol) cmd_TRP_slope[2] -= incStep;
        else if (rpy_enc_deg[1] < -RPtol*3) cmd_TRP_slope[2] += incStep*7.5;
        else if (rpy_enc_deg[1] < -RPtol) cmd_TRP_slope[2] += incStep;
        // ---
        if (cmd_TRP_slope[0] > maxT) cmd_TRP_slope[0] = maxT;
        else if (cmd_TRP_slope[0] < -maxT) cmd_TRP_slope[0] = -maxT;
        if (cmd_TRP_slope[1] > maxRP) cmd_TRP_slope[1] = maxRP;
        else if (cmd_TRP_slope[1] < -maxRP) cmd_TRP_slope[1] = -maxRP;
        if (cmd_TRP_slope[2] > maxRP) cmd_TRP_slope[2] = maxRP;
        else if (cmd_TRP_slope[2] < -maxRP) cmd_TRP_slope[2] = -maxRP;
        // ---
        rc_cmd[0] = RCcenter + cmd_TRP_slope[1]; // roll
        rc_cmd[1] = RCcenter + cmd_TRP_slope[2]; // pitch
        rc_cmd[3] = RCcenter; // yaw
        rc_cmd[2] = 1000 + cmd_TRP_slope[0]; // throttle (1000-2000)
        rc_cmd[4] = 1000; // AUX1: arm (on: 900-1200)
        // ---
        if (abs(rpy_enc_deg[0]) <= RPtol && abs(rpy_enc_deg[1]) <= RPtol) {
            cmd_TRP_slope[0] *= 0.75;
            cmd_TRP_slope[1] *= 0.75;
            cmd_TRP_slope[2] *= 0.75;
            SLOPE_START = false;
        }
    };

    for (int i = 0; i < 16; i++) { 
        if (i == 2) rc_cmd[i] = 1000;        // lo: throttle
        else if (i == 4) rc_cmd[i] = 2000;   // hi: AUX1 arm (900-1200)
        else if (i == 5) rc_cmd[i] = 2000;   // hi: AUX2 angle (1400-2100)
        else if (i == 6) rc_cmd[i] = 2000;   // hi: AUX2 beep (900-1200)
        else rc_cmd[i] = 1500;               // center
    }
    const us0 PERIODus(10000); // 100Hz, posctrl + crsfFC
    auto nextP = clock0::now() + PERIODus;
    ticFTsec();
    cout << ">>> [OK] Main (100)" << endl;

    /* --------------------------------------------------- */
    SLOPE_START     = false;
    print_debug     = false;
    bool odom_debug = false;
    /******************* Motion Planner  *******************/
    if (!odom_debug) {
        odomHover(5);

        // odomMoveIncWorldY(-3.5,5);
        // odomRotIncWorldYaw(-90,4);
        // odomMoveIncWorldX(3.5,1);
        // odomRotIncWorldYaw(-90,4);
        // odomMoveIncWorldY(3.5,1);
        // odomRotIncWorldYaw(-90,4);
        // odomMoveIncWorldX(-3.5,1);
        
        // odomHover(2);
    }
    /************************ (^_^) ************************/
    
    while (true) {
        if (startMission) {
            switch (mot_seq) {
                case 0: { motDisArm(); mot_seq++; ticFTsec(); break; }
                case 1: { if (dctlStages == 1 || SLOPE_START) { motArm(); mot_seq++; ticFTsec(); } break; }
                case 2: { 
                    if (tocFTsec() > 0.001f && SLOPE_START) { slopeStartFunc(); } 
                    if (!SLOPE_START) { mot_seq++; ticFTsec(); }
                    break; 
                }
                case 3: { if (tocFTsec() > 1.0f) { motFlightPID(); flightTime = round((tocFTsec() - 2.0f) * 10.0f) / 10.0f; } break; }
            }
        }
        else { 
            if (flightTime == 0 && emgcCode == 0) {
                motDisArm(); mot_seq=0;
                if (nOdom > 0 && tocFTsec() > 2.0f) {
                    if (fps_main_th[0] > 75 && fps_main_th[1] > 75 && fps_main_th[2] > 11) {
                        if (fps_imu > 75 && fps_of > 7 && fps_alt > 22 && fps_tof1 > 11 && fps_adc > 75) {
                            startMission = true; 
                        }
                    }
                }
                else if (nOdom <= 0 && frs_print) { frs_print = false; cout << " No mission ..." << endl; }
            }
            else if (emgcCode <= -11 && emgcCode >= -16) { motDisArm(); }
            else {
                if (mot_seq == 3) { mot_seq++; rc_cmd[2] = 1000; ticFTsec(); }
                else if (mot_seq == 4) {
                    if (tocFTsec() > 0.05f) { motDisArm(); }
                    if (tocFTsec() > 1.05f) { mot_seq++; startDctlLog = false; set_buz_led(1,1,1,1,0); delay(50); set_buz_led(); delay(50); set_buz_led(1,1,1,1,0); delay(50); set_buz_led(); }
                }
            }
        }

        // write to FC constantly
        if (!digitalRead(PIN_WPI_BUTF) || !digitalRead(PIN_WPI_BUTR)) emgcCode = -15;
        if (emgcCode == -15) motDisArm();
        dctlLog150[34]=rc_cmd[0]; dctlLog150[35]=rc_cmd[1]; dctlLog150[36]=rc_cmd[2]; dctlLog150[37]=rc_cmd[3];
        dctlLog150[38]=rc_cmd[4]; dctlLog150[39]=rc_cmd[5]; dctlLog150[40]=rc_cmd[6];
        for (int i = 0; i < 16; i++) { ch_us_mot[i] = rc_cmd[i]; }
        int len = build_crsf_frame(frame_mot, ch_us_mot);
        write(fd_crsf_fc, frame_mot, len); 

        //------------------------------
        if (!digitalRead(PIN_WPI_BUTF) || !digitalRead(PIN_WPI_BUTR)) emgcCode = -15;
        this_thread::sleep_until(nextP);
        nextP += PERIODus;
        cnt_main_th[0]++;

        //------------------------------
        if (odom_debug) {
            if (read_of && !digitalRead(PIN_WPI_BUTR)) {
                cout<<"Ended..."<<endl;
                set_buz_led(1,-1,-1,-1,0); delay(50);
                set_buz_led(0,-1,-1,-1,0); delay(2000);
            }
            if (!read_of && !digitalRead(PIN_WPI_BUTR)) {
                cout<<"Started..."<<endl;
                read_of=true;
                set_buz_led(1,0,0,0,1); delay(50);
                set_buz_led(0,0,0,0,1); delay(2000);
            }
            if (read_of) set_buz_led(-1,-1,-1,-1,1); 
            else set_buz_led(-1,1,1,1,-1);
        }   
    }
}