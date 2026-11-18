#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

bool dmpReady = false;
uint8_t devStatus;
uint8_t fifoBuffer[64]; 
Quaternion q;           
VectorFloat gravity;    
float ypr[3];           // [yaw, pitch, roll] 角度格納用

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2Cクロック
    #endif

    Serial.begin(9600);
    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    // キャリブレーションオフセット
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        dmpReady = true;
    }
}

void loop() {
    if (!dmpReady) return;

    // DMPのデータ取得
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        float roll  = ypr[2] * 180/M_PI; // 左右の傾き
        float pitch = ypr[1] * 180/M_PI; // 前後の傾き

        // 符号をマイナスにすることで、右に傾けたときに左(-X)へ動くように反転
        float x = -((roll / 45.0) * 5.0);   //90度で5ぐらいになる(僕のmpu6050の場合)
        float y = (pitch / 45.0) * 5.0;

        Serial.print("X:");
        Serial.print(x, 1);
        Serial.print(",Y:");
        Serial.println(y, 1);
    }
}
