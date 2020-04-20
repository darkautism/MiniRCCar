#include <SoftwareSerial.h>
#include <Arduino.h>
#include <Servo.h>

#include "power.h"

//SoftwareSerial dongle(5, 4); // RX, TX
//HardwareSerial dongle(1);

Servo myservo;
#define dongle Serial

#define PIN_SERVO 3
#define PIN_STANDBY 4
#define PIN_MONTOR_A2 5
#define PIN_MONTOR_A1 6

void setup()
{
    PowerUp();
    pinMode(PIN_STANDBY, OUTPUT);
    pinMode(PIN_MONTOR_A2, OUTPUT);
    pinMode(PIN_MONTOR_A1, OUTPUT);
    digitalWrite(PIN_STANDBY, LOW);
    digitalWrite(PIN_MONTOR_A2, LOW);
    digitalWrite(PIN_MONTOR_A1, LOW);

    myservo.attach(PIN_SERVO);
    //Serial.begin(9600);
    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB port only
    }
    dongle.begin(38400); // custom HC05DS4's baud is 38400
}

char buf[79];
int bytes = 0;

// 9 bytes
typedef struct
{
    uint8_t L_X;
    uint8_t L_Y;
    uint8_t R_X;
    uint8_t R_Y;
    // DPAD			0b1111
    // SQUARE_PAD	0b10000
    // X_PAD		0b100000
    // O_PAD		0b1000000
    // TRIANGLE_PAD	0b10000000
    uint8_t PAD;
    // L1		0b0001
    // R1		0b0010
    // L2		0b0100
    // R2		0b1000
    // SHARE	0b10000
    // OPTION	0b100000
    // L3		0b1000000
    // R3		0b10000000
    uint8_t BTN;
    uint8_t TOUCHPS;
    uint8_t L2Analogy;
    uint8_t R2Analogy;
} report0x01;

// 77 bytes
typedef struct
{
    uint8_t unknow;
    uint8_t ReportID;
    uint8_t L_X;
    uint8_t L_Y;
    uint8_t R_X;
    uint8_t R_Y;
    // DPAD			0b1111
    // SQUARE_PAD	0b10000
    // X_PAD		0b100000
    // O_PAD		0b1000000
    // TRIANGLE_PAD	0b10000000
    uint8_t PAD;
    // L1		0b0001
    // R1		0b0010
    // L2		0b0100
    // R2		0b1000
    // SHARE	0b10000
    // OPTION	0b100000
    // L3		0b1000000
    // R3		0b10000000
    uint8_t BTN;
    uint8_t TOUCHPS;
    uint8_t L2Analogy;
    uint8_t R2Analogy;
    byte Timestamp[2];
    uint8_t Battery;
    uint16_t AngularVelocity[3];
    uint16_t Acceleration[3];
    byte unknow99[51];
} report0x11;

report0x01 *r01;
report0x11 *r11;

int cur_angle = 90;
int prev_angle = 90;
int prev_power = 0;
int cur_power = 0;
int X = 128, Y = 128;
long prev_got_time = 0;

void loop()
{
    PowerTask();
ReLoad:
    if (dongle.available())
    {
        prev_got_time = millis();
        bytes = dongle.readBytes(buf, 2);
        if (bytes != 2 && buf[0] != 0xa1) // filter it if it's we not handle packet
            goto ReLoad;
        switch (buf[1])
        {
        case 0x01:
            bytes = dongle.readBytes(buf, 9);
            if (bytes != 9)
            {
                return;
            }
            r01 = (report0x01 *)buf;
            X = r01->L_X;
            Y = r01->R_Y;
            break;

        case 0x11:
            bytes = dongle.readBytes(buf, 77);
            if (bytes != 77)
            {
                return;
            }
            r11 = (report0x11 *)buf;
            X = r11->L_X;
            Y = r11->R_Y;
            break;
        }
    }
    else
    {
        if (millis() - prev_got_time > 1000) // DC
            X = Y = 128;
    }

    cur_angle = 90 + (X - 128) / 3;
    cur_power = Y - 128;

    if (cur_angle != prev_angle)
    {
        prev_angle = cur_angle;
        myservo.write(cur_angle);
    }

    if (prev_power != cur_power)
    {
        prev_power = cur_power;
        if (cur_power <= 3 && cur_power >= -3)
        {
            digitalWrite(PIN_STANDBY, LOW);
        }
        else
        {
            digitalWrite(PIN_STANDBY, HIGH);
            if (cur_power > 0)
            {
                digitalWrite(PIN_MONTOR_A1, HIGH);
                analogWrite(PIN_MONTOR_A2, 256 + cur_power * 2);
            }
            else
            {
                digitalWrite(PIN_MONTOR_A1, LOW);
                analogWrite(PIN_MONTOR_A2, cur_power * 2);
            }
        }
    }
}