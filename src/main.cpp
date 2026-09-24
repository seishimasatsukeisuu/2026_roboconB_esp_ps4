#include <Arduino.h>
#include <CAN.h>
#include <PS4Controller.h>
#include <math.h>

// PS4入力
int lx;
int ly;
int up_straight;
int down_straight;
int r_straight;
int l_straight;
int l2;
int r2;

// ベル直入力
int circleState = false; // pwm 2999
int lastcircleState = false;
int triangleState = false; // pwm 500
int lasttriangleState = false;

// CAN pwm送信用
int16_t motor[4] = {0};

// CANデータ計算用
float vx;
float vy;
float rot;

static uint32_t last_can_tx = 0;

void setup()
{
  Serial.begin(115200);

  while (!Serial)
    ;
  PS4.begin("00:02:5b:00:a5:ac");

  CAN.setPins(4, 5);      // 16,17ピンはつかえない(4,5ピンは使えた)
  if (!CAN.begin(1000E3)) // 1000kbpsで開始
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }

  volatile uint32_t *pREG_IER = (volatile uint32_t *)0x3ff6b010;
  *pREG_IER &= ~(uint8_t)0x10;

  Serial.println("Ready");
}

void loop()
{
  if (PS4.isConnected())
  {
    lx = -PS4.LStickX();
    ly = PS4.LStickY();
    up_straight = PS4.Up();
    down_straight = PS4.Down();
    r_straight = PS4.Right();
    l_straight = PS4.Left();
    l2 = -PS4.L2Value();
    r2 = -PS4.R2Value();

    circleState = PS4.data.button.circle;
    triangleState = PS4.data.button.triangle;

    if (circleState && !lastcircleState)
    {
      CAN.beginPacket(0x102);

      CAN.write(1);

      CAN.endPacket();
    }
    lastcircleState = circleState;

    if (triangleState && !lasttriangleState)
    {
      CAN.beginPacket(0x102);

      CAN.write(2);

      CAN.endPacket();
    }
    lasttriangleState = triangleState;

    vx = ly;
    vy = lx;

    if (abs(vx) < 10)
      vx = 0;
    if (abs(vy) < 10)
      vy = 0;
    if (abs(l2) < 10)
      l2 = 0;
    if (abs(r2) < 10)
      r2 = 0;
    if (up_straight)
    {
      vx = 100;
      vy = 0;
    }
    if (down_straight)
    {
      vx = -100;
      vy = 0;
    }
    if (r_straight)
    {
      vx = 0;
      vy = -100;
    }
    if (l_straight)
    {
      vx = 0;
      vy = 100;
    }

    constexpr float INV_SQRT2 = 0.70710678f;
    rot = (r2 - l2) * 0.25f;
    float gain = 20.0f;

    float v1 = ((-vx + vy) * INV_SQRT2 + rot) * gain;
    float v2 = ((vx + vy) * INV_SQRT2 + rot) * gain;
    float v3 = ((-vx - vy) * INV_SQRT2 + rot) * gain;
    float v4 = ((vx - vy) * INV_SQRT2 + rot) * gain;

    float v[4] = {v1, v2, v3, v4};

    for (int i = 0; i < 4; i++)
    {
      motor[i] = (int16_t)constrain(v[i], -2999, 2999);
    }
  }

  if (!PS4.isConnected())
  {
    for (int i = 0; i < 4; i++)
      motor[i] = 0;
  }

  // CAN送信
  if (micros() - last_can_tx >= 20000)
  {
    last_can_tx = micros();

    CAN.beginPacket(0x103);

    for (int i = 0; i < 4; i++)
    {
      CAN.write((uint8_t)(motor[i] >> 8));
      CAN.write((uint8_t)(motor[i] & 0xFF));
    }

    CAN.endPacket();
  }
}