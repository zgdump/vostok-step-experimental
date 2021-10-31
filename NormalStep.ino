////////////////////////////////////////////////////////////////////////////////
// Библиотеки


#include <ServoDriverSmooth.h>
#include "NewPwmServoDriver.h"
#include <PPMReader.h>


////////////////////////////////////////////////////////////////////////////////
// Inline Константы

#define LIGHT_PIN  6
#define CAM1_PIN   9
#define CAM2_PIN   30
  
#define DOCKING_MODE_UNDOCK   0
#define DOCKING_MODE_NEUTRAL  1
#define DOCKING_MODE_DOCK     2
  
#define DOCKING_MODE_UNDOCK_VALUE   2200
#define DOCKING_MODE_NEUTRAL_VALUE  1500
#define DOCKING_MODE_DOCK_VALUE     800


////////////////////////////////////////////////////////////////////////////////
// Макросы

#define RAD_TO_ANGLE(x) (x * 180.0 / M_PI)


////////////////////////////////////////////////////////////////////////////////
// Классы, структуры, енамы

// Описание ноги
struct Leg {
  ServoDriverSmooth *a;        // Серво на платформе
  ServoDriverSmooth *b;        // Серво на ноге
  ServoDriverSmooth *r;        // Поворотная серва
  NewPwmServoDriver *d;  // Стыковочная серва

  uint8_t pinA;  // Пин сервы на платформе
  uint8_t pinB;  // Пин серы на ноге
  uint8_t pinR;  // Пин поворотной сервы
  uint8_t pinD;  // Пин стыковочной сервы
};


////////////////////////////////////////////////////////////////////////////////
// Состояние термостата

PPMReader ppm(/* pin */ 2, 10);

Leg* legs[2] = { nullptr, nullptr };
uint8_t currentLeg = 0;
uint8_t previousLeg = 255;
uint32_t servoTimer = 0;
uint8_t dockingModes[] = { -1, -1 };

bool lightEnabled = false;

////////////////////////////////////////////////////////////////////////////////
// Lifecycle

void setup() {

  Serial.begin(115200);

  // Настройка пинов
  pinMode(LIGHT_PIN, OUTPUT);   // Свет
  pinMode(CAM1_PIN, OUTPUT);    // Камеры 1
  pinMode(CAM2_PIN, OUTPUT);    // Камера 2

  // Стыковочная серва
  auto dockingServo = new NewPwmServoDriver();
  dockingServo->begin();
  dockingServo->setPWMFreq(60);

  // Настройка первой ноги
  legs[0] = new Leg();
  legs[0]->pinA = 0;
  legs[0]->a = newServo(legs[0]->pinA, 45);
  legs[0]->pinB = 1;
  legs[0]->b = newServo(legs[0]->pinB, 45);
  legs[0]->pinR = 2;
  legs[0]->r = newServo(legs[0]->pinR, 0);
  legs[0]->pinD = 3;
  legs[0]->d = dockingServo;

  // Настройка второй ноги
  legs[1] = new Leg();
  legs[1]->pinA = 4;
  legs[1]->a = newServo(legs[1]->pinA, 45);
  legs[1]->pinB = 5;
  legs[1]->b = newServo(legs[1]->pinB, 45);
  legs[1]->pinR = 6;
  legs[1]->r = newServo(legs[1]->pinR, 0);
  legs[1]->pinD = 7;
  legs[1]->d = dockingServo;

  // Инициализация таймеров
  servoTimer = millis();
}

void loop() {

  // Жизнедеятельность серв
  if (millis() - servoTimer >= 20) {
    servoTimer += 20;

    legs[0]->a->tickManual();
    legs[0]->b->tickManual();
    legs[0]->r->tickManual();

    legs[1]->a->tickManual();
    legs[1]->b->tickManual();
    legs[1]->r->tickManual();
  }

  // Обработка состояния пульта

  // Свет
  bool lightEnabled = readTwoStatePpm(6) == 1;
  digitalWrite(LIGHT_PIN, lightEnabled ? HIGH : LOW);

  // Переключение ног
  previousLeg = currentLeg;
  currentLeg = readTwoStatePpm(8);

  // Переключение камер
  if (previousLeg != currentLeg) {
    digitalWrite(CAM1_PIN, currentLeg == 0 ? HIGH : LOW);
    digitalWrite(CAM2_PIN, currentLeg == 1 ? HIGH : LOW);
  }

  // Переключатель режима стыковки
  int dockingMode = readThreeStatePpm(7);
  if (dockingMode != dockingModes[currentLeg]) {

    // Устанавливаем режим стыковки для текущей сервы
    dockingModes[currentLeg] = dockingMode;
    setServoDockingMode(legs[currentLeg], dockingMode);

    // Если поменяли ногу, отключаем предыдущую
    if (previousLeg != currentLeg) {
      dockingModes[previousLeg] = 1;
      setServoDockingMode(legs[previousLeg], DOCKING_MODE_NEUTRAL);
    }
  }

  // Поворотная серва
  int rotateServoAngle = readPpm(1, 0, 160);
  legs[currentLeg]->r->setTargetDeg(rotateServoAngle);

  // Раздвижение ног
  const float a = 8.0;       // Расстояние между сервами около платформы
  const float c = 16.0;      // Длина ноги
  const float min_b = 24.0;  // Минимальное расстояние между сервами у ног
  const float max_b = 38.0;  // Максимальное расстояние между сервами у ног
  const float min_h = 10.0;  // Минимальная высота между сервами
  const float max_h = 04.5;  // Максимальная высота между сервами

  float position_value  = (float) ppm.latestValidChannelValue(2, 0);

  float b = mapf(position_value, 1000.0, 2000.0, min_b, max_b);
  float h = mapf(position_value, 1000.0, 2000.0, min_h, max_h);

  int r = round(RAD_TO_ANGLE(atan(h / c)));

  Serial.println("Angle " + String(r));

  legs[currentLeg]->a->setTargetDeg(r);
  legs[currentLeg]->b->setTargetDeg(r);
}


////////////////////////////////////////////////////////////////////////////////
// Servo

ServoDriverSmooth *newServo(uint8_t pin, uint8_t start_angle) {
  auto servo = new ServoDriverSmooth();

  servo->setSpeed(200);
  servo->setAccel(0.2);

  servo->attach(pin, 150, 600);
  servo->smoothStart();

  servo->setAutoDetach(false);
  servo->setTargetDeg(start_angle);

  return servo;
}

void setServoDockingMode(Leg *leg, uint8_t mode) {
  if (mode == DOCKING_MODE_UNDOCK) {
    leg->d->writeMicroseconds(leg->pinD, DOCKING_MODE_UNDOCK_VALUE);
  } else if (mode == DOCKING_MODE_NEUTRAL) {
    leg->d->writeMicroseconds(leg->pinD, DOCKING_MODE_NEUTRAL_VALUE);
  } else if (mode == DOCKING_MODE_DOCK) {
    leg->d->writeMicroseconds(leg->pinD, DOCKING_MODE_DOCK_VALUE);
  }
}


////////////////////////////////////////////////////////////////////////////////
// PPM

uint16_t readPpm(uint8_t channel, const uint16_t range_start, const uint16_t range_end) {
  int v  = ppm.latestValidChannelValue(channel, 0);
  int vf = map(v, 1000, 2000, range_start, range_end);
  return vf;
}

uint8_t readTwoStatePpm(uint8_t channel) {
  int vf = readPpm(channel, 0, 100);
  if (0 <= vf && vf <= 5) {
    return 0;
  } else if (95 <= vf && vf <= 100) {
    return 1;
  }
}

uint8_t readThreeStatePpm(uint8_t channel) {
  int vf = readPpm(channel, 0, 100);
  if (0 <= vf && vf <= 5) {
    return 0;
  } else if (45 <= vf && vf <= 55) {
    return 1;
  } else if (95 <= vf && vf <= 100) {
    return 2;
  }
}


////////////////////////////////////////////////////////////////////////////////
// Math

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*

  void loop() {



  // Мотор
  //int v0 = ppm.latestValidChannelValue(2, 0);
  //float b  = map(v0, 1000.0, 2000.0, 20.0, 75.0);

  servos[0].setTargetDeg(45);
  servos[1].setTargetDeg(45);
  //servos[3].setTargetDeg(b);
  //servos[4].setTargetDeg(b);
  //Serial.println("C: " + String(c) + ", Xf: " + String (xf) + ", Yf: " + String(yf) + ", A1: " + String(aa) + ", A2: " + String(ab));

  // Фиксатор
  //  static int mode1 = -1;
  //  int v1 = ppm.latestValidChannelValue(6, 0);
  //  int a1 = map(v1, 1000, 2000, 0, 100);
  //  if (0 <= a1 && a1 <= 5 && mode1 != 0) {
  //    dockingServo.setPWM(dockingServosPins[0], 4020, 0);
  //    mode1 = 0;
  //  } else if (45 <= a1 && a1 <= 55 && mode1 != 1) {
  //    dockingServo.setPWM(dockingServosPins[0], 0, 0);
  //    mode1 = 1;
  //  } else if (95 <= a1 && a1 <= 100 && mode1 != 2) {
  //    dockingServo.setPWM(dockingServosPins[0], 4020, 0);
  //    mode1 = 2;
  //  }

  // Фиксатор
  //  static int mode2 = -1;
  //  int v2 = ppm.latestValidChannelValue(7, 0);
  //  int a2 = map(v2, 1000, 2000, 0, 100);
  //  if (0 <= a2 && a2 <= 5 && mode2 != 0) {
  //    dockingServo.setPWM(dockingServosPins[1], 2000, 0);
  //    mode2 = 0;
  //  } else if (45 <= a2 && a2 <= 55 && mode2 != 1) {
  //    dockingServo.setPWM(dockingServosPins[1], 0, 0);
  //    mode2 = 1;
  //  } else if (95 <= a2 && a2 <= 100 && mode2 != 2) {
  //    dockingServo.setPWM(dockingServosPins[1], 4000, 0);
  //    mode2 = 2;
  //  }
  }

*/
