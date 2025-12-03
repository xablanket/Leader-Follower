#include <Arduino.h>

// 引脚定义
#define LEFT_MOTOR_PWM 10
#define LEFT_MOTOR_DIR 16
#define RIGHT_MOTOR_PWM 9
#define RIGHT_MOTOR_DIR 15

#define BUMPER_LEFT 4
#define BUMPER_RIGHT 5

#define LINE_SENSOR_LEFT 12
#define LINE_SENSOR_CENTER_LEFT 18
#define LINE_SENSOR_CENTER 20
#define LINE_SENSOR_CENTER_RIGHT 21
#define LINE_SENSOR_RIGHT 22

// 按钮引脚定义
#define BUTTON_A 14
#define BUTTON_B 30
#define BUTTON_C 17

// 按钮状态变量
bool lastButtonAState = HIGH;
bool lastButtonBState = HIGH;
bool lastButtonCState = HIGH;
bool currentButtonAState = HIGH;
bool currentButtonBState = HIGH;
bool currentButtonCState = HIGH;

// 控制参数
#define TARGET_DISTANCE 300
#define BASE_SPEED 30
#define MAX_SPEED 120
#define MIN_SPEED 0
#define FILTER_SAMPLES 5       // 平均滤波窗口大小
#define BUMPER_THRESHOLD 80    // 碰撞传感器阈值
#define LINE_THRESHOLD 35      // 线传感器阈值

// 传感器数据结构
struct SensorData {
  int bumperLeft;
  int bumperRight;
  int lineLeft;
  int lineCenterLeft;
  int lineCenter;
  int lineCenterRight;
  int lineRight;
};

// 滤波数据结构
struct FilterData {
  int bumperLeft[FILTER_SAMPLES];
  int bumperRight[FILTER_SAMPLES];
  int lineLeft[FILTER_SAMPLES];
  int lineCenterLeft[FILTER_SAMPLES];
  int lineCenter[FILTER_SAMPLES];
  int lineCenterRight[FILTER_SAMPLES];
  int lineRight[FILTER_SAMPLES];
  
  int bumperLeftIndex = 0;
  int bumperRightIndex = 0;
  int lineLeftIndex = 0;
  int lineCenterLeftIndex = 0;
  int lineCenterIndex = 0;
  int lineCenterRightIndex = 0;
  int lineRightIndex = 0;
};

SensorData currentSensors;
FilterData filterData;

// 环境光基准
int ambientLightBumperLeft = 0;
int ambientLightBumperRight = 0;
int ambientLightLineLeft = 0;
int ambientLightLineCenterLeft = 0;
int ambientLightLineCenter = 0;
int ambientLightLineCenterRight = 0;
int ambientLightLineRight = 0;

// 电机初始化
void initMotors() {
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  
  digitalWrite(LEFT_MOTOR_DIR, LOW);
  digitalWrite(RIGHT_MOTOR_DIR, LOW);
}

// 按钮初始化
void initButtons() {
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
}

// 环境光校准
void calibrateAmbientLight() {
  delay(1000);
  
  int sumBumperLeft = 0, sumBumperRight = 0;
  int sumLineLeft = 0, sumLineCenterLeft = 0, sumLineCenter = 0;
  int sumLineCenterRight = 0, sumLineRight = 0;
  
  const int samples = 10;
  
  for (int i = 0; i < samples; i++) {
    // 校准碰撞传感器
    sumBumperLeft += analogRead(BUMPER_LEFT);
    sumBumperRight += analogRead(BUMPER_RIGHT);
    
    // 校准线传感器
    sumLineLeft += analogRead(LINE_SENSOR_LEFT);
    sumLineCenterLeft += analogRead(LINE_SENSOR_CENTER_LEFT);
    sumLineCenter += analogRead(LINE_SENSOR_CENTER);
    sumLineCenterRight += analogRead(LINE_SENSOR_CENTER_RIGHT);
    sumLineRight += analogRead(LINE_SENSOR_RIGHT);
    
    delay(50);
  }
  
  // 计算每个传感器的环境光基准
  ambientLightBumperLeft = sumBumperLeft / samples;
  ambientLightBumperRight = sumBumperRight / samples;
  ambientLightLineLeft = sumLineLeft / samples;
  ambientLightLineCenterLeft = sumLineCenterLeft / samples;
  ambientLightLineCenter = sumLineCenter / samples;
  ambientLightLineCenterRight = sumLineCenterRight / samples;
  ambientLightLineRight = sumLineRight / samples;
}

// 移动时的平均滤波函数
int movingAverageFilter(int newValue, int* buffer, int& index) {
  buffer[index] = newValue;
  index = (index + 1) % FILTER_SAMPLES;
  
  long sum = 0;
  for (int i = 0; i < FILTER_SAMPLES; i++) {
    sum += buffer[i];
  }
  return sum / FILTER_SAMPLES;
}

// 传感器读取
void readSensors() {
  // 读取并处理碰撞传感器
  int rawBumperLeft = max(0, analogRead(BUMPER_LEFT) - ambientLightBumperLeft);
  int rawBumperRight = max(0, analogRead(BUMPER_RIGHT) - ambientLightBumperRight);
  
  currentSensors.bumperLeft = movingAverageFilter(rawBumperLeft, 
                                                 filterData.bumperLeft, 
                                                 filterData.bumperLeftIndex);
  currentSensors.bumperRight = movingAverageFilter(rawBumperRight, 
                                                  filterData.bumperRight, 
                                                  filterData.bumperRightIndex);
  
  // 碰撞传感器阈值处理
  if (currentSensors.bumperLeft < BUMPER_THRESHOLD) currentSensors.bumperLeft = 0;
  if (currentSensors.bumperRight < BUMPER_THRESHOLD) currentSensors.bumperRight = 0;
  
  // 读取并处理所有线传感器
  currentSensors.lineLeft = movingAverageFilter(
    max(0, analogRead(LINE_SENSOR_LEFT) - ambientLightLineLeft),
    filterData.lineLeft, filterData.lineLeftIndex);
    
  currentSensors.lineCenterLeft = movingAverageFilter(
    max(0, analogRead(LINE_SENSOR_CENTER_LEFT) - ambientLightLineCenterLeft),
    filterData.lineCenterLeft, filterData.lineCenterLeftIndex);
    
  currentSensors.lineCenter = movingAverageFilter(
    max(0, analogRead(LINE_SENSOR_CENTER) - ambientLightLineCenter),
    filterData.lineCenter, filterData.lineCenterIndex);
    
  currentSensors.lineCenterRight = movingAverageFilter(
    max(0, analogRead(LINE_SENSOR_CENTER_RIGHT) - ambientLightLineCenterRight),
    filterData.lineCenterRight, filterData.lineCenterRightIndex);
    
  currentSensors.lineRight = movingAverageFilter(
    max(0, analogRead(LINE_SENSOR_RIGHT) - ambientLightLineRight),
    filterData.lineRight, filterData.lineRightIndex);
  
  // 线传感器阈值处理
  if (currentSensors.lineLeft < LINE_THRESHOLD) currentSensors.lineLeft = 0;
  if (currentSensors.lineCenterLeft < LINE_THRESHOLD) currentSensors.lineCenterLeft = 0;
  if (currentSensors.lineCenter < LINE_THRESHOLD) currentSensors.lineCenter = 0;
  if (currentSensors.lineCenterRight < LINE_THRESHOLD) currentSensors.lineCenterRight = 0;
  if (currentSensors.lineRight < LINE_THRESHOLD) currentSensors.lineRight = 0;
}

// 串口输出函数
void handleSerialOutput() {
  // 读取当前按钮状态
  currentButtonAState = digitalRead(BUTTON_A);
  currentButtonBState = digitalRead(BUTTON_B);
  currentButtonCState = digitalRead(BUTTON_C);
  
  // 按下按钮A
  if (lastButtonAState == HIGH && currentButtonAState == LOW) {
    outputSensorValues();
  }
  
  // 按下按钮B
  if (lastButtonBState == HIGH && currentButtonBState == LOW) {
    outputAmbientLightValues();
  }
  
  // 按下按钮C
  if (lastButtonCState == HIGH && currentButtonCState == LOW) {
    outputFilteredValues();
  }
  
  // 更新上一次按钮状态
  lastButtonAState = currentButtonAState;
  lastButtonBState = currentButtonBState;
  lastButtonCState = currentButtonCState;
}

// 输出原始传感器值
void outputSensorValues() {
  Serial.println("=== 原始传感器值 ===");
  Serial.print("BL: "); Serial.print(analogRead(BUMPER_LEFT));
  Serial.print(" | BR: "); Serial.println(analogRead(BUMPER_RIGHT));
  
  Serial.print("LL: "); Serial.print(analogRead(LINE_SENSOR_LEFT));
  Serial.print(" | LCL: "); Serial.print(analogRead(LINE_SENSOR_CENTER_LEFT));
  Serial.print(" | LC: "); Serial.print(analogRead(LINE_SENSOR_CENTER));
  Serial.print(" | LCR: "); Serial.print(analogRead(LINE_SENSOR_CENTER_RIGHT));
  Serial.print(" | LR: "); Serial.println(analogRead(LINE_SENSOR_RIGHT));
  Serial.println();
}

// 输出环境光补偿值
void outputAmbientLightValues() {
  Serial.println("=== 环境光补偿值 ===");
  Serial.print("BL基准: "); Serial.print(ambientLightBumperLeft);
  Serial.print(" | BR基准: "); Serial.println(ambientLightBumperRight);
  
  Serial.print("LL基准: "); Serial.print(ambientLightLineLeft);
  Serial.print(" | LCL基准: "); Serial.print(ambientLightLineCenterLeft);
  Serial.print(" | LC基准: "); Serial.print(ambientLightLineCenter);
  Serial.print(" | LCR基准: "); Serial.print(ambientLightLineCenterRight);
  Serial.print(" | LR基准: "); Serial.println(ambientLightLineRight);
  Serial.println();
}

// 输出平均滤波值
void outputFilteredValues() {
  Serial.println("=== 滤波后传感器值 ===");
  Serial.print("BL: "); Serial.print(currentSensors.bumperLeft);
  Serial.print(" | BR: "); Serial.println(currentSensors.bumperRight);
  
  Serial.print("LL: "); Serial.print(currentSensors.lineLeft);
  Serial.print(" | LCL: "); Serial.print(currentSensors.lineCenterLeft);
  Serial.print(" | LC: "); Serial.print(currentSensors.lineCenter);
  Serial.print(" | LCR: "); Serial.print(currentSensors.lineCenterRight);
  Serial.print(" | LR: "); Serial.println(currentSensors.lineRight);
  
  // 输出距离和转向误差
  int distanceError = calculateDistanceError();
  int steeringError = calculateSteeringError();
  Serial.print("距离误差: "); Serial.print(distanceError);
  Serial.print(" | 转向误差: "); Serial.println(steeringError);
  Serial.println();
}

// 计算距离误差
int calculateDistanceError() {
  int totalBumper = currentSensors.bumperLeft + currentSensors.bumperRight;
  return totalBumper - TARGET_DISTANCE;
}

// 计算转向误差
int calculateSteeringError() {
  int bumperSteeringError = currentSensors.bumperRight - currentSensors.bumperLeft;
  
  // 使用线传感器计算位置误差
  int lineLeftSum = currentSensors.lineLeft + currentSensors.lineCenterLeft;
  int lineRightSum = currentSensors.lineRight + currentSensors.lineCenterRight;
  int lineSteeringError = lineRightSum - lineLeftSum;
  
  // 综合碰撞传感器和线传感器的转向误差
  // 权重需要测试
  int totalSteeringError = (bumperSteeringError * 7 + lineSteeringError * 3) / 10;
  
  return totalSteeringError;
}

// 设置电机速度
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // 限制速度范围
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);
  
  // 设置左电机
  if (leftSpeed >= 0) {
    digitalWrite(LEFT_MOTOR_DIR, LOW);
    analogWrite(LEFT_MOTOR_PWM, leftSpeed);
  } else {
    digitalWrite(LEFT_MOTOR_DIR, HIGH);
    analogWrite(LEFT_MOTOR_PWM, -leftSpeed);
  }
  
  // 设置右电机
  if (rightSpeed >= 0) {
    digitalWrite(RIGHT_MOTOR_DIR, LOW);
    analogWrite(RIGHT_MOTOR_PWM, rightSpeed);
  } else {
    digitalWrite(RIGHT_MOTOR_DIR, HIGH);
    analogWrite(RIGHT_MOTOR_PWM, -rightSpeed);
  }
}

// 主控制函数
void followControl() {
  // 计算距离误差和转向误差
  int distanceError = calculateDistanceError();
  int steeringError = calculateSteeringError();
  
  // 根据距离误差调整速度
  int baseSpeed = BASE_SPEED;
  if (distanceError > 0) {
    // 距离太近，减速
    baseSpeed = max(MIN_SPEED, BASE_SPEED - distanceError / 10);
  } else if (distanceError < -100) {
    // 距离太远，加速
    baseSpeed = min(MAX_SPEED, BASE_SPEED + (-distanceError) / 20);
  }
  
  // 根据转向误差调整左右轮速差
  int speedDifference = steeringError / 8; // 比例系数需要调试
  
  // 计算最终电机速度
  int leftSpeed = baseSpeed + speedDifference;
  int rightSpeed = baseSpeed - speedDifference;
  
  setMotorSpeed(leftSpeed, rightSpeed);
}

void setup() {
  // 初始化串口
  Serial.begin(9600);
  
  // 初始化电机
  initMotors();
  
  // 初始化按钮
  initButtons();
  
  // 环境光校准
  calibrateAmbientLight();
  
  // 停止电机
  setMotorSpeed(0, 0);
  
  Serial.println("系统初始化完成");
  Serial.println("按钮A: 原始传感器值 | 按钮B: 环境光基准 | 按钮C: 滤波后值");
  
  delay(2000); // 等待2秒开始
}

void loop() {
  // 读取传感器数据
  readSensors();
  
  // 执行跟随控制
  followControl();
  
  // 串口输出
  handleSerialOutput();
  
  delay(20);
}
