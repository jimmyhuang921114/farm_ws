#include <SimpleFOC.h>

// ============================================================
// AS5600
// ============================================================
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// ============================================================
// Motor / Driver
// ============================================================
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);


// ============================================================
// Velocity PID
// 你目前已經調好的參數 —— 先不要動
// ============================================================
float velocity_P = 0.6;
float velocity_I = 0.27;
float velocity_D = 0.0;

float velocity_LPF = 0.16;


// ============================================================
// Position PID
// 第一階段先只開 P
// ============================================================
float position_P = 4.0;
float position_I = 0.01;
float position_D = 0.262;


// ============================================================
// Position PID variables
// ============================================================
float target_angle = 0.0;

float position_integral = 0.0;
float previous_position_error = 0.0;


// ============================================================
// Velocity command
// ============================================================
float target_velocity = 0.0;
float ramped_velocity = 0.0;


// ============================================================
// Limits
// ============================================================

// Position PID 最大允許輸出的速度
float max_velocity = 40.0;       // rad/s

// 速度命令最大變化率
float max_acceleration = 40.0;   // rad/s^2

// Position Integral Limit
float integral_limit = 2.0;


// ============================================================
// Timing
// ============================================================
unsigned long previous_time_us = 0;


// ============================================================
// Serial Plotter
// ============================================================
unsigned long last_print_time = 0;
const unsigned long print_interval = 50;


// ============================================================
// Setup
// ============================================================
void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println("================================");
  Serial.println("Cascade Position + Velocity PID");
  Serial.println("================================");


  // ----------------------------------------------------------
  // Sensor
  // ----------------------------------------------------------
  sensor.init();

  motor.linkSensor(&sensor);


  // ----------------------------------------------------------
  // Driver
  // ----------------------------------------------------------

  // !!! 這裡必須填你實際的供電電壓 !!!
  driver.voltage_power_supply = 24.9;

  // Driver 最大允許輸出
  driver.voltage_limit = 3.0;

  driver.init();

  motor.linkDriver(&driver);


  // ----------------------------------------------------------
  // SimpleFOC Controller
  //
  // 注意：
  // SimpleFOC 本身只負責 Velocity PID
  // Position PID 是我們自己在 loop 裡面算
  // ----------------------------------------------------------
  motor.controller = MotionControlType::velocity;


  // ==========================================================
  // Velocity PID
  // ==========================================================
  motor.PID_velocity.P = velocity_P;
  motor.PID_velocity.I = velocity_I;
  motor.PID_velocity.D = velocity_D;

  motor.LPF_velocity.Tf = velocity_LPF;


  // ==========================================================
  // Motor limits
  // ==========================================================
  motor.velocity_limit = max_velocity;

  // 你目前測起來最穩定的 1.5V
  motor.voltage_limit = 1.5;

  motor.voltage_sensor_align = 1.5;


  // ----------------------------------------------------------
  // Init motor
  // ----------------------------------------------------------
  motor.init();

  Serial.println("Starting FOC alignment...");

  int foc_status = motor.initFOC();

  Serial.print("initFOC status = ");
  Serial.println(foc_status);

  if (!foc_status) {

    Serial.println("FOC INIT FAILED!");

    while (1) {
      delay(1000);
    }
  }

  Serial.println("FOC INIT SUCCESS");


  // ----------------------------------------------------------
  // Enable motor
  // ----------------------------------------------------------
  motor.enable();


  // ----------------------------------------------------------
  // 現在的位置就是初始 Target
  //
  // 所以開機後不會突然跑去 0 rad
  // ----------------------------------------------------------
  motor.loopFOC();

  target_angle = motor.shaft_angle;

  previous_position_error = 0.0;

  target_velocity = 0.0;
  ramped_velocity = 0.0;

  previous_time_us = micros();


  Serial.println();
  Serial.println("Ready!");
  Serial.println();
  Serial.println("輸入相對角度 (degree)");
  Serial.println("例如:");
  Serial.println("30   -> +30 deg");
  Serial.println("-30  -> -30 deg");
  Serial.println();
}


// ============================================================
// Loop
// ============================================================
void loop() {

  // ----------------------------------------------------------
  // FOC
  // ----------------------------------------------------------
  motor.loopFOC();


  // ----------------------------------------------------------
  // dt
  // ----------------------------------------------------------
  unsigned long now_us = micros();

  float dt = (now_us - previous_time_us) * 1e-6;

  previous_time_us = now_us;


  // 防止異常 dt
  if (dt <= 0.0 || dt > 0.1) {
    dt = 0.001;
  }


  // ==========================================================
  // 1. Position Error
  // ==========================================================
  float position_error =
      target_angle - motor.shaft_angle;


  // ==========================================================
  // 2. Position PID
  // ==========================================================


  // ----------------------------------------------------------
  // P
  // ----------------------------------------------------------
  float P_term =
      position_P * position_error;


  // ----------------------------------------------------------
  // I
  // ----------------------------------------------------------
  position_integral +=
      position_error * dt;

  position_integral =
      constrain(
        position_integral,
        -integral_limit,
        integral_limit
      );

  float I_term =
      position_I * position_integral;


  // ----------------------------------------------------------
  // D
  // ----------------------------------------------------------
  float position_derivative =
      (position_error - previous_position_error)
      / dt;

  previous_position_error =
      position_error;

  float D_term =
      position_D * position_derivative;


  // ----------------------------------------------------------
  // Position PID Output
  //
  // Output = Target Velocity
  // ----------------------------------------------------------
  target_velocity =
      P_term
      + I_term
      + D_term;


  // ----------------------------------------------------------
  // Velocity Limit
  // ----------------------------------------------------------
  target_velocity =
      constrain(
        target_velocity,
        -max_velocity,
        max_velocity
      );


  // ==========================================================
  // 3. Ramp / Acceleration Limit
  // ==========================================================

  float max_velocity_change =
      max_acceleration * dt;


  float velocity_difference =
      target_velocity - ramped_velocity;


  velocity_difference =
      constrain(
        velocity_difference,
        -max_velocity_change,
        max_velocity_change
      );


  ramped_velocity += velocity_difference;


  // ==========================================================
  // 4. Velocity PID
  //
  // SimpleFOC 內部：
  //
  // ramped_velocity
  //        ↓
  // Velocity PID
  //        ↓
  // FOC
  // ==========================================================

  motor.move(ramped_velocity);


  // ==========================================================
  // Serial Input
  // ==========================================================

  if (Serial.available()) {

    String input =
        Serial.readStringUntil('\n');

    input.trim();


    if (input.length() > 0) {

      float input_degree =
          input.toFloat();
          
      Serial.print("ReceivedAngle:");
      Serial.println(input_degree, 2);


      float input_rad =
          input_degree * PI / 180.0;


      // ------------------------------------------------------
      // 相對位置命令
      // ------------------------------------------------------
      target_angle =
          motor.shaft_angle + input_rad;


      // 新命令時清掉 Position I
      position_integral = 0.0;


      Serial.println();
      Serial.println("==============================");

      Serial.print("Move = ");
      Serial.print(input_degree);
      Serial.println(" deg");

      Serial.print("Current = ");
      Serial.print(
        motor.shaft_angle * 180.0 / PI
      );
      Serial.println(" deg");

      Serial.print("Target = ");
      Serial.print(
        target_angle * 180.0 / PI
      );
      Serial.println(" deg");

      Serial.println("==============================");
    }
  }

  // ============================================================
  // Serial Plotter - Position + Velocity Debug
  // ============================================================

  if (millis() - last_print_time >= print_interval) {

    last_print_time = millis();

    // 位置資訊（degree）
    float target_angle_deg =
        target_angle * 180.0 / PI;

    float actual_angle_deg =
        motor.shaft_angle * 180.0 / PI;

    float position_error_deg =
        position_error * 180.0 / PI;

    // 速度誤差
    float velocity_error =
        ramped_velocity - motor.shaft_velocity;


    Serial.print("TargetPos:");
    Serial.print(target_angle_deg, 3);

    Serial.print(",");

    Serial.print("ActualPos:");
    Serial.print(actual_angle_deg, 3);

    Serial.print(",");

    Serial.print("PosError:");
    Serial.print(position_error_deg, 3);

    Serial.print(",");

    Serial.print("TargetVel:");
    Serial.print(target_velocity, 3);

    Serial.print(",");

    Serial.print("RampVel:");
    Serial.print(ramped_velocity, 3);

    Serial.print(",");

    Serial.print("ActualVel:");
    Serial.print(motor.shaft_velocity, 3);

    Serial.print(",");

    Serial.print("VelError:");
    Serial.println(velocity_error, 3);
  }
}