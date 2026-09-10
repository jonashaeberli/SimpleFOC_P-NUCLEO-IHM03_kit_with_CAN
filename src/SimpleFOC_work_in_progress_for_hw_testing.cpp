#include <Arduino.h>
#include <SimpleFOC.h>
#include <ACANFD_STM32.h>


// SPI - Setup using SPI3
const int CS_PIN = PA15;
SPIClass SPI_3(PC12, PC11, PC10);
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, CS_PIN);

// Driver Setup using 3PWM (Mode is LOW)
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8,PA9,PA10,PB13,PB14,PB15);

// Motor
BLDCMotor motor = BLDCMotor(7, 10.0f, 147);

// Current sense
LowsideCurrentSense current_sense = LowsideCurrentSense(0.33f, 1.528f, PA1, PB1, PB0);

// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char* cmd) { command.motor(&motor, cmd); }

// angle command
char angle_command[16];

void setup() {
  // monitoring port
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // setup can communication
  ACANFD_STM32_Settings settings (250 * 1000, DataBitRateFactor::x1) ; // 250 kbit/s
  const uint32_t errorCode = fdcan1.beginFD (settings) ;
  if (0 == errorCode) {
    Serial.println ("can ok") ;
  }else{
    Serial.print ("Error can: 0x") ;
    Serial.println (errorCode, HEX) ;
  }

  // init the sensor
  sensor.min_elapsed_time = 0.001;
  sensor.init(&SPI_3);
  motor.linkSensor(&sensor);

  Serial.println("Sensor ready");


  // init the sensor
  driver.voltage_power_supply = 12.0;
  driver.voltage_limit = 10.0;
  driver.pwm_frequency = 20000;

  if (!driver.init()){
    Serial.println("Driver init failed!");
    return;
  }

  current_sense.linkDriver(&driver);
  
  // (Optional: You may still need skip_align=true here for your high-resistance gimbal motor)
  // current_sense.skip_align = true;

  // 2. THEN init current sense
  if(!current_sense.init()){
    Serial.println("Current sense init failed!");
    return;
  }

  // 3. Link the motor and the driver
  motor.linkDriver(&driver);

  // link current sense to motor
  motor.linkCurrentSense(&current_sense);

  motor.voltage_sensor_align = 2;
  motor.controller = MotionControlType::angle;
  motor.torque_controller = TorqueControlType::foc_current;

  // enable current monitoring
  // motor.useMonitoring(Serial);
  // motor.monitor_downsample = 100;

  // Set safe limits
  motor.current_limit = 0.9;
  motor.target = 0;
  motor.LPF_velocity.Tf = 0.01;
  motor.PID_velocity.P = 0.043;
  motor.PID_velocity.I = 0.01;
  motor.PID_velocity.D = 0.00001;
  motor.LPF_current_q.Tf = 0.02;
  motor.LPF_current_d.Tf = 0.02;

  // initialize motor
  if(!motor.init()){
    Serial.println("Motor init failed!");
    return;
  }
  // align sensor and start FOC
  if(!motor.initFOC()){
    Serial.println("FOC init failed!");
    return;
  }


  // add target command M
  command.add('M', doMotor, "Motor");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target using serial terminal and command M:"));

  _delay(1000);
}

void loop() {
  motor.loopFOC();
  motor.move();
  motor.monitor();
  command.run();


  CANFDMessage message;
  // check if a new can message has arrived
  if (fdcan1.receiveFD0(message)) {
    Serial.print("Received CAN ID: 0x");
    Serial.println(message.id, HEX);
    
    Serial.print("Data: ");
    for (int i = 0; i < message.len; i++) {
      Serial.print(message.data[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    snprintf(
    angle_command,
      sizeof(angle_command),
      "M%u",
      static_cast<unsigned>(message.data[0])
    );
    command.run(angle_command);
  }
}
