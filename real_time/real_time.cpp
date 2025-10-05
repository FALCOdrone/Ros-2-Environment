/*
 * Teensy 4.1 Drone Controller
 * Receives commands from ROS2 via serial and controls drone hardware
 */

#include <ArduinoJson.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <NewPing.h>
#include <Adafruit_BMP280.h> // For barometer
#include <TinyGPS++.h> // For GPS
#include <HMC5883L.h> // For Magnetometer

// NOTE: Communication with ROS2 is handled via serial JSON messages

// Hardware pins -> modify them from teensy 
#define MOTOR_FL_PIN 2   // Front Left Motor (PWM)
#define MOTOR_FR_PIN 3   // Front Right Motor (PWM)
#define MOTOR_BL_PIN 5   // Back Left Motor (PWM)
#define MOTOR_BR_PIN 6   // Back Right Motor (PWM)

#define SONAR_TRIGGER_PIN 7
#define SONAR_ECHO_PIN 8
#define MAX_DISTANCE 400  // Maximum distance in cm

// Motor objects
Servo motorFL, motorFR, motorBL, motorBR;

// Sensors
MPU6050 mpu;
NewPing sonar(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN, MAX_DISTANCE);
TinyGPSPlus gps;  // GPS object should be global
HMC5883L compass; // Magnetometer object should be global

// Control variables
struct DroneState {
  bool armed = false;
  bool position_control = false;
  float target_x = 0.0, target_y = 0.0, target_z = 0.0;
  float target_roll = 0.0, target_pitch = 0.0, target_yaw = 0.0;
  float thrust = 0.0;
  unsigned long last_command_time = 0;
};

DroneState drone_state;

// Safety parameters -> TODO: modify those parameters according to the motors datasheet
const unsigned long COMMAND_TIMEOUT = 1000; // 1 second
const int MIN_PWM = 1000;
const int MAX_PWM = 2000;
const int MOTOR_ARM_VALUE = 1050; // small pwm value to disarm motors

// Timing
unsigned long last_sensor_publish = 0;
unsigned long last_imu_read = 0;
unsigned long last_baro_read = 0;
unsigned long last_gps_read = 0;
unsigned long last_mag_read = 0;

// Sensor read intervals
const unsigned long SENSOR_PUBLISH_INTERVAL = 20; // 50Hz
const unsigned long IMU_READ_INTERVAL = 10; // 100Hz
const unsigned long BARO_READ_INTERVAL = 100; // 10Hz -> to check
const unsigned long GPS_READ_INTERVAL = 1000; // 1Hz -> to check
const unsigned long MAG_READ_INTERVAL = 50; // 20Hz

// GPS reference point (set this to your launch location)
const float REF_LAT = 45.4773; // Example: Politecnico di Milano coordinates
const float REF_LON = 9.2277;
const float REF_ALT = 120.0;   // meters above sea level

// Earth radius in meters
const float EARTH_RADIUS = 6378137.0;

// Magnetometer calibration offsets (you need to calibrate these values)
// These should be determined through a calibration procedure
float mag_offset_x = 0.0;
float mag_offset_y = 0.0;
float mag_offset_z = 0.0;
float mag_scale_x = 1.0;
float mag_scale_y = 1.0;
float mag_scale_z = 1.0;

void setup() {
  Serial.begin(115200);
  
  // Initialize motors
  motorFL.attach(MOTOR_FL_PIN);
  motorFR.attach(MOTOR_FR_PIN);
  motorBL.attach(MOTOR_BL_PIN);
  motorBR.attach(MOTOR_BR_PIN);
  
  // Initialize all motors to minimum PWM for safety
  disarmMotors();
  
  // Initialize IMU
  Wire.begin();
  mpu.initialize();

  // Initialize barometer
  Adafruit_BMP280 bmp;
  if (!bmp.begin(0x76)) {
    Serial.println("{\"type\":\"STATUS\",\"data\":\"Barometer initialization failed\"}");
  } else {
    Serial.println("{\"type\":\"STATUS\",\"data\":\"Barometer initialized successfully\"}");
  }

  // Initialize GPS
  Serial1.begin(9600); // GPS is connected to Serial1
  Serial.println("{\"type\":\"STATUS\",\"data\":\"GPS initialized successfully\"}");
  
  // Initialize magnetometer
  if (compass.begin()) {
    Serial.println("{\"type\":\"STATUS\",\"data\":\"Magnetometer initialized successfully\"}");
  } else {
    Serial.println("{\"type\":\"STATUS\",\"data\":\"Magnetometer initialization failed\"}");
  }
  
  if (mpu.testConnection()) {
    Serial.println("{\"type\":\"STATUS\",\"data\":\"IMU connected successfully\"}");
  } else {
    Serial.println("{\"type\":\"STATUS\",\"data\":\"IMU connection failed\"}");
  }
  
  Serial.println("{\"type\":\"STATUS\",\"data\":\"Teensy drone controller ready\"}");
}

void loop() {
  unsigned long current_time = millis();
  
  // Process incoming commands
  processSerialCommands();
  
  // Safety check - disarm if no recent commands
  if (current_time - drone_state.last_command_time > COMMAND_TIMEOUT) {
    if (drone_state.armed) {
      disarmMotors();
      Serial.println("{\"type\":\"STATUS\",\"data\":\"Safety timeout - motors disarmed\"}");
    }
  }
  
  // Read and publish sensor data
  if (current_time - last_imu_read >= IMU_READ_INTERVAL) {
    readIMU();
    last_imu_read = current_time;
  }

  // Read barometer and GPS data
  if (current_time - last_baro_read >= BARO_READ_INTERVAL) {
    readBarometer();
    last_baro_read = current_time;
  }
  if (current_time - last_gps_read >= GPS_READ_INTERVAL) {
    readGPS();
    last_gps_read = current_time;
  }
  if (current_time - last_mag_read >= MAG_READ_INTERVAL) {
    readMagnetometer();
    last_mag_read = current_time;
  }

  // Update motor outputs if armed
  if (drone_state.armed) {
    updateMotors();
  }
}

void processSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      parseCommand(command);
    }
  }
}

void parseCommand(String command) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, command);
  
  if (error) {
    Serial.println("{\"type\":\"STATUS\",\"data\":\"JSON parse error\"}");
    return;
  }
  
  String cmd_type = doc["cmd"];
  drone_state.last_command_time = millis();
  
  if (cmd_type == "CMD_VEL") {
    handleMovementCommand(doc["data"]);
  } else if (cmd_type == "CONTROL_WRENCH") {
    handleControlWrenchCommand(doc["data"]);
  } else if (cmd_type == "TAKEOFF") {
    handleTakeoffCommand();
  } else if (cmd_type == "LAND") {
    handleLandCommand();
  } else if (cmd_type == "POS_CTRL") {
    drone_state.position_control = doc["data"]["enabled"];
    Serial.println("{\"type\":\"STATUS\",\"data\":\"Position control mode updated\"}");
  }
}

void handleControlWrenchCommand(JsonObject data) {
  // Process the incoming control wrench command from ROS2 bridge
  // Expected JSON format: {"force": {"x": 0, "y": 0, "z": thrust}, "torque": {"x": roll, "y": pitch, "z": yaw}}
  drone_state.thrust = data["force"]["z"];
  drone_state.target_roll = data["torque"]["x"];
  drone_state.target_pitch = data["torque"]["y"]; 
  drone_state.target_yaw = data["torque"]["z"];
  
  // Update command timestamp for safety
  drone_state.last_command_time = millis();
}

void handleMovementCommand(JsonObject data) {
  if (drone_state.position_control) { // we are currently using position control mode 
    // Position control mode - treat as target positions
    drone_state.target_x = data["linear"]["x"];
    drone_state.target_y = data["linear"]["y"];
    drone_state.target_z = data["linear"]["z"];
  } else {
    // Velocity control mode - convert to motor commands
    float linear_x = data["linear"]["x"];
    float linear_y = data["linear"]["y"];
    float linear_z = data["linear"]["z"];
    float angular_z = data["angular"]["z"];
    
    // Simple mixing - convert velocities to motor commands
    // Convert velocity to thrust: linear_z = 0 means hover, positive = more thrust
    const float HOVER_THRUST = 13.73; // Newtons for 1.4kg drone (1.4 × 9.81)
    drone_state.thrust = HOVER_THRUST + (linear_z * 5.0); // Scale velocity to thrust
    drone_state.thrust = constrain(drone_state.thrust, 0, 27.46); // Limit to 2x hover thrust
    drone_state.target_roll = constrain(linear_y * 45, -45, 45);   // degrees
    drone_state.target_pitch = constrain(linear_x * 45, -45, 45);  // degrees
    drone_state.target_yaw = constrain(angular_z * 45, -45, 45);   // degrees
  }
}

void handleTakeoffCommand() {
  drone_state.armed = true;
  // For a 1.4kg drone, hover thrust = 1.4kg × 9.81 m/s² = 13.73N
  // Takeoff thrust should be ~110% of hover thrust for safe takeoff
  const float DRONE_MASS = 1.4; // kg - actual drone mass
  const float HOVER_THRUST = DRONE_MASS * 9.81; // 13.73N
  drone_state.thrust = HOVER_THRUST * 1.1; // 15.1N for takeoff

  Serial.println("{\"type\":\"STATUS\",\"data\":\"Motors armed - taking off\"}");
}

void handleLandCommand() {
  const float SMALL_MASS = 1.1; // kg - small mass for landing
  const float LANDING_THRUST = SMALL_MASS * 9.81; // small_mass is lower than the drone mass
  drone_state.thrust = LANDING_THRUST;  // Set thrust to landing thrust
  // Gradually reduce thrust and then disarm
  delay(100); // Simple landing delay
  disarmMotors();
  Serial.println("{\"type\":\"STATUS\",\"data\":\"Landing sequence initiated\"}");
}

void startMotorsButton() {
  // wait for the serial command "A" to arm the motors
  // This is a safety feature to prevent accidental motor start
  while(!Serial.available()) {
    // Wait for command
    delay(100);
  }

  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command != "A") {
    Serial.println("{\"type\":\"STATUS\",\"data\":\"Invalid command to arm motors\"}");
    return;
  }

  Serial.println("{\"type\":\"STATUS\",\"data\":\"Arming motors...\"}");

  // then we arm the motors...
  if (!drone_state.armed) {
    motorFL.writeMicroseconds(MOTOR_ARM_VALUE);
    motorFR.writeMicroseconds(MOTOR_ARM_VALUE);
    motorBL.writeMicroseconds(MOTOR_ARM_VALUE);
    motorBR.writeMicroseconds(MOTOR_ARM_VALUE);
    drone_state.armed = true;
    drone_state.thrust = 0.0;  // Reset thrust to zero
    Serial.println("{\"type\":\"STATUS\",\"data\":\"Motors armed\"}");
  }
}

void stopMotorsButton() {
  // wait for the serial command "D" to disarm the motors
  // This is a safety feature to prevent accidental motor stop
  while(!Serial.available()) {
    // Wait for command
    delay(100);
  }
  String command = Serial.readStringUntil('\n');
  command.trim();

  if (command != "D") {
    Serial.println("{\"type\":\"STATUS\",\"data\":\"Invalid command to disarm motors\"}");
    return;
  }

  Serial.println("{\"type\":\"STATUS\",\"data\":\"Disarming motors...\"}");

  // then we disarm the motors...
  if (drone_state.armed) {
    disarmMotors();
    Serial.println("{\"type\":\"STATUS\",\"data\":\"Motors disarmed\"}");
  }
}

void disarmMotors() {
  motorFL.writeMicroseconds(MOTOR_ARM_VALUE);
  motorFR.writeMicroseconds(MOTOR_ARM_VALUE);
  motorBL.writeMicroseconds(MOTOR_ARM_VALUE);
  motorBR.writeMicroseconds(MOTOR_ARM_VALUE);
  drone_state.armed = false;
  drone_state.thrust = 0.0;  // Reset thrust to zero
}

void updateMotors() {
  // Check if we have control wrench data (preferred) or fallback to manual control
  // The control wrench provides more accurate PID-controlled values

  float base_thrust_force = drone_state.thrust;  // force.z from wrench
  float roll_torque = drone_state.target_roll;       // torque.x from wrench
  float pitch_torque = drone_state.target_pitch;     // torque.y from wrench
  float yaw_torque = drone_state.target_yaw;         // torque.z from wrench

  // Drone physical parameters for torque to force conversion
  const float ARM_LENGTH = 0.15;     // Distance from center to motor (meters) - adjust for your drone
  const float HOVER_THRUST = 13.73;  // Newtons (for 1.4kg drone) -> which means that we are applying little less than 75% of the throttle
  const float MAX_THRUST = 27.46;    // Maximum thrust capability (approximately 2x hover)
  const float MIN_THRUST = 0.0;      // Minimum thrust (0N)
  
  // Convert torques to force adjustments (Torque = Force × Distance, so Force = Torque / Distance)
  float roll_force_adj = roll_torque / ARM_LENGTH;     // Convert roll torque to force difference
  float pitch_force_adj = pitch_torque / ARM_LENGTH;   // Convert pitch torque to force difference  
  float yaw_force_adj = yaw_torque / ARM_LENGTH;       // Convert yaw torque to force difference
  
  // Calculate individual motor forces for X configuration
  // Base thrust distributed equally among 4 motors
  float base_motor_force = base_thrust_force / 4.0;
  
  // Motor mixing for X configuration - calculate force per motor
  // FL: Front Left, FR: Front Right, BL: Back Left, BR: Back Right
  float motor_fl_force = base_motor_force - roll_force_adj + pitch_force_adj - yaw_force_adj;
  float motor_fr_force = base_motor_force + roll_force_adj + pitch_force_adj + yaw_force_adj;
  float motor_bl_force = base_motor_force - roll_force_adj - pitch_force_adj + yaw_force_adj;
  float motor_br_force = base_motor_force + roll_force_adj - pitch_force_adj - yaw_force_adj;
  
  // Convert individual motor forces to PWM values
  // Linear mapping: 0N -> 1000μs, MAX_THRUST/4 -> 2000μs (since each motor provides 1/4 of max thrust)
  float max_motor_force = MAX_THRUST / 4.0;  // Maximum force per motor
  
  int motor_fl = (int)(1000 + (motor_fl_force / max_motor_force) * 1000);
  int motor_fr = (int)(1000 + (motor_fr_force / max_motor_force) * 1000);
  int motor_bl = (int)(1000 + (motor_bl_force / max_motor_force) * 1000);
  int motor_br = (int)(1000 + (motor_br_force / max_motor_force) * 1000);

  // Constrain to safe values
  motor_fl = constrain(motor_fl, 1000, 2000);
  motor_fr = constrain(motor_fr, 1000, 2000);
  motor_bl = constrain(motor_bl, 1000, 2000);
  motor_br = constrain(motor_br, 1000, 2000);
  
  // Write to motors
  motorFL.writeMicroseconds(motor_fl);
  motorFR.writeMicroseconds(motor_fr);
  motorBL.writeMicroseconds(motor_bl);
  motorBR.writeMicroseconds(motor_br);
  
  // Optional: Debug output
  /*
  Serial.print("Motors: FL="); Serial.print(motor_fl);
  Serial.print(" FR="); Serial.print(motor_fr);
  Serial.print(" BL="); Serial.print(motor_bl);
  Serial.print(" BR="); Serial.println(motor_br);
  */
}

void readIMU() {
  // Publish IMU data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  StaticJsonDocument<256> imu_doc;
  imu_doc["type"] = "IMU";
  
  // Simple quaternion (you'd want proper AHRS here)
  imu_doc["data"]["qw"] = 1.0;
  imu_doc["data"]["qx"] = 0.0;
  imu_doc["data"]["qy"] = 0.0;
  imu_doc["data"]["qz"] = 0.0;

  // Gravity vector and linear acceleration
  imu_doc["data"]["gravity"]["x"] = 0.0; // Assuming no tilt for simplicity
  imu_doc["data"]["gravity"]["y"] = 0.0;
  imu_doc["data"]["gravity"]["z"] = 9.81; // Standard gravity
  imu_doc["data"]["linear_acceleration"]["x"] = ax / 16384.0 * 9.81; // Convert to m/s^2
  imu_doc["data"]["linear_acceleration"]["y"] = ay / 16384.0 * 9.81;
  imu_doc["data"]["linear_acceleration"]["z"] = az / 16384.0 * 9.81;
  imu_doc["data"]["angular_velocity"]["x"] = gx / 131.0; // Convert to rad/s
  imu_doc["data"]["angular_velocity"]["y"] = gy / 131.0;
  imu_doc["data"]["angular_velocity"]["z"] = gz / 131.0;
  imu_doc["data"]["timestamp"] = millis(); // Add timestamp
  
  serializeJson(imu_doc, Serial);
  Serial.println();
  
  // Publish sonar data

  /*unsigned int distance = sonar.ping_cm();
  if (distance > 0) {
    StaticJsonDocument<128> sonar_doc;
    sonar_doc["type"] = "SONAR";
    sonar_doc["data"]["distance"] = distance / 100.0; // Convert to meters
    
    serializeJson(sonar_doc, Serial);
    Serial.println();
  }
  */
}

void readMagnetometer() {
  // Read magnetometer data and calculate yaw

  // TODO: Implement better preprocessing for magnetometer data
  // In order to compensate hard and soft iron distortions
  // by: 1) measuring the raw magnetometer values
  // 2) applying calibration offsets for removing the mean magneti filed
  // value avaraging the raw data
  // 3) applying soft iron correction by trying to convert the distorted elipsoid
  // data to a sphere (which means constant mag. field of the earth, neglecting iron distortions)

  int16_t mx_raw, my_raw, mz_raw;
  
  if (compass.isReady()) {
    compass.read(&mx_raw, &my_raw, &mz_raw);
    
    // Apply calibration (hard iron and soft iron correction)
    float mx = (mx_raw - mag_offset_x) * mag_scale_x;
    float my = (my_raw - mag_offset_y) * mag_scale_y;
    float mz = (mz_raw - mag_offset_z) * mag_scale_z;
    
    // Get current IMU data for tilt compensation
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Convert accelerometer data to angles (in radians)
    float ax_g = ax / 16384.0;  // Convert to g
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;
    
    float roll = atan2(ay_g, az_g);
    float pitch = atan2(-ax_g, sqrt(ay_g * ay_g + az_g * az_g));
    
    // Tilt compensation for magnetometer
    float mx_comp = mx * cos(pitch) + mz * sin(pitch);
    float my_comp = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
    
    // Calculate yaw (heading) in radians
    float yaw_rad = atan2(-my_comp, mx_comp);
    
    // Convert to degrees and normalize to 0-360
    float yaw_deg = yaw_rad * 180.0 / PI;
    if (yaw_deg < 0) {
      yaw_deg += 360.0;
    }
    
    StaticJsonDocument<256> mag_doc;
    mag_doc["type"] = "MAGNETOMETER";
    mag_doc["data"]["x_raw"] = mx_raw;     // Raw magnetometer values
    mag_doc["data"]["y_raw"] = my_raw;
    mag_doc["data"]["z_raw"] = mz_raw;
    mag_doc["data"]["x"] = mx;             // Calibrated values
    mag_doc["data"]["y"] = my;
    mag_doc["data"]["z"] = mz;
    mag_doc["data"]["yaw"] = yaw_deg;      // Yaw in degrees (0-360)
    mag_doc["data"]["yaw_rad"] = yaw_rad;  // Yaw in radians
    mag_doc["data"]["roll"] = roll * 180.0 / PI;  // Roll for debugging
    mag_doc["data"]["pitch"] = pitch * 180.0 / PI; // Pitch for debugging
    
    serializeJson(mag_doc, Serial);
    Serial.println();
  } else {
    // Magnetometer not ready or failed
    StaticJsonDocument<128> error_doc;
    error_doc["type"] = "STATUS";
    error_doc["data"] = "Magnetometer not ready";
    serializeJson(error_doc, Serial);
    Serial.println();
  }
}

// Adding barometer and GPS sensors
// would follow a similar pattern, initializing the sensors,
// reading data, and publishing it in the loop.
// For example, you would use a library like Adafruit_BMP280 for the barometer
// and TinyGPS++ for GPS, then read and publish their data in the loop.

void readBarometer() {
  // Example function to read barometer data
  Adafruit_BMP280 bmp;
  if (bmp.begin()) {
    float temperature = bmp.readTemperature();
    float pressure = bmp.readPressure();
    // Convert pressure to altitude if needed
    float altitude = bmp.readAltitude(pressure); // reading altitude based on pressure
    StaticJsonDocument<128> baro_doc;
    baro_doc["type"] = "BAROMETER";  // Changed from "BARO" to match Python bridge
    baro_doc["data"]["temperature"] = temperature;
    baro_doc["data"]["pressure"] = pressure;
    baro_doc["data"]["altitude"] = altitude;
    serializeJson(baro_doc, Serial);
    Serial.println();
  }
}

void readGPS() {
  // Read GPS data from Serial1 (where GPS module is connected)
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      if (gps.location.isUpdated()) {
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        float altitude = gps.altitude.meters();
        
        // Convert GPS to ENU coordinates using reference point
        float lat_rad = radians(latitude);
        float lon_rad = radians(longitude);
        float ref_lat_rad = radians(REF_LAT);
        float ref_lon_rad = radians(REF_LON);
        
        // Calculate differences
        float d_lat = lat_rad - ref_lat_rad;
        float d_lon = lon_rad - ref_lon_rad;
        float d_alt = altitude - REF_ALT;
        
        // Convert to ENU coordinates (East-North-Up)
        float x = d_lon * EARTH_RADIUS * cos(ref_lat_rad);  // East
        float y = d_lat * EARTH_RADIUS;                     // North
        float z = d_alt;                                    // Up
        
        StaticJsonDocument<256> gps_doc;
        gps_doc["type"] = "GPS";
        gps_doc["data"]["x"] = x;     // East in meters
        gps_doc["data"]["y"] = y;     // North in meters
        gps_doc["data"]["z"] = z;     // Up in meters
        gps_doc["data"]["speed"] = gps.speed.mps(); // Speed in m/s
        
        serializeJson(gps_doc, Serial);
        Serial.println();
      }
    }
  }
}
