#define ADC_TO_VOLTS(x) x * 3.3f / 4095
// nominal resistance and beta derived from this thermistor
// https://www.amazon.com/dp/B0CM3FSFF9?ref_=cm_sw_r_cp_ud_dp_4A33XQ4N359D6HDTNADD
#define R25 84.32 /* nominal resistance of thermistor */
#define BETA 3851 /* beta value of thermistor */
#define VOLTS_IN 3.3f
#define R1 150

// PID values
#define kp 255.0f
#define ki 0.0f
#define kd 10.0f
// #define dt 0.05f
float prev_error = 0;
float integral = 0;

// TBH values
float tbh = 0;
float tbh_out = 0;
#define gain 0.0025f

// #define scale 1000.0f

#define SENSOR_PIN 15
#define MOT_IN1 4
#define MOT_IN2 16
#define EEP 17

const float T25_INVERSE = 1 / 298.15f; 

long delta_millis = 0;
long cur_time = 0;
long delta_delay = 0;

float palm_temp = 27;
float thumb_temp = 27;
float index_temp = 27;
float middle_temp = 27;
float ring_temp = 27;
float little_temp = 27;

float temps[6] = {27, 27, 27, 27, 27, 27};
float deltas[6] = {0, 0, 0, 0, 0, 0};

// Calculate temperature from resistance with following equation
// T(R) = 1 / ( ( ln( R / R25 ) / BETA ) + T25_INVERSE )
// https://www.giangrandi.org/electronics/ntc/ntc.shtml
float calculate_temperature(float r) {
  float leftTerm = log(r / R25) / BETA;
  return 1 / (leftTerm + T25_INVERSE);
}

// Calculate second resistor in voltage divider circuit with the following equation
// r2 = 9r1 * vOut) / (vSource - vOut);
float calculate_resistance(float r1, float vOut) {
  return (r1 * vOut) / (VOLTS_IN - vOut);
}

float get_temperature() {
  int sensorVal = analogRead(SENSOR_PIN);
  float v = ADC_TO_VOLTS(sensorVal);
  float r = calculate_resistance(R1, v);
  float t = calculate_temperature(r);
  return t - 273.15f;
}

void set_motor_pwm(int pwm, int IN1_PIN, int IN2_PIN)
{
  if (pwm < 0) {  // reverse speeds
    analogWrite(IN1_PIN, -pwm);
    analogWrite(IN2_PIN, LOW);

  } else { // stop or forward
    analogWrite(IN1_PIN, LOW);
    analogWrite(IN2_PIN, pwm);
  }
}

void pid_temp(float target, int motor_a, int motor_b) {
  float cur_temp = get_temperature();
  float dt = delta_millis / 1000.0f;
  if(dt <= 0.000001f) {
    dt = 0.000001f;
  }
  // Serial.println(cur_temp);
  float error = target - cur_temp;

  float p = error * kp;
  
  integral += error * dt;

  integral = min(255.0f,max(-255.0f, integral));

  float i = ki * integral;


  float d = kd * (error - prev_error) / dt;

  float output = p + i + d;
  // Serial.printf("p = %.2f i = %.2f d = %.2f \n", p, i, d);
  prev_error = error;

  int out = round(min(255.0f, max(-255.0f, output)));

  set_motor_pwm(out, motor_a, motor_b);
  
}

// void tbh_temp(float target, int motor_a, int motor_b) {
//   float error = target - get_temperature();

//   float dt = delta_millis / 1000.0f;
//   if(dt <= 0.000001f) {
//     dt = 0.000001f;
//   }

//   tbh_out += error * gain / dt;

//   tbh_out = min(255.0f, max(-255.0f, tbh_out));

//   if(error * prev_error < 0) {
//     tbh_out = 0.5 * (tbh_out + tbh);
//     tbh = tbh_out;
//   }
//   prev_error = error;
//   set_motor_pwm(round(tbh_out), motor_a, motor_b);
// }

void read_temps_bytes(float * out) {
  if(Serial.available() == 0) {
    return;
  }

  byte temp_bytes[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  short short_buffer = 0;
  Serial.readStringUntil('S');
  int bytes_read = Serial.readBytes(temp_bytes, 12);
  if(bytes_read != 12) {
    return;
  }
  for(int i = 0; i < 6; i++) {
    short_buffer = (temp_bytes[i * 2]) | (temp_bytes[(i * 2) + 1] << 8);
    temps[i] = (short_buffer > 1500 && short_buffer < 4000) ? short_buffer / 100.0f: 27.0f;
    // out[i] = (short_buffer > -25600 && short_buffer < 25600) ? short_buffer / 100.0f : 0;
  }
  // char buffer [64];
  // snprintf(buffer, 64, "P%.2f,T%.2f,I%.2f,M%.2f,R%.2f,L%.2f", out[0], out[1], out[2], out[3], out[4], out[5]);
  // Serial.println(buffer);
}

void setup() {
  Serial.begin(9600);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(MOT_IN1, OUTPUT);
  pinMode(MOT_IN2, OUTPUT);
  pinMode(15, INPUT_PULLUP);
  pinMode(EEP, OUTPUT);
  analogWrite(MOT_IN1, LOW);
  analogWrite(MOT_IN2, LOW);
  digitalWrite(EEP,HIGH);
  prev_error = 0;
  integral = 0;
}

void loop() {
  delta_millis = millis() - cur_time;
  cur_time += delta_millis;
  delta_delay += delta_millis;
  // read_temps_string();
  // read_temps_bytes(deltas);
  read_temps_bytes(temps);

  // String in = Serial.readStringUntil('\n');
  // float in_val = in.toFloat();
  
  // tbh_temp(38, MOT_IN1, MOT_IN2);
  // pid_temp(0, MOT_IN1, MOT_IN2);
  pid_temp(temps[0], MOT_IN1, MOT_IN2);
  // int output = round(deltas[0] * 100); 
  // set_motor_pwm(output, MOT_IN1, MOT_IN2);
  // Serial.println(temps[0]);
  // if(delta_delay > 20) {
  //   delta_delay -= 20;
  // }
  // Serial.print("Output power: ");
  // Serial.println(output);
  // Serial.print("Temperature: ");
  Serial.println(get_temperature());
  delay(20);

}
