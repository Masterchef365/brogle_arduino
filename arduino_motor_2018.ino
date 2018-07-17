#include <PID_v1.h>
#include <Encoder.h>
#include <Wire.h>

#define SLAVE_ADDRESS 0x04

#define PWM_PIN_LEFT 5 
#define PWM_PIN_RIGHT 6 

#define ENC_PIN_A 2
#define ENC_PIN_B 3

#define LIMIT_SWC_PIN 4

#define MESSAGE_BUFFER_SIZE 32

const unsigned int error_threshold = 10;

#define MOTOR_VARS() \
	VAR(target) \
	VAR(position) \
	VAR(kp) \
	VAR(ki) \
	VAR(kd) \
	VAR(mode) \
	VAR(is_homing) \
	VAR(home_pwm) \
	VAR(enable) \
	VAR(max_pwm) \

#define VAR(NAME) motor_num_##NAME,
typedef enum MotorVarNum {
	motor_num_none,
	MOTOR_VARS()
} MotorVarNum;
#undef VAR

#define VAR(NAME) double NAME;
	MOTOR_VARS();
#undef VAR

Encoder encoder(ENC_PIN_A, ENC_PIN_B);

unsigned char latest_i2c_received = motor_num_none;
unsigned char message_buffer[MESSAGE_BUFFER_SIZE];

double pid_in, pid_out;
PID pid(&pid_in, &pid_out, &target, kp, ki, kd, DIRECT);

void motor_set_pwm(int speed) {
	if (speed > 0) {
		analogWrite(PWM_PIN_LEFT, speed);
		analogWrite(PWM_PIN_RIGHT, 0);
	} else {
		analogWrite(PWM_PIN_RIGHT, -speed);
		analogWrite(PWM_PIN_LEFT, 0);
	}
}

void setup() {
	Serial.begin(38400);
	pinMode(PWM_PIN_LEFT, OUTPUT);
	pinMode(PWM_PIN_RIGHT, OUTPUT);
	pinMode(ENC_PIN_A, INPUT);
	pinMode(ENC_PIN_B, INPUT);

	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(0, 0);
	pid.SetSampleTime(5);

	Wire.begin(SLAVE_ADDRESS);
	Wire.onReceive(receiveData);
	Wire.onRequest(sendData);
}

void loop() {
	if (enable > 0.0) {
		if (digitalRead(LIMIT_SWC_PIN) == HIGH) {
			is_homing = 0.0;
			position = 0.0;
			encoder.write(position);
		}
		if (is_homing > 0.0) {
			motor_set_pwm(home_pwm);	
		} else {
			pid_in = encoder.read();
			if (abs(pid_in - target) > error_threshold) {
				pid.Compute();
				motor_set_pwm(pid_out);
			} else {
				motor_set_pwm(0);
			}
		}
	} else {
		motor_set_pwm(0);
	}
}

void receiveData(int byteCount) {
	char* bufcpy = message_buffer;
	while (Wire.read() != '['); //Sync to type_recieved message
	unsigned char type = Wire.read();

	size_t bytes_read = 0;
	while (
			Wire.available() && 
			(*bufcpy++ = Wire.read()) != '\n' && 
			bytes_read < MESSAGE_BUFFER_SIZE
			);
	*bufcpy = '\0';
	float parsed = atof(message_buffer);

	switch(type) {
#define VAR(NAME) case motor_num_##NAME: NAME = parsed; break;
		MOTOR_VARS()
#undef VAR
	}
	switch (type) {
		case motor_num_kp:
		case motor_num_ki:
		case motor_num_kd:
			pid.SetTunings(kp, ki, kd);
			break;
		case motor_num_max_pwm:
			pid.SetOutputLimits(-parsed, parsed);
			break;
	}

	latest_i2c_received = type;
}

void sendData() {
	Wire.write(latest_i2c_received);
}
