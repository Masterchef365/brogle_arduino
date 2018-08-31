#include <PID_v1.h>
#include <Encoder.h>
#include <Wire.h>

#define PWM_PIN_LEFT 5 
#define PWM_PIN_RIGHT 6 

#define ENC_PIN_A 2
#define ENC_PIN_B 3

#define LIMIT_SWC_PIN 10

#define SLAVE_ADDRESS 0x04

#define MESSAGE_BUFFER_SIZE 32

#define MOTOR_KEYS() \
	KEY(target) \
	KEY(position) \
	KEY(kp) \
	KEY(ki) \
	KEY(kd) \
	KEY(mode) \
	KEY(home) \
	KEY(home_pwm) \
	KEY(enable) \
	KEY(max_pwm) \

#define KEY(NAME) motor_key_##NAME,
enum MotorKey {
	motor_key_none,
	MOTOR_KEYS()
	motor_key_count
};
#undef KEY

union MotorKeyMessage {
	struct {
		enum MotorKey id : 7;
		unsigned char rw : 1;
	} id_rw;
	unsigned char num;
};

#define KEY(NAME) double NAME;
struct {
	float dummy;
	MOTOR_KEYS();
} float_table;
float* float_table_ptr = (float*)&float_table;
#undef VAR

char messsage_buf_out[MESSAGE_BUFFER_SIZE];
char messsage_buf_in[MESSAGE_BUFFER_SIZE];

const unsigned int error_threshold = 5;

Encoder encoder(ENC_PIN_A, ENC_PIN_B);

double pid_in, pid_out;
PID pid(&pid_in, &pid_out, &float_table.target, float_table.kp, float_table.ki, float_table.kd, DIRECT);

void motor_set_pwm(int speed) {
	if (abs(speed) <= float_table.max_pwm) {
		if (speed > 0) {
			analogWrite(PWM_PIN_LEFT, speed);
			analogWrite(PWM_PIN_RIGHT, 0);
		} else {
			analogWrite(PWM_PIN_RIGHT, -speed);
			analogWrite(PWM_PIN_LEFT, 0);
		}
	}
}

void setup() {
	Serial.begin(38400);
	pinMode(PWM_PIN_LEFT, OUTPUT);
	pinMode(PWM_PIN_RIGHT, OUTPUT);
	pinMode(ENC_PIN_A, INPUT);
	pinMode(ENC_PIN_B, INPUT);
	pinMode(LIMIT_SWC_PIN, INPUT_PULLUP);

	float_table = {0};

	pid.SetMode(AUTOMATIC);
	pid.SetOutputLimits(0, 0);
	pid.SetSampleTime(5);

	Wire.begin(SLAVE_ADDRESS);
	Wire.onReceive(receiveData);
	Wire.onRequest(sendData);
}

void loop() {
	if (float_table.enable > 0.0) {
		if (float_table.home > 0.0) {
			if (digitalRead(LIMIT_SWC_PIN) == LOW) {
				float_table.home = 0.0;
				float_table.position = 0.0;
				float_table.target = float_table.position;
				encoder.write(float_table.position);
			}
			motor_set_pwm(float_table.home_pwm);	
		} else {
			pid_in = encoder.read();
			if (abs(pid_in - float_table.target) > error_threshold) {
				pid.Compute();
				Serial.print(pid_in);
				Serial.print(" : ");
				Serial.println(pid_out);
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
	union MotorKeyMessage msg_key;
	msg_key.num = Wire.read();
	/*
	Serial.print(msg_key.id_rw.rw ? "w" : "r");
	Serial.print(" : ");
	Serial.print(msg_key.id_rw.id);
	Serial.print(" | ");
	*/

	char* message_buffer_cpy = messsage_buf_out;
	for (int i = 0; i < byteCount; i++) {
		*message_buffer_cpy++ = Wire.read();
	}

	if (msg_key.id_rw.rw) {
		float_table_ptr[msg_key.id_rw.id] = atof(messsage_buf_out);
		if (msg_key.id_rw.id == motor_key_kp || msg_key.id_rw.id == motor_key_ki || msg_key.id_rw.id == motor_key_kd)
			pid.SetTunings(float_table.kp, float_table.ki, float_table.kd);
		if (msg_key.id_rw.id == motor_key_max_pwm)
			pid.SetOutputLimits(-float_table.max_pwm, float_table.max_pwm);
		if (msg_key.id_rw.id == motor_key_position)
			encoder.write(float_table.position);
	} else {
		sprintf(messsage_buf_out, "%c%f", msg_key.num, float_table_ptr[msg_key.id_rw.id]);
	}
}

void sendData() {
	//Serial.print("Sending: ");
	//Serial.println(messsage_buf_out);
	Wire.write(messsage_buf_out, MESSAGE_BUFFER_SIZE);
}
