#include "SPI_MSTransfer_T4/SPI_MSTransfer_T4.h"
#include "Encoder.h"
#include "Arduino.h"
#include "SPI.h"
const int slave_ID=3;
SPI_MSTransfer_T4<&SPI, slave_ID> mySPI;

Encoder myEnc1(23, 22);
Encoder myEnc2(21, 20);
Encoder myEnc3(19, 18);

int pins[3][5]={  {14,15,16,23,22},
                  {3,4,5,21,20},
                  {6,7,8,19,18}             
                
                };//dir,pwm,slp,+a_vout_a,vout_b

class motor // 單一馬達的類別
{

  public:
    int mPID;
    bool dir;
    int dirpin;
    int pwmpin;
    int slppin;
    int encoder_a_pin;
    int encoder_b_pin;
    float pwm=0;
    float Angle;
    float errorAngle=0;
    float integralErrorAngle=0;
    float pulsePerRotation=480;
    float Ki=0.15,Kp=0.9,Kd=0.05;
    float deltaTime = 0.010; 
    float target_angle;
    motor a();
    motor b();
    motor c();
    motor() {};

    ~motor() {};

    motor(motor a,motor b,motor c){a=a;b=b;c=c;};

    motor(int pid,int *pins)//建立motor類別;所需參數:pid->馬達在該版的編號、pins->馬達在該版需要的腳位
    {
      mPID=pid;
      dirpin=*pins;
      pwmpin=*pins+1;
      slppin=*pins+2;
      encoder_a_pin=*pins+3;
      encoder_b_pin=*pins+4;
    };

    motor(int pid,int *pins,float ki,float kp,float kd)
    {
      mPID=pid;
      dirpin=*pins;
      pwmpin=*pins+1;
      slppin=*pins+2;
      encoder_a_pin=*pins+3;
      encoder_b_pin=*pins+4;
      Ki=ki;
      Kp=kp;
      Kd=kd;
    };

    float get_angle(long encoder_val)//將encoder讀到的值轉成實際角度
    {
      Angle=float(encoder_val)/ pulsePerRotation * 360;
      return Angle;
    }

    void generate_control_signal(float target,bool relative)//生成pwm訊號與dir訊號
    { 
      target_angle=target;
      float new_errAngle = target_angle - Angle;
      float errorAngleDot = (new_errAngle - errorAngle) / deltaTime;
      // float new_integralErrorAngle = integralErrorAngle + new_errAngle * deltaTime;
      float new_integralErrorAngle = (fabs(int(integralErrorAngle))>1500) ?  (1+int(new_errAngle<0)*-2)*1000:integralErrorAngle + new_errAngle * deltaTime;
      float controlSignal = Kp * new_errAngle + Kd * errorAngleDot + Ki*new_integralErrorAngle;

      if (controlSignal>0) dir=LOW;
      else dir=HIGH;

      errorAngle=new_errAngle;
      pwm=controlSignal;
      integralErrorAngle=new_integralErrorAngle;
    }

    void run()
    {
      // Serial.print("motor");
      // Serial.print(mPID);

      int pwmValue = fabs(int(pwm));
      if (pwmValue > 128){
        pwmValue = 128;
      }
      analogWrite(pwmpin,pwmValue);
      digitalWrite(dirpin,dir);

      // return pwmValue;
    }

  //   void get_SPI()
  //   {
  //     board.onTransfer(_GET_SPI);
  //   }

  //   void brake();
    
  // private:
  //   void _GET_SPI(uint16_t *buffer, uint16_t length, AsyncMST info){
  //     Serial.print(" --> PacketID: "); Serial.println(buffer[0]);
  //     if (buffer[0] == mPID){
  //       get_angle(buffer[0]);
  //     }
  //   }
};

motor motor1(0 , pins[0]);
motor motor2(1 , pins[1]);
motor motor3(2 , pins[2]);
int tt=0;
void myCB(uint16_t *buffer, uint16_t length, AsyncMST info) {
  
  if(millis()-tt>1000){
    Serial.print(info.packetID);
    Serial.print(" ");
    Serial.println(buffer[1]);
    Serial.println();
    tt=millis();
  }



  if(int(info.packetID)==slave_ID){
      if (buffer[0]==motor1.mPID){
        // motor1.generate_control_signal(buffer[1],true);
        if (motor1.target_angle!=buffer[1]){
          motor1.dir=buffer[2];
          motor1.target_angle=buffer[1]*((-2)*int(buffer[2])+1);
        }

      }
      if (buffer[0]==motor2.mPID){
        // motor2.generate_control_signal(buffer[1],true);
        if(motor2.target_angle!=buffer[1]){
          motor2.dir=buffer[2];
          motor2.target_angle=buffer[1]*((-2)*int(buffer[2])+1);
        }

      }
      if (buffer[0]==motor3.mPID){
        // motor3.generate_control_signal(buffer[1],true);
        if(motor3.target_angle!=buffer[1]){
          motor3.dir=buffer[2];
          motor3.target_angle=buffer[1]*((-2)*int(buffer[2])+1);
        }
      }
  }
}
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(1); 
  
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  pinMode(22, INPUT);
  pinMode(23, INPUT);
  digitalWrite(16, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(8, HIGH);
  SPI.begin();
  mySPI.begin();
  mySPI.onTransfer(myCB);
  delay(3000);
}

void loop() {

  delay(1);
  mySPI.events();
  static uint32_t t = millis();
  motor1.get_angle(myEnc1.read());
  motor1.generate_control_signal(motor1.target_angle,true);
  motor1.run();
  motor2.get_angle(myEnc2.read());
  motor2.generate_control_signal(motor2.target_angle,true);
  motor2.run();
  motor3.get_angle(myEnc3.read());
  motor3.generate_control_signal(motor3.target_angle,true);
  motor3.run();


  if ( millis() - t > 1000 ) {
    mySPI.events();
    Serial.print("motor1_integralErrorAngle:");
    Serial.println(motor1.target_angle);
    Serial.println(motor2.target_angle);
    Serial.println(motor3.target_angle);

    Serial.print("motor1:");
    Serial.print(motor1.Angle);
    Serial.print(" , ");
    Serial.print("motor1_encoder: ");
    Serial.println(myEnc1.read());
    // Serial.print(",");
    // Serial.print("motor2_angle:");
    Serial.print("motor2:");
    Serial.print(motor2.Angle);
    Serial.print(" , ");
    Serial.print("motor2_encoder: ");
    Serial.println(myEnc2.read());
    // Serial.print("motor3_angle:");
    Serial.print("motor3:");
    Serial.print(motor3.Angle);
    Serial.print(" , ");
    Serial.print("motor3_encoder: ");
    Serial.println(myEnc3.read());
    Serial.println();
    t = millis();
  }
    // Serial.print("motor1_angle:");

    // Serial.print("motor1 dir: ");
    // Serial.print(motor1.target_angle);

    // Serial.print("motor2 dir: ");
    // Serial.print(motor2.target_angle);

    // Serial.print("motor3 dir: ");
    // Serial.print(motor3.target_angle);

  //   Serial.print("millis: "); Serial.println(millis());
  //   Serial.print("MOTOR1: "); Serial.println(myEnc1.read());
  //   Serial.print("MOTOR2: "); Serial.println(myEnc2.read());
  //   Serial.print("MOTOR3: "); Serial.println(myEnc3.read());
  //   Serial.println();
    
  // }
  uint16_t buf1[9] = { motor1.mPID ,fabs(motor1.Angle), ( motor1.Angle<0 ),motor2.mPID ,fabs(motor2.Angle), ( motor2.Angle<0 ),motor3.mPID ,fabs(motor3.Angle), ( motor3.Angle<0 )};
  // uint16_t buf2[3] = { motor2.mPID ,motor2.Angle, ( motor2.Angle<0 )};
  // uint16_t buf3[3] = { motor3.mPID ,motor3.Angle, ( motor3.Angle<0 )};
  // Serial.println(buf1[0]);
  // Serial.println(buf2[0]);
  // Serial.println(buf3[0]);
  mySPI.transfer16(buf1, 9, slave_ID-1);
  // mySPI.transfer16(buf2, 3, 0);
  // mySPI.transfer16(buf3, 3, 0);
  // }
}

