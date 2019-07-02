//VARIABLES::::

float Kp=1,Ki=0,Kd=0;
float P=0, I=0, D=0, PID_value=0;
float num=0,error=0,denom=0;
float previous_error=0, previous_I=0;
int sensor[7]={0,0,0,0,0,0,0},val[7]={0,0,0,0,0,0,0},smin[7]={1024,1024,1024,1024,1024,1024,1024},smax[7]={0,0,0,0,0,0,0};
float weight[7]={-41.25,-27.5,-13.75,0,13.75,27.5,41.25};
int rmspeed,lmspeed,imspeed=100;

//FUNCTIONS::::
void calibrate_sensor(void);
void read_sensor_values(void);
void find_error(void);
void calculate_pid(void);
void motor_control(void);

void setup()
{
 pinMode(5,OUTPUT);
 pinMode(6,OUTPUT);
 pinMode(2,OUTPUT);
 pinMode(4,OUTPUT);
 pinMode(7,OUTPUT);
 pinMode(8,OUTPUT);
 Serial.begin(9600);
 digitalWrite(9,HIGH);
}

void loop() {
  if(digitalRead(11)){
    calibrate_sensor();
    }

  if(digitalRead(10)){
    delay(2000);
    while(digitalRead(10)==0){
      read_sensor_values();
      find_error();
      Serial.print(error);
      Serial.println();
      calculate_pid();
      //Serial.print(PID_value);
      //Serial.println();
      motor_control();
     }
   }

}
void calibrate_sensor(){
   int tim=millis();
   int temp,sec=0,actim,k;
   digitalWrite(13,HIGH);
   while(sec<=5000){
      actim=millis();
      sec=actim-tim;
      for(int k=0;k<7;k++){
          temp=analogRead(k);
          smin[k]=min(temp,smin[k]);
          smax[k]=max(temp,smax[k]);
      }
   }
   digitalWrite(13,LOW);
   return;
}


void read_sensor_values(){
  for(int i=0;i<7;i++){
    sensor[i]=analogRead(i);
  }
  for(int i=0;i<7;i++){
    val[i]=map(sensor[i],smin[i],smax[i],0,1024);
  }

  return;
}
void find_error(){
    num=(weight[0]*(val[6]-val[0]))+(weight[1]*(val[5]-val[1]))+(weight[2]*(val[4]-val[2]));
    for(int i=0;i<7;i++){
      denom+=val[i];
    }
    error=num/denom;
}
void calculate_pid(){
    P = error;
    I = I + previous_I;
    D = error-previous_error;

    PID_value = (Kp*P) + (Ki*I) + (Kd*D);

    previous_I=I;
    previous_error=error;
    return;
}
void motor_control(){
  int lmspeed = imspeed-PID_value;
  int rmspeed = imspeed+PID_value;

  constrain(lmspeed,0,255);
  constrain(rmspeed,0,255);

  analogWrite(5,lmspeed);
  analogWrite(6,rmspeed);

  digitalWrite(2,HIGH);
  digitalWrite(4,LOW);
  digitalWrite(8,LOW);
  digitalWrite(7,HIGH);

}
