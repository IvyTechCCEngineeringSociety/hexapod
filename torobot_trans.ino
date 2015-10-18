#include <Servo.h>
Servo ser;
Servo servo1;
Servo servo2;
int JoyX = 1; // Connect pin X from the joystick to analog pin A0
int JoyY = 0; //Connect pin Y from the joystick to analog pin A1
int button = A2;//Connect press button from the joystick to analog pin A2
int X; //Value of pin X
int Y; //Value of pin Y
int a; //Value of push button
/*Setup=========== */
/*======Leg setup======*/
double fe = 2;// femur lenght 
double ti = 3.25;//tibia lenght
double cox = 1.0625;// coxa lenght

/*       Body setup 
      /\             /\      
     /  \L1-------R1/  \
    /\ /           \   /\
   /  \L2           R2/  \
     /\\           //\
    /  \L3-------R3/  \
  */
double LR1 = 3.5; //Distance between L1 & R1
double LR2 = 5.5; //Distance between L2 & R2
double LL3 = 5.875;//Distance between L2 & LR2

/*===Command Inputs===*/
double h;  //body height above the ground
double s;  //Projection of femur & tibia on the ground
double LR;  //Left - Right Ward
double FB;  //Front - Back Ward
double D;  //Rotate
double R;  //Roll
double P;  //Pitch
double W;  //Yaw
double LRM;
double FBM;

/*===Body & Legs Position==*/

double footpos[6][2];  //Array for the calculated coordinate of the legs legpos{x,y}
double bodycal[6][2]; //Array for the calculated coordinate of the body bodycal{x,y}
double angle[6][3];   //Array for the calculated angle of each joint
double ser_angle[6][3]; //Array for correct servo angle due to hardwares placement
/*===Parameters===*/
//Each legs will have its own parameters involve
// d : Height above the ground
// s : Projection of femur & tibia on the ground
//s' : Projection of femur & tibia on x axis
//L  : Distance between femur & tibia joint
//offset : Offset angel of each servo
//x-center : X-distance from center to each corner of the body
//y-center : Y-distance from center to each corner of the body
double par[6][4];     //Array of parameters 
double inipar[6][3]; //Array of initial parameters


void ik(){
  //This function computing the coordinate of 6 points of the body & foot according to the input commands
  // i : each point of the body & foot
  for(int i = 0; i < 6, i++){
    //Calculating body coordinate
    bodycal[i][0] = (inipar[i][1] + LRM)*cos(radians(R))*cos(radians(W))- (inipar[i][2] + FBM)*cos(radians(R))*sin(radians(W));//x coordinate
    bodycal[i][1] = (inipar[i][1] + LRM)*cos(radians(P))*sin(radians(W))+ (inipar[i][2] + FBM)*cos(radians(P))*cos(radians(W));//y coordinate
    //Calculating foot coordinate
    footpos[i][0] = ((cox + s)*cos(radians(inipar[i][0])) + LRM)*cos(radians(D)) - ((cox + s)*sin(radians(inipar[i]0])) + FBM)*sin(radians(D)) + inipar[i][1] ; //x coordinate
    footpos[i][1] = ((cox + s)*cos(radians(inipar[i][0])) + LRM)*sin(radians(D)) + ((cox + s)*sin(radians(inipar[i]0])) + FBM)*cos(radians(D)) + inipar[i][2] ; //y coordinate
    //Calculating the parameters
    par[i][0] = h + inipar[i][1]*sin(radians(R)) + inipar[i][2]*sin(radians(P));// Calculating d
    par[i][1] = sqrt(sq(s*cos(radians(inipar[i][0])) + (inipar[i][1]-bodycal[i][0]) + LRM) + sq(s*sin(radians(inipar[i][0])) + (inipar[i][2] - bodycal[i][1]) + FBM)); //calculating s
    par[i][2] = footpos[i][0] - bodycal[i][0];//calculating s'
    par[i][3] = sqrt(sq(par[i][0]) + sq(par[i][1]));//calculating L
  }
}

void angik(){
  for(int i = 0; i < 6; i++){
    angle[i][0] = degrees(acos(par[i][2] / (par[i][1] + cox)));  //coxa angle
    angle[i][1] = degrees(acos((sq(fe) + sq(par[i][3]) - sq(ti))/(2*fe*par[i][3]))) - degrees(atan(par[i][0]/par[i][1])); //femur angle
    angle[i][3] = degrees(acos((sq(fe) + sq(ti) - sq(par[i][3]))/(2*fe*ti)));  //tibia angle
  }
}

void correction(){
  for( int i = 0; i <3; i++){
    ser_angle[i][0] = 
  
void setup() {
  Serial.begin(9600);
  ser.attach(7);
  servo1.attach(8);
  servo2.attach(9);
  //R1
  inipar[0][0] = 30;
  inipar[0][1] = 1.75;
  inipar[0][2] = 2.9375;
  //R2
  inipar[1][0] = 0;
  inipar[1][1] = 2.75;
  inipar[1][2] = 0;
  //R3
  inipar[2][0] = -30;
  inipar[2][1] = 1.75;
  inipar[2][2] = -2.9375;
  //L1
  inipar[3][0] = 150;
  inipar[3][1] = -1.75;
  inipar[3][2] = 2.9375;
  //L2
  inipar[4][0] = 180;
  inipar[4][1] = -2.75;
  inipar[4][2] = 0;
  //L3
  inipar[5][0] = -150;
  inipar[5][1] = -1.75;
  inipar[5][2] = -2.9375;
}

void loop() {
  X = analogRead(JoyX);
  Y= analogRead(JoyY);
  b = map(X,0,1023,0,179);
  a = map(Y,0,1023,0,179);

 for (double m = 1.5; m < 4.5; m+=0.25)
 {
 hei(m);
 delay(500);
 }
 for(double m = 4.5; m > 1.5; m-= 0.25)
 {
   hei(m);
   delay(500);
 }
 
 delay(20);


}
void test(double e, double f)
{

  ser.write(e);
  servo1.write(f);
  servo2.write(f);
  delay(20);
}
void hei(double y)
//this function change the height of the body
//from the ground to the femur joint
{
  double i; // parameter for the height
  double ft;//distance from the femur joint to the tip of the tibia
  double tia,tiaL,tiaR;
  double fea,feaL,feaR;
  double theta;//angle of i and fe
  i = y;//map the default height at central pos
  ft = sqrt(i*i + fe*fe);//distance from femur joint to the tip of tibia on the ground
  //with respect to the height of the body above the ground
  theta = atan(i/fe)*180/PI;                     
  fea = (acos((fe*fe + ft*ft - ti*ti)/(2*fe*ft))*180/PI) - theta;//femur angle
  tia = (acos((fe*fe + ti*ti - ft*ft )/(2*fe*ti))*180/PI);//tibia angle
  //the angle to put in the servo will be varied depends on the direction
  //of the servo.
  feaR = map(fea + 90,0,180,500,2500);
  tiaR = map(180 - tia, 0 ,180, 500, 2500);
  feaL= map(90 - fea,0 ,180, 500, 2500);
  tiaL = map(tia,0 ,180, 500, 2500);
  moveall(2,6,11,feaR,100);
  moveall(31,27,22,feaL,100);
  moveall(3,7,12,tiaR,100);
  moveall(30,26,21,tiaL,100);
  delay(10);
}
void angle(double x1, double y1, double x2, double y2)
//this function turns the translational position of the body to the angles
//at which, each joint(servo) of the hexapod will move to
{
  //x1 is the x-axis translation of the body
  //y1 is the y-axis translation of the body
  //x2 is the off set for L
  //y2 is the off set for h
  x1=map(X,0,1023,-1.5,1.5);
  y1=map(Y,0,1023,-1.5,1.5);
  double coa,coa1,coa2,coa3,coa4,coa5,coa6;//coxa step angle to destination point
  double fea,fea1,fea2,fea3,fea4,fea5,fea6;//femur angle at destination point
  double tia,tia1,tia2,tia3,tia4,tia5,tia6;//tibia angle at destination point
  double L1,L2,L3,L4,L5,L6; // the leg length from the femur joint to the end of the tibia view from top
  double ft;//distance from femur joint to the end of the tibia view from side
  
  L = sqrt(y1*y1 + pow((-x1 + cox + fe*cos(PI/4)),2)) ;// the length of the leg at destination point
  ft = sqrt(h*h + pow((L-cox),2));// distance from femur joint to the destination point
  coa = atan(y1/(-x1 + cox + fe*cos(PI/4)))*180/PI;// angle of coxa at destination point
  fea = (acos((fe*fe + ft*ft - ti*ti)/(2*fe*ft))- atan(h/(L-cox)))*180/PI; 
  tia = (acos((fe*fe+ti*ti-ft*ft)/(2*fe*ti)))*180/PI;
  ser.write(90-coa);
  servo1.write(90-(45-fea));
  servo2.write(90+(45-tia));
}


void idle()// this function returns all the joints to its central pos
{
  //move the left coxas to the central pos
  moveall(1,6,10,1500,20); //left side
  moveall(23,27,32,1500,20); //right side
  //move the left femurs to the central pos
  moveall(2,7,11,1500,20); //left side
  moveall(22,26,31,1500,20);//right side
  //move the tibias to the central pos
  moveall(3,8,12,1500,20); //left side
  moveall(21,25,30,1500,20);// right side
}

void moven(int servo, int pos, int time) 
{
  Serial.print("#");
  Serial.print(servo);
  Serial.print("P");
  Serial.print(pos);
  Serial.print("T");
  Serial.println(time);
  delay(20);
}

void movetwo(int servo1, int servo2, int pos, int time)
{
  Serial.print("#");
  Serial.print(servo1);
  Serial.print("P");
  Serial.print(pos);
  Serial.print("#");
  Serial.print(servo2);
  Serial.print("P");
  Serial.print(pos);
  Serial.print("T");
  Serial.println(time);
  delay(time);
}

void moveall(int cox1, int servo2, int servo3,int pos,int time) 
{
  Serial.print("#");
  Serial.print(servo1);
  Serial.print("P");
  Serial.print(pos);
  Serial.print("#");
  Serial.print(servo2);
  Serial.print("P");
  Serial.print(pos);
  Serial.print("#");
  Serial.print(servo3);
  Serial.print("P");
  Serial.print(pos);
  Serial.print("T");
  Serial.println(time);
  delay(20);
}
