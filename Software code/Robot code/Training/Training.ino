#include <Wire.h>
#include <HMC5883L.h>
HMC5883L compass;
int error = 0;
int l_motor1 = 3;
int l_motor2 = 2;//DIGITAL pin in RIGHT OF JUMPER of h bridge
int r_motor1 = 5;
int r_motor2 = 4;//DIGITAL RIGHT OF JUMPER
int ir_pin = 1;// output of infrared sensor 
const int trigPin = 7;
const int echoPin = 8;
boolean testing_flag = 0;
float ultra_dist = 0,heading = 0;
int bot_state = 0;
int pwm_val = 170;
char ser_data;
/////////////////////             Mapping parameters         ////////////////////////////////

int ir_past=0,ir_present=0;//storing encoder values
int x = 0, y = 1, a = 0, b = 0, c = 0;
int head_1 = 163, head_2 = 34, head_3 = 284, head_4 = 224;
int head_state = 4,prev_head_state = 4,store_head;
int pt_coord[10][2] = {
                        {-100,-100},
                        {-100,-100},
                        {-100,-100},
                        {-100,-100},
                        {-100,-100},
                        {-100,-100},
                        {-100,-100},
                        {-100,-100},
                        {-100,-100},
                        {-100,-100}
                      };
int pharm_coord[2] = {-100,-100};
int nurse_coord[2] = {-100,-100};
int ward_coord[3][2] = {
                         {-100,-100},
                         {-100,-100},
                         {-100,-100}
                        };
int curr_coord[2] = {0,0};
int curr_pt;
int points = 0;
int dist;

        /////////       for training       //////////
int path_stack[25];
int train_path_index = 0;
        ////////////////////////////////////////////

        /////////       for testing       //////////
int path[10];
int path_index = -1;
        ////////////////////////////////////////////

boolean path_plan = 0;
int path_points_enc = 0;
int connected_matr[10][3];
int conn_nodes[10];
int nurse_pt, pharm_pt, ward_pt[3];
int rep_count = 0;
int pos[10];
//int j;
///////////////////////////////////////////////////////////////////////////////////
void setup()
{
  pinMode(l_motor1,OUTPUT);
  pinMode(l_motor2,OUTPUT);
  pinMode(r_motor1,OUTPUT);
  pinMode(r_motor2,OUTPUT);

//  pinMode(ir_pin, INPUT);
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  Serial.begin(9600);
  Wire.begin();
  compass = HMC5883L();
  error = compass.SetScale(1.3);
  error = compass.SetMeasurementMode(Measurement_Continuous);
  Serial.println("Wireless Connection established\nCheck alignment. y/n?");
  while(!Serial.available());
  ser_data = Serial.read();
  if(ser_data == 'y')  align();
  for (int i=0; i<25; i++) path_stack[i] = 0;
  record('n');
}

void loop()
{
  heading = magneto();
  prev_head_state = head_state;
  if     (heading > head_1-15 && heading < head_1+15)    head_state = 1;
  else if(heading > head_2-15 && heading < head_2+15)    head_state = 2;
  else if(heading > head_3-15 && heading < head_3+15)    head_state = 3;
  else if(heading > head_4-15 && heading < head_4+15)    head_state = 4;
  else head_state = 0;
  if(head_state == 0)
  {
    store_head = prev_head_state;
    head_state = prev_head_state;
  }

  //////////////       training       ///////////////////
  if (Serial.available())
  {
    ser_data = Serial.read();
    switch(ser_data)
    {
    case 'w':
              Serial.println("Forward");
              fwd();                
              break;
    case 'a':
//                Serial.println("Left");
              left();
              break;
    case 'd':
//                Serial.println("Right");
              right();
              break;
    case 's':
//                Serial.println("Stop");
              stop_bot();
              break;
    case 'r':
              Serial.println("Record");
              record('n');
              break;
    case 'p':
              Serial.println("Pharmacy");
              record('P');
              break;
    case '1':
              Serial.println("Bed1");
              record('1');
              break;
    case '2':
              Serial.println("Bed2"); 
              record('2');
              break;                 
    case '3':
              Serial.println("Bed3");
              record('3');
              break;
    case 'n':
              Serial.println("Nurse");
              record('N');
              break;
    case 'm':
              Serial.println("Delete point recorded");
              points--;
              train_path_index--;
              break;
    case 'x':
              Serial.println("Training complete");
              for(int i=0;i<points;i++)
              {
                for(int j=0;j<3;j++)
                {
                  connected_matr[i][j] = 0;
                }
              }
              for(int i=0;i<points;i++) pos[i] = 0;
              for(int i=0;i<points;i++) conn_nodes[i] = 0;  
    
              for(int i=0;i<points;i++)
              {
                int j = 0;
                rep_count = 0;
                while(j<path_points_enc)
                {
                  if( (i+1)==path_stack[j] )
                  {
                    rep_count++;
                    if(i==0)
                    {
                      connected_matr[i][0] = path_stack[j+1];
                      conn_nodes[i]++;
                    }
                    else if(i==(points-1))
                    {
                      connected_matr[i][0] = path_stack[j-1];
                      conn_nodes[i]++;
                    }
                    else
                    {
                      if(connected_matr[i][0] != path_stack[j-1]	&&
                         connected_matr[i][1] != path_stack[j-1]	&&
                         connected_matr[i][2] != path_stack[j-1])            connected_matr[i][conn_nodes[i]++] = path_stack[j-1];
    
                      if(connected_matr[i][0] != path_stack[j+1]	&&
                         connected_matr[i][1] != path_stack[j+1]	&&
                         connected_matr[i][2] != path_stack[j+1])            connected_matr[i][conn_nodes[i]++] = path_stack[j+1];
                    }
                  }
                  j++;
                }
                pos[i] = rep_count;
              }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////                
              for(int i=0;i<points;i++)
              {
                Serial.print(i+1); Serial.print(" ("); Serial.print(conn_nodes[i]); Serial.print(") : ");
                Serial.print(pt_coord[i][x]);  Serial.print("\t");  Serial.println(pt_coord[i][y]);
              }
              for(int i=0;i<path_points_enc;i++)
              {
                Serial.print(path_stack[i]);  Serial.print("\t");
              }                                
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////                
              //Print the connected nodes
              Serial.print("\nConnected nodes\n");
              for(int i=0;i<points;i++)
              {
                for(int j=0;j<3;j++)
                {
                  Serial.print(connected_matr[i][j]);
                  Serial.print(" ");
                }
                Serial.print("\n");
              }
              testing_flag = 1;
              break;
    default:
              Serial.println("Invalid input");  
              break;             
    }
  }
}

float magneto()
{
  MagnetometerRaw raw = compass.ReadRawAxis();
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  float declinationAngle = 0.0457;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  float headingDegrees = heading * 180/M_PI; 
  //Serial.println(headingDegrees);
  return headingDegrees;  
}
void record(char key)
{
  switch(key)
  {
  case 'n':
            pt_coord[points][x] = curr_coord[x];
            pt_coord[points][y] = curr_coord[y];
            break;
  case 'N':
            pt_coord[points][x] = curr_coord[x];
            pt_coord[points][y] = curr_coord[y];
            nurse_coord[x] = curr_coord[x];
            nurse_coord[y] = curr_coord[y];
            nurse_pt = points+1;            
            break;
  case 'P':
            pt_coord[points][x] = curr_coord[x];
            pt_coord[points][y] = curr_coord[y];
            pharm_coord[x] = curr_coord[x];
            pharm_coord[y] = curr_coord[y];              
            pharm_pt = points+1;            
            break;
  case '1':
            pt_coord[points][x] = curr_coord[x];
            pt_coord[points][y] = curr_coord[y];
            ward_coord[0][x] = curr_coord[x];
            ward_coord[0][y] = curr_coord[y];              
            ward_pt[0] = points+1;            
            break;
  case '2':
            pt_coord[points][x] = curr_coord[x];
            pt_coord[points][y] = curr_coord[y];
            ward_coord[1][x] = curr_coord[x];
            ward_coord[1][y] = curr_coord[y];              
            ward_pt[1] = points+1;                        
            break;
  case '3':
            pt_coord[points][x] = curr_coord[x];
            pt_coord[points][y] = curr_coord[y];
            ward_coord[2][x] = curr_coord[x];
            ward_coord[2][y] = curr_coord[y];              
            ward_pt[2] = points+1;                        
            break;              
  }
  curr_pt = points+1;
  Serial.print("Recorded point "); Serial.println(points+1);
  Serial.print((float)pt_coord[points][x]);  Serial.print("\t");  Serial.println((float)pt_coord[points][y]);
  path_stack[train_path_index++] = points+1;
  path_points_enc++;
  points++;
}
void fwd()
{
  char stop_char = '0';
  int encoder_count = 0;
  analogWrite(l_motor1, 1.1*pwm_val);
  digitalWrite(l_motor2,LOW);
  analogWrite(r_motor1, 1.1*pwm_val);
  digitalWrite(r_motor2,LOW);    
  
  ir_present = analogRead(ir_pin);
  if (ir_present>500)
    ir_present = 1;
  else if (ir_present<100)
    ir_present = 0;      
  ir_past = ir_present;
  
  while(stop_char!='s')
  {
    stop_char = Serial.read();
    /////////////////////       calculate curr_coord          ///////////////////////////////
    ir_present = analogRead(ir_pin);
    if (ir_present>500)
      ir_present = 1;
    else if (ir_present<100)
      ir_present = 0;
    if(ir_present != ir_past)
    {
      if(head_state == 0)  head_state = store_head;
      switch(head_state)
      {
      case 1:
              curr_coord[x] = curr_coord[x] + 1;
              break;
      case 2:
              curr_coord[y] = curr_coord[y] + 1;
              break;
      case 3:
              curr_coord[x] = curr_coord[x] - 1;
              break;
      case 4:
              curr_coord[y] = curr_coord[y] - 1;
              break;
      }
      Serial.print((float)curr_coord[x]);  Serial.print("\t");  Serial.println((float)curr_coord[y]);
      point_revisit();
    }
    delay(100);
    ir_past = ir_present;          
  }
  stop_bot();
}
void rev()
{
  analogWrite(l_motor1, 255-pwm_val);
  digitalWrite(l_motor2,HIGH);
  analogWrite(r_motor1, 255-pwm_val);
  digitalWrite(r_motor2,HIGH);    
}
void stop_bot()
{
  analogWrite(l_motor1, 0);
  digitalWrite(l_motor2,LOW);
  analogWrite(r_motor1, 0);
  digitalWrite(r_motor2,LOW);  
}
void move_right(int pwm)
{
  analogWrite(l_motor1, pwm);
  digitalWrite(l_motor2,LOW);
  analogWrite(r_motor1, 255-pwm);
  digitalWrite(r_motor2,HIGH);
}
void right()
{
  int curr_state = head_state;
  int head_err = 1000;
  switch(curr_state)
  {
  case 0:
          head_state = store_head;    
  case 1:
          while(head_err > 5)
          {
            heading = magneto();
            head_err = abs((head_4 - (int)heading + 360)%360);
            if(head_err > 30)  move_right(0.85*pwm_val);
            else move_right(0.75*pwm_val);
          }
          stop_bot();
          break;
  case 2:
          while(head_err > 5)
          {
            heading = magneto();
            head_err = abs((head_1 - (int)heading + 360)%360);
            if(head_err > 30)  move_right(0.85*pwm_val);
            else move_right(0.75*pwm_val);
          }
          stop_bot();            
          break;
  case 3:
          while(head_err > 5)
          {
            heading = magneto();
            head_err = abs((head_2 - (int)heading + 360)%360);
            if(head_err > 30)  move_right(0.85*pwm_val);
            else move_right(0.75*pwm_val);
          }
          stop_bot();            
          break;
  case 4:
          while(head_err > 5)
          {
            heading = magneto();
            head_err = abs((head_3 - (int)heading + 360)%360);
            if(head_err > 30)  move_right(0.85*pwm_val);
            else move_right(0.75*pwm_val);
          }
          stop_bot();            
          break;
  }
  Serial.println(curr_state);  
  Serial.println(heading);  
}
void move_left(int pwm)
{
  analogWrite(l_motor1, 255-pwm);
  digitalWrite(l_motor2,HIGH);
  analogWrite(r_motor1, pwm);
  digitalWrite(r_motor2,LOW);
}
void left()
{
  int curr_state = head_state;
  int head_err = 1000;
  switch(curr_state)
  {
  case 0:
          head_state = store_head;
  case 1:
          while(head_err > 5)
          {
            heading = magneto();
            head_err = abs((head_2 - (int)heading + 360)%360);
            if(head_err > 30)  move_left(0.75*pwm_val);
            else move_left(0.65*pwm_val);               
          }
          stop_bot();
          break;
  case 2:
          while(head_err > 5)
          {
            heading = magneto();
            head_err = abs((head_3 - (int)heading + 360)%360);
            if(head_err > 30)  move_left(0.85*pwm_val);
            else move_left(0.75*pwm_val);
          }
          stop_bot();
          break;
  case 3:
          while(head_err > 5)
          {
            heading = magneto();
            head_err = abs((head_4 - (int)heading + 360)%360);
            if(head_err > 30)  move_left(0.85*pwm_val);
            else move_left(0.75*pwm_val);
          }
          stop_bot();
          break;
  case 4:
          while(head_err > 5)
          {
            heading = magneto();
            head_err = abs((head_1 - (int)heading + 360)%360);
            if(head_err > 30)  move_left(0.85*pwm_val);
            else move_left(0.75*pwm_val);
          }
          stop_bot();
          break;           
  }
  Serial.println(curr_state); 
  Serial.println(heading);
}
void point_revisit()
{
  for(int i=0;i<points-1;i++)  
  {
   if( (curr_coord[x] == pt_coord[i][x]) && (curr_coord[y] == pt_coord[i][y]) )
   {
     path_stack[train_path_index++] = i+1;
     path_points_enc++;
     Serial.print("\nRevisited point ");  Serial.print(i+1);  Serial.print("\n");
     curr_pt = i+1;
     i = points;  // Make the for loop end
   }
  }
}
void align()
{
  char align_char;
  int head_count = 0;
  boolean aligned_head1 = 0, align_flag = 0;
  int left_count = 0;
  int right_count = 0;
  int head_err = 1000;
  Serial.print("Align to heading ");
  Serial.println(head_count+1);  
  while(align_flag==0)
  {
    heading = magneto();
//            Serial.println(heading);
    if(Serial.available())
    {
      align_char = Serial.read();
      switch(align_char)
      {
      case 'a':
                move_left(0.9*pwm_val);
                if(aligned_head1)
                left_count++;
//                Serial.println("Left");
                delay(100);
                stop_bot();
                break;
      case 'd':
                move_right(0.9*pwm_val);
                if(aligned_head1)                
                left_count--;
//                Serial.println("Right");                
                delay(100);
                stop_bot();
                break;
      case '1':
                heading = magneto();
                head_1 = heading;
                aligned_head1 = 1;
                Serial.print("Heading 1 : ");
                Serial.println(head_1);
                head_count++;                  
                Serial.print("Align to heading ");
                Serial.println(head_count+1);                  
                break;
      case '2':
                Serial.print(left_count); Serial.print("counts turn left");
                left_count = 0;
                heading = magneto();
                head_2 = heading;
                Serial.print("Heading 2 : ");
                Serial.println(head_2);                
                head_count++;                  
                Serial.print("Align to heading ");
                Serial.println(head_count+1);                  
                break;
      case '3':
                Serial.print(left_count); Serial.print("counts turn left");
                left_count = 0;      
                heading = magneto();
                head_3 = heading;
                Serial.print("Heading 3 : ");
                Serial.println(head_3);                
                head_count++;
                Serial.print("Align to heading ");
                Serial.println(head_count+1);
                break;
      case '4':
                Serial.print(left_count); Serial.print("counts turn left");
                left_count = 0;      
                heading = magneto();
                head_4 = heading;
                Serial.print("Heading 4 : ");
                Serial.println(head_4);                
                head_count++;
                align_flag = 1;
                Serial.println("Go to initial heading. Aligning to heading 1");
                break;
      }
    }
    if(align_flag)
    {
      head_state = 4;
      left();
      stop_bot();
      Serial.println(heading);
      Serial.println("Aligned\n");    
    }
  }  
}
