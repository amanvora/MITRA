#include <LiquidCrystal.h>
#include <Wire.h>
#include <HMC5883L.h>
HMC5883L compass;
LiquidCrystal lcd(12, 11, 13, 10, 9, 6);
char name[15];
int error = 0;
int l_motor1 = 3;
int l_motor2 = 2;//DIGITAL pin in RIGHT OF JUMPER of h bridge
int r_motor1 = 5;
int r_motor2 = 4;//DIGITAL RIGHT OF JUMPER
int ir_pin = 1;// output of infrared sensor 
const int trigPin = 7;
const int echoPin = 8;
boolean testing_flag = 0;
boolean lcd_flag = 0;
int o = 0;  
float ultra_dist = 0,heading = 0;
int bot_state = 0;
int pwm_val = 170;
char ser_data;
/////////////////////             Mapping parameters         ////////////////////////////////

int ir_past=0,ir_present=0;//storing encoder values
int x = 0, y = 1, a = 0, b = 0, c = 0;
int head_1 = 155, head_2 = 50, head_3 = 289, head_4 = 219;
int head_state = 1,prev_head_state = 4,store_head = 4;
int pt_coord[7][2] = {
                        {0,0},
                        {4,0},
                        {4,2},
                        {6,0},
                        {6,-2},
                        {6,-4},
                        {6,-6}
                      };
int pharm_coord[2] = {4,2};
int nurse_coord[2] = {-100,-100};
int ward_coord[3][2] = {
                         {6,-2},
                         {6,-4},
                         {6,-6}
                        };
int curr_coord[2] = {6,-6};
int curr_pt = 7;
int points = 7;
int dist;
int dest = 0;
        /////////       for testing       //////////
int path[10];
int path_index = -1;
        ////////////////////////////////////////////

boolean path_plan = 0;
int path_points_enc = 15;
int connected_matr[7][3] = {
                              {2,0,0},
                              {1,3,4},
                              {2,0,0},
                              {2,5,0},
                              {4,6,0},
                              {5,7,0},
                              {6,0,0}
                            };
int conn_nodes[7] = {1,3,1,2,2,2,1};
int nurse_pt, pharm_pt = 3, bed_pt[3] = {5,6,7};
int rep_count = 0;
int pos[10];
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
  lcd.begin(16, 2);
  Wire.begin();
  compass = HMC5883L();
  error = compass.SetScale(1.3);
  error = compass.SetMeasurementMode(Measurement_Continuous);
  Serial.println("Wireless Connection established");
}

void loop()
{
  lcd_display();  
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
//    misaligned();
  }  
  /////////////////////              Go to destination                     /////////////////////////////
  if(Serial.available())
  {
     ser_data = '1';
    dest = bed_pt[(int)(ser_data-48)];
    Serial.print(dest);
    goto_dest(curr_pt,pharm_pt);
    Serial.print("\nPath\n");
    for(int i=0; i<=path_index; i++)
    {
     Serial.print(path[i]);   Serial.print("\t");  
    }  
    right();
    delay(500);
    right();
    delay(500);
    go_bot();
    goto_dest(curr_pt,dest);
    delay(10000);
    left();
    delay(500);
    left();
    delay(500);    
    go_bot();
  }
}

float ultrasonic()
{
  long duration, cm;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  cm =duration/29/2;
  delay(100);
  return(cm);
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
void lcd_display()
{
  while(lcd_flag == 0)
  {
    while(!Serial.available());
    ser_data = Serial.read();
    if(ser_data=='0')
    {
      lcd_flag = 1;
    }
    else
    {
      name[o] = ser_data;
      o++;
    }
  }
///////////            Display Medicine's name        ///////////////////////
  lcd.setCursor(0, 0);
  lcd.print("Medicine:");  
  lcd.setCursor(0, 1);
  lcd.print(name);  
}
void goto_dest(int st_pt, int dest)
{
  path_plan = 0;
  path_index = -1;
  int guessed_points = 0;  
  boolean check_more = 1;
  int array[2] = {0,0};
  int temp_array[5][2];
  for(int i=0; i<10; i++) path[i] = 0;
  for(int i=0; i<5; i++)
  {
    for(int j=0; j<2; j++)
    {
      temp_array[i][j] = 0;
    }
  }
  push_stack(st_pt,dest); if(path_plan) goto path_planned;
  while(path_plan!=1)
  {
    int k = 0;
    for(int i=0; i<3; i++)
    {
      if(path_index == 0)
      {
        if( connected_matr[curr_pt-1][i] != path[path_index] )
        {
          array[k++] = connected_matr[curr_pt-1][i];
          if(k == 2) i = 2;
        }
      }
      else if(path_index>0)
      {
        if( connected_matr[curr_pt-1][i] != path[path_index-1] )
        {
          array[k++] = connected_matr[curr_pt-1][i];
          if(k == 2) i = 2;
        }        
      }
    }
    if(array[0] != 0 && array[1] == 0)
    {
      push_stack(array[0],dest); if(path_plan) goto path_planned;
    }
    else
    {
      if(array[0] != 0)
      {
        if(conn_nodes[array[0]-1] == 1)
        {
          if(array[0] == dest)
          {
            push_stack(array[0],dest); if(path_plan) goto path_planned;
            guessed_points = 0;
          }
          else
          {
            push_stack(array[1],dest); if(path_plan) goto path_planned;
          }
          check_more = 0;
        }
      }
      if(array[1] != 0)
      {
        if(conn_nodes[array[1]-1] == 1)
        {
          if(array[1] == dest)
          {
            push_stack(array[1],dest); if(path_plan) goto path_planned;
            guessed_points = 0;
          }
          else
          {
            push_stack(array[0],dest); if(path_plan) goto path_planned;
          }
          check_more = 0;
        }
      }
      if(check_more)
      {
        check_more = 0;
        push_stack(array[0],dest); if(path_plan) goto path_planned;
        guessed_points++;
        for(int n=0; n<5; n++)
        {
          int m = 0;
          for(int L=0; L<3; L++)
          {
            if( connected_matr[curr_pt-1][L] != path[path_index-1] )
            {
              temp_array[n][m++] = connected_matr[curr_pt-1][L];
              if(m == 2) L = 2;
            }       
          }
          if(temp_array[n][0] != 0 && temp_array[n][1] == 0)
          {
            push_stack(temp_array[n][0],dest); if(path_plan) goto path_planned;
            guessed_points++;
          }
          else if(temp_array[n][0] != 0 && temp_array[n][1] != 0)
          {
            if(conn_nodes[temp_array[n][0]-1] == 1)
            {
              if(temp_array[n][0] == dest)
              {
                push_stack(temp_array[n][0],dest); if(path_plan) goto path_planned;
                guessed_points = 0;                
              }
              else
              {
                push_stack(temp_array[n][1],dest); if(path_plan) goto path_planned;
                guessed_points++;                
              }
            }
            else if(conn_nodes[temp_array[n][1]-1] == 1)
            {
              if(temp_array[n][1] == dest)
              {
                push_stack(temp_array[n][1],dest); if(path_plan) goto path_planned;
                guessed_points = 0;                
              }
              else
              {
                push_stack(temp_array[n][0],dest); if(path_plan) goto path_planned;
                guessed_points++;                
              }
            }
          }
          else if(temp_array[n][0] == 0 && temp_array[n][1] == 0)
          {
            for(int z=0; z<guessed_points; z++)  path[path_index-z] = 0;
            path_index -= guessed_points;
            guessed_points = 0;
            push_stack(array[1],dest); if(path_plan) goto path_planned;
            goto continue_loop;
          }
        }
      }continue_loop:;
    }
  }
  path_planned: ;
  curr_pt = path[0];
  curr_coord[x] = pt_coord[curr_pt-1][x];
  curr_coord[y] = pt_coord[curr_pt-1][y];
}
void push_stack(int node, int dest)
{  
  path_index++;
  path[path_index] = node;
  curr_pt = node;
  if(path[path_index] == dest)
  {
    path_plan = 1;
  }
}
void go_bot()
{
  int dest_head_state;
  for(int i=1; i<=path_index; i++)
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
    ///////////////////////////////////////////////////////////////////////
    if (curr_coord[x] == pt_coord[path[i]-1][x])
    {
      dist = pt_coord[path[i]-1][y] - curr_coord[y];
      if (dist>0)
      {
        dest_head_state = 2;
        if(head_state != dest_head_state)
          align(head_state,dest_head_state);
      }
      else
      {
        dest_head_state = 4;
        if(head_state != dest_head_state)
          align(head_state,dest_head_state);
      }
    }
    if (curr_coord[y] == pt_coord[path[i]-1][y])
    {
      dist = pt_coord[path[i]-1][x] - curr_coord[x];
      if (dist>0)
      {
        dest_head_state = 1;
        if(head_state != dest_head_state)
          align(head_state,dest_head_state);
      }
      else
      {
        dest_head_state = 3;
        if(head_state != dest_head_state)
          align(head_state,dest_head_state);
      }
    }
    ///////////////////////////////////////////////////////////////////////
    fwd(abs(dist));
    Serial.print(curr_pt);
    curr_pt = path[i];
    curr_coord[x] = pt_coord[path[i]-1][x];
    curr_coord[y] = pt_coord[path[i]-1][y];    
  }
}

void fwd(int dist)
{
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
  
  while(dist)
  {
    ultra_dist = ultrasonic();
    if(ultra_dist>15)
    {
      analogWrite(l_motor1, 1.1*pwm_val);
      digitalWrite(l_motor2,LOW);
      analogWrite(r_motor1, 1.1*pwm_val);
      digitalWrite(r_motor2,LOW);          
      /////////////////////       calculate curr_coord          ///////////////////////////////
      ir_present = analogRead(ir_pin);
      if (ir_present>500)
        ir_present = 1;
      else if (ir_present<100)
        ir_present = 0;
      if(ir_present != ir_past)
      {
        dist--;
      }
      delay(50);
      ir_past = ir_present;
    }
    else
      stop_bot();
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
          head_state = 4;
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
          head_state = 1;          
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
          head_state = 2;          
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
          head_state = 3;          
          stop_bot();            
          break;
  }
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
          head_state = 2;          
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
          head_state = 3;          
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
          head_state = 4;          
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
          head_state = 1;          
          stop_bot();
          break;           
  }
}
void align(int head, int dest_head)
{
  switch(head)
  {
    case 1:
            if(dest_head == 2) left();
            else if(dest_head == 4) right();
            break;
    case 2:
            if(dest_head == 3) left();
            else if(dest_head == 1) right();
            break;
    case 3:
            if(dest_head == 4) left();
            else if(dest_head == 2) right();
            break;
    case 4:
            if(dest_head == 1) left();
            else if(dest_head == 3) right();
            break;            
  }
}
