
/*
----------------------------------------------------------------------------
                            PROJECT DESCRIPTION
----------------------------------------------------------------------------
This is a code for controlling a 6 DOF robot which use a combination of the 
Waveshare SC15 and ST3215 servos. The method for installation has been provided
in the README.md file.
  * Method used the system works on a ROS2 network present on the raspberry pi
  * Which communicates to the ESP32 via a Serial network.
---------------------------------------------------------------------------
*/
//sizeof 
//importing necessary libraries
#include <Arduino.h>
#include <SCServo.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>

//Initializing Display
//Display Resolution is 128 pixels by 32 pixels
Adafruit_SSD1306 display(128,32);

//PIN numbers to communicate with the servos
#define S_RXD 18
#define S_TXD 19

SCSCL sc;
SMS_STS st;

//Variables

int id_st[5] = {1, 2, 3, 4, 5};
int id_sc[2] = {6, 7};

int id_status[7] = {-1,-1,-1,-1,-1,-1,-1};
int *torque_stat;
int *state_arr;
int **robot_state_arr;
int * connected_servos;

//Debug Commands
int ping_test(SMS_STS st, int id);
int ping_test(SCSCL sc, int id);
int *ping_test(int id[]);

//Util Commands
void set_joint(int id, int position, int velocity, int acceleration);
void set_multiple_joints(u8 id[], s16 position[], u16 velocity[], u8 acceleration[]);
//void set_joint_trajectory(int id[], int position[][], int velocity[][], int acceleration[][]);

//FeedBack Commands
int * get_joint_state(int);
int **get_robot_state();

// Test Commands
int *torque_enable(int id[], int status_to_set[]);//review once
int servo_classifier(int id);

//Config Commands
void set_register(int,int); // sets the ID of the servo motors

void display_text(String);
void display_text(String, int);
void setup()
{

  //Starting display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();

  Serial.begin(115200);

  //Serial used as a debug tool
  Serial.println("Initializing robotic Arm");
  display_text("Welcome!\nInitializing arm");
  

  //Starting communication with Servos
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sc.pSerial = &Serial1;
  st.pSerial = &Serial1;

  // Checking number of servos connected..
  Serial.println("Pinging to check connected Servos");
  display_text("Welcome!\nPinging Servos...");
  
  //Checking and creating list of connected servos
  int servo_id_list[] = {1, 2, 3, 4, 5, 6, 7};
  connected_servos = ping_test(servo_id_list);
  int no = 0;
  for (int i = 0; i < 7; i++)
  {
    if (connected_servos[i] != -1)
    {
      no++;
    }
  }
  display_text("Robotic Arm\nNo of Servos :", no);

  //Need to calibrate arms buy switches
  //1) make use of limit switches in order to reach 0 and 180 and then callibrate

}

void loop()
{
  display_text("Robotic arm initialized\nWaiting for cmds");
  
}

int servo_classifier(int id)
{
  if(id == id_sc[0] || id == id_sc[1])
  {
    return 1;
  }
  return 0;
}

int ping_test(SCSCL sc, int id)
{
  int ID = sc.Ping(id);
  if(ID != -1)
  {
    Serial.print("Servo ID is");
    Serial.println(ID,DEC);
    delay(100);
    return ID;
    }

  Serial.println("Servo ID Not found! Check connection");
  return -1;
}


int ping_test(SMS_STS st, int id)
{
  
    int ID = st.Ping(id);
    if(ID != -1)
    {
      Serial.print("Servo ID is");
      Serial.println(ID,DEC);
      delay(100);
      return ID;
    }
    Serial.println("Servos Not found! Check connection");
    return -1;
  }

int * ping_test(int id[7])
{
  for (int i = 0; i < 7; i++)
  {
    if(servo_classifier(id[i]))
    {
      id_status[i] = ping_test(st, id[i]);
    }
    else
    {
      id_status[i] = ping_test(sc, id[i]);
    }
  }
  return id_status;
}

int * torque_enable(int id[], int status_to_set[])
{
  free(torque_stat);
  torque_stat = (int *)malloc(sizeof(id));
  for (int i = 0; i < sizeof(id) / sizeof(int); i++)
  {
    if(servo_classifier(id[i]))
    {
      st.EnableTorque(id[i], status_to_set[i]);
      torque_stat[i] = status_to_set[i];
    }
    else
    {
      sc.EnableTorque(id[i], status_to_set[i]); //assuming id to enable is correctly given as an input
      torque_stat[i] = status_to_set[i];
    }
  }
  return torque_stat;
}

//Position Mode Set joints
void set_joint(int id, int position, int velocity, int acceleration)
{
  if (servo_classifier(id))
  {
  
    sc.WritePosEx(id, position, velocity, acceleration);
    while(abs(sc.ReadPos(id)- position) > 5) //System implementation that will delay until servo is at desired position
    delay(1);
  } 
  else
  {
    st.WritePosEx(id, position, velocity, acceleration);
    while(abs(st.ReadPos(id)- position) > 5)
    {

      Serial.println(st.ReadPos(id));
      delay(1);
    } // System implementation that will delay until servo is at desired position
  }
}

//Assum
void set_multiple_joints(u8 id[], s16 position[], u16 velocity[], u8 acceleration[])
{
  int sc_index[2] = {-1,-1}, st_index[5] = {-1,-1,-1,-1,-1};
  int sc_curr = 0;
  int st_curr = 0;
  //might not work need to test
  for (int i = 0; i < (sizeof(id) / sizeof(u8)); i++)
  {

    if(servo_classifier(id[i]))
    {
      sc_index[sc_curr] = i;
      sc_curr++;
    }
    else
    {
      st_index[st_curr] = i;
      st_curr++;
    }
  }
  u8 *st_ind = (u8 *)malloc(st_curr * sizeof(u8));
  s16 *st_pos = (s16 *)malloc(st_curr * sizeof(s16));
  u16 *st_vel = (u16 *)malloc(st_curr * sizeof(u16));
  u8 *st_acc = (u8 *)malloc(st_curr * sizeof(u8));
  for (int i = 0; i < st_curr;i++)
  {
    st_ind[i] = id[st_index[i]];
    st_pos[i] = position[st_index[i]];
    st_vel[i] = velocity[st_index[i]];
    st_acc[i] = acceleration[st_index[i]];
  }
  st.SyncWritePosEx(st_ind, st_curr, st_pos, st_vel, st_acc);
  for (int i = 0; i < sc_curr;i++)
  {
    sc.WritePosEx(id[sc_index[i]], position[sc_index[i]], velocity[sc_index[i]], acceleration[sc_index[i]]);
  }
  delay(2000);// set a 2 second delay as default but need to change later
  free(st_ind);
  free(st_pos);
  free(st_vel);
  free(st_acc);
}


void set_register(int prev_id, int new_id)
{
  if(servo_classifier(prev_id))
  {
    for (int i = 0; i < 2;i++)
    {
      if(prev_id == id_sc[i])
      {
        sc.unLockEprom(prev_id);
        sc.writeByte(prev_id, SCSCL_ID, new_id);
        sc.LockEprom(new_id);
        id_sc[i] = new_id;
      }
    }
  }
  else
  {
    for (int i = 0; i < 5;i++)
    {
      if(prev_id == id_st[i])
      {
        st.unLockEprom(prev_id);
        st.writeByte(prev_id, SMS_STS_ID, new_id);
        st.LockEprom(new_id);
        id_st[i] = new_id;
      }
    }
  }
}
 
//FeedBack Commands
int * get_joint_state(int id)
{
  free(state_arr);
  state_arr = (int *)malloc(8 * sizeof(int));
  if (servo_classifier(id))
  {
    state_arr[0] = sc.ReadPos(id);
    state_arr[1] = sc.ReadSpeed(id);
    state_arr[2] = sc.ReadLoad(id);
    state_arr[3] = sc.ReadVoltage(id);
    state_arr[4] = sc.ReadTemper(id);
    state_arr[5] = sc.ReadMove(id);
    state_arr[6] = sc.ReadCurrent(id);
    state_arr[7] = sc.ReadMode(id);
  }
  else
  {
    state_arr[0] = st.ReadPos(id);
    state_arr[1] = st.ReadSpeed(id);
    state_arr[2] = st.ReadLoad(id);
    state_arr[3] = st.ReadVoltage(id);
    state_arr[4] = st.ReadTemper(id);
    state_arr[5] = st.ReadMove(id);
    state_arr[6] = st.ReadCurrent(id);
    state_arr[7] = st.ReadMode(id);
  }
  return state_arr;
}

int **get_robot_state()
{
  free(robot_state_arr);
  robot_state_arr = (int **)malloc(7 * sizeof(int *));
  int *temp_arr;
  for (int i = 1; i < 8; i++)
  {
    temp_arr = get_joint_state(i);
    robot_state_arr[i - 1] = (int *)malloc(7 * sizeof(int));
    for (int j = 0; j < 7;j++)
    {
      robot_state_arr[i - 1][j] = temp_arr[j];
    }
  }

  return robot_state_arr;
}

void display_text(String txt)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(txt);
  //display.println("Initializing robotic arm..");
  display.display(); 
}
void display_text(String txt,int dat)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(txt);
  display.println(dat);
  // display.println("Initializing robotic arm..");
  display.display(); 
}