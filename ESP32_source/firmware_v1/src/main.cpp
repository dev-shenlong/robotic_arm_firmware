
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
//importing necessary libraries
#include <Arduino.h>
#include <SCServo.h>
#include <math.h>

//PIN numbers to communicate with the servos
#define S_RXD 18
#define S_TXD 19

SCSCL sc;
SMS_STS st;

int id_st[5] = {1, 2, 3, 4, 5};
int id_sc[2] = {6, 7};

int id_status[7] = {0,0,0,0,0,0,0};
int *torque_stat;

//Setup Commands
int ping_test(SMS_STS st, int id);
int ping_test(SCSCL sc, int id);
int *ping_test(int *id[]);

//Util Commands
void set_joint(int id, int position, int velocity, int acceleration);
void set_multiple_joints(u8 id[], s16 position[], u16 velocity[], u8 acceleration[]);
//void set_joint_trajectory(int id[], int position[][], int velocity[][], int acceleration[][]);

//FeedBack Commands
int *get_joint_state();

// Test Commands
int *torque_enable(int id[], int status_to_set[]);//review once

//Config Commands
void set_register(int,int); // sets the ID of the servo motors

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sc.pSerial = &Serial1;
  st.pSerial = &Serial1;
  
}

void loop()
{
  //st.WritePosEx(1,2048,4000,50);
  
  sc.WritePos(2, 1000, 1500, 50);
  delay(10000);
  //st.WritePosEx(1, 0, 3000, 50);
  sc.WritePos(2, 0, 1500, 50);
  delay(5000);
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

int * ping_test(int id[])
{
  for (int i = 0; i < sizeof(id) / (sizeof(id[0])); i++)
  {
    if(id[i] == id_sc[0] || id[i] == id_sc[1])
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
    if(id[i] == id_sc[0] || id[i] == id_sc[1])
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
  if (id == id_sc[0] || id == id_sc[1])
  {
  
    sc.WritePosEx(id, position, velocity, acceleration);
    while(abs(sc.ReadPos(id)- position) > 0) //System implementation that will delay until servo is at desired position
    delay(1);
  }
  else
  {
    st.WritePosEx(1, position, velocity, acceleration);
    while(abs(st.ReadPos(id)- position) > 0) //System implementation that will delay until servo is at desired position
    delay(1);
  }
}

//Assum
void set_multiple_joints(u8 id[], s16 position[], u16 velocity[], u8 acceleration[])
{
  int sc_index[2] = {-1,-1}, st_index[5] = {-1,-1,-1,-1,-1};
  int sc_curr = 0;
  int st_curr = 0;
  for (int i = 0; i < sizeof(id) / sizeof(int); i++)
  {

    if(id[i] == id_sc[0] || id[i] == id_sc[1])
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
  if(prev_id == id_sc[0] || prev_id == id_sc[1])
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