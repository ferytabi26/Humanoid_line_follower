#define ID_NUM_2  2
#define ID_NUM_3  3
#define ID_NUM_4  4
#define ID_NUM_5  5
#define ID_NUM_6  6
#define ID_NUM_7  7
#define ID_NUM_8  8
#define ID_NUM_9  9
#define ID_NUM_10  10
#define ID_NUM_11  11
#define ID_NUM_12  12
#define ID_NUM_13  13

#define ID_NUM_15  15
#define ID_NUM_16  16
#define ID_NUM_17  17
#define ID_NUM_18  18
#define ID_NUM_19  19
#define ID_NUM_20  20
#define ID_NUM_21  21
#define ID_NUM_22  22

/* Control table defines */
#define P_GOAL_POSITION    30
#define P_GOAL_SPEED    32

#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

//definisi gyro
#define GY25_ROLL 'R'
#define GY25_PITCH 'P'
#define GY25_YAW 'Y'
///////////////////////////GYRO///////////////////////////////////
class GY25 {
  private:
    float roll, pitch, yaw;
    unsigned char buffer[8];
    byte counter;
  public:
    GY25(long int baudrate) {
      Serial2.begin(baudrate);
    }
    void init() {
      delay(3000);
      Serial2.write(0XA5);
      Serial2.write(0X54);
      delay(3000);
      Serial2.write(0XA5);
      Serial2.write(0X51);
    }
    
    void update() {
      Serial2.write(0XA5);
      Serial2.write(0X51);
      while(Serial2.available()) {   
        buffer[counter] = (unsigned char) Serial2.read();
        if(counter == 0 && buffer[0] != 0xAA) return;              
        if(counter == 8) {                
          // Perform check if data package is correct     
          if(buffer[0] == 0xAA && buffer[7] == 0x55) {         
            yaw = (int16) (buffer[1]<<8|buffer[2]) / 100.00;   
            pitch = (int16) (buffer[3]<<8|buffer[4]) / 100.00;
            roll = (int16) (buffer[5]<<8|buffer[6]) / 100.00;
          }
          counter = 0;
        }
        counter++;
      }
    }
    
    int getAngle(char angle_type) {
      if(angle_type == GY25_ROLL) {
        return roll;
      } else if(angle_type == GY25_PITCH) {
        return pitch;
      } else if(angle_type == GY25_YAW) {
        return yaw;
      } else {
        return 0;
      }
    }
};
//////////////////////////////////////////////////////////////
int de2pos_AX12(float _angle){
  int ret;
  float D2R = 3.14f/180.0f;
  _angle = _angle * D2R;
  ret = _angle * 197;
  ret = ret + 512;
  
  return ret;
}

int de2pos_MX28(float _angle){
  int ret;
  float D2R = 3.14f/180.0f;
  _angle = _angle * D2R;
  ret = _angle * 788;
  ret = ret + 2048;
  
  return ret;
}

Dynamixel Dxl(DXL_BUS_SERIAL1);
GY25 gyro(115200);

word keadaan_awal[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(0),60,
  ID_NUM_4,de2pos_MX28(0),60,
  ID_NUM_5,de2pos_MX28(0),60,
  ID_NUM_6,de2pos_MX28(0),60,
  ID_NUM_7,de2pos_MX28(0),60,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(0),60,
  ID_NUM_10,de2pos_MX28(0),60,
  ID_NUM_11,de2pos_MX28(0),60,
  ID_NUM_12,de2pos_MX28(0),60,
  ID_NUM_13,de2pos_MX28(0),60,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(0),60,
  ID_NUM_17,de2pos_AX12(0),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(0),60,
  ID_NUM_20,de2pos_AX12(0),60,};

word LURUS_1[60]=
{   
  ID_NUM_3,de2pos_MX28(8),40,
  ID_NUM_4,de2pos_MX28(47),150,
  ID_NUM_5,de2pos_MX28(83),150,
  ID_NUM_6,de2pos_MX28(40),150,
  ID_NUM_7,de2pos_MX28(4),40,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(-6),40,
  ID_NUM_10,de2pos_MX28(-32),150,
  ID_NUM_11,de2pos_MX28(-55),150,
  ID_NUM_12,de2pos_MX28(-30),150,
  ID_NUM_13,de2pos_MX28(-3),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word LURUS_2[60]=
{ 
  ID_NUM_3,de2pos_MX28(8),40,
  ID_NUM_4,de2pos_MX28(50),150,
  ID_NUM_5,de2pos_MX28(63),150,
  ID_NUM_6,de2pos_MX28(38),150,
  ID_NUM_7,de2pos_MX28(4),40,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(-6),40,
  ID_NUM_10,de2pos_MX28(-32),150,
  ID_NUM_11,de2pos_MX28(-55),150,
  ID_NUM_12,de2pos_MX28(-30),150,
  ID_NUM_13,de2pos_MX28(-3),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word LURUS_3[60]=
{ 
  ID_NUM_3,de2pos_MX28(6),40,
  ID_NUM_4,de2pos_MX28(42),150,
  ID_NUM_5,de2pos_MX28(55),150,
  ID_NUM_6,de2pos_MX28(37),150,
  ID_NUM_7,de2pos_MX28(-1),40,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(-8),40,
  ID_NUM_10,de2pos_MX28(-32),150,
  ID_NUM_11,de2pos_MX28(-55),150,
  ID_NUM_12,de2pos_MX28(-30),150,
  ID_NUM_13,de2pos_MX28(-5),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word LURUS_4[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(6),40,
  ID_NUM_4,de2pos_MX28(32),150,
  ID_NUM_5,de2pos_MX28(55),150,
  ID_NUM_6,de2pos_MX28(30),150,
  ID_NUM_7,de2pos_MX28(3),40,
  ID_NUM_9,de2pos_MX28(-8),40,
  ID_NUM_10,de2pos_MX28(-47),150,
  ID_NUM_11,de2pos_MX28(-83),150,
  ID_NUM_12,de2pos_MX28(-40),150,
  ID_NUM_13,de2pos_MX28(-10),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word LURUS_5[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(6),40,
  ID_NUM_4,de2pos_MX28(32),150,
  ID_NUM_5,de2pos_MX28(55),150,
  ID_NUM_6,de2pos_MX28(30),150,
  ID_NUM_7,de2pos_MX28(3),40,
  ID_NUM_9,de2pos_MX28(-8),40,
  ID_NUM_10,de2pos_MX28(-50),150,
  ID_NUM_11,de2pos_MX28(-63),150,
  ID_NUM_12,de2pos_MX28(-38),150,
  ID_NUM_13,de2pos_MX28(-10),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word LURUS_6[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(8),40,
  ID_NUM_4,de2pos_MX28(32),150,
  ID_NUM_5,de2pos_MX28(55),150,
  ID_NUM_6,de2pos_MX28(30),150,
  ID_NUM_7,de2pos_MX28(5),40,
  ID_NUM_9,de2pos_MX28(-6),40,
  ID_NUM_10,de2pos_MX28(-42),150,
  ID_NUM_11,de2pos_MX28(-55),150,
  ID_NUM_12,de2pos_MX28(-37),150,
  ID_NUM_13,de2pos_MX28(-8),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
///////////////////////////////////////////////////////////////////////
word KIRI_1[60]=
{   
  ID_NUM_2,de2pos_AX12(-10),60,
  ID_NUM_3,de2pos_MX28(8),40,
  ID_NUM_4,de2pos_MX28(47),150,
  ID_NUM_5,de2pos_MX28(83),150,
  ID_NUM_6,de2pos_MX28(40),150,
  ID_NUM_7,de2pos_MX28(4),40,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(-6),40,
  ID_NUM_10,de2pos_MX28(-32),150,
  ID_NUM_11,de2pos_MX28(-55),150,
  ID_NUM_12,de2pos_MX28(-30),150,
  ID_NUM_13,de2pos_MX28(-3),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word KIRI_2[60]=
{ 
  ID_NUM_2,de2pos_AX12(-10),60,
  ID_NUM_3,de2pos_MX28(8),40,
  ID_NUM_4,de2pos_MX28(50),150,
  ID_NUM_5,de2pos_MX28(63),150,
  ID_NUM_6,de2pos_MX28(38),150,
  ID_NUM_7,de2pos_MX28(4),40,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(-6),40,
  ID_NUM_10,de2pos_MX28(-32),150,
  ID_NUM_11,de2pos_MX28(-55),150,
  ID_NUM_12,de2pos_MX28(-30),150,
  ID_NUM_13,de2pos_MX28(-3),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word KIRI_3[60]=
{ 
  ID_NUM_2,de2pos_AX12(-10),60,
  ID_NUM_3,de2pos_MX28(6),40,
  ID_NUM_4,de2pos_MX28(42),150,
  ID_NUM_5,de2pos_MX28(55),150,
  ID_NUM_6,de2pos_MX28(37),150,
  ID_NUM_7,de2pos_MX28(-1),40,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(-8),40,
  ID_NUM_10,de2pos_MX28(-32),150,
  ID_NUM_11,de2pos_MX28(-55),150,
  ID_NUM_12,de2pos_MX28(-30),150,
  ID_NUM_13,de2pos_MX28(-5),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word KIRI_4[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(6),40,
  ID_NUM_4,de2pos_MX28(32),150,
  ID_NUM_5,de2pos_MX28(55),150,
  ID_NUM_6,de2pos_MX28(30),150,
  ID_NUM_7,de2pos_MX28(3),40,
  ID_NUM_8,de2pos_AX12(-10),60,
  ID_NUM_9,de2pos_MX28(-8),40,
  ID_NUM_10,de2pos_MX28(-47),150,
  ID_NUM_11,de2pos_MX28(-83),150,
  ID_NUM_12,de2pos_MX28(-40),150,
  ID_NUM_13,de2pos_MX28(-10),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word KIRI_5[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(6),40,
  ID_NUM_4,de2pos_MX28(32),150,
  ID_NUM_5,de2pos_MX28(55),150,
  ID_NUM_6,de2pos_MX28(30),150,
  ID_NUM_7,de2pos_MX28(3),40,
  ID_NUM_8,de2pos_AX12(-10),60,
  ID_NUM_9,de2pos_MX28(-8),40,
  ID_NUM_10,de2pos_MX28(-50),150,
  ID_NUM_11,de2pos_MX28(-63),150,
  ID_NUM_12,de2pos_MX28(-38),150,
  ID_NUM_13,de2pos_MX28(-10),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word KIRI_6[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(8),40,
  ID_NUM_4,de2pos_MX28(32),150,
  ID_NUM_5,de2pos_MX28(55),150,
  ID_NUM_6,de2pos_MX28(30),150,
  ID_NUM_7,de2pos_MX28(5),40,
  ID_NUM_8,de2pos_AX12(-10),60,
  ID_NUM_9,de2pos_MX28(-6),40,
  ID_NUM_10,de2pos_MX28(-42),150,
  ID_NUM_11,de2pos_MX28(-55),150,
  ID_NUM_12,de2pos_MX28(-37),150,
  ID_NUM_13,de2pos_MX28(-8),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};

///////////////////////////////////////////////////////////////////
word KANAN_1[60]=
{   
  ID_NUM_2,de2pos_AX12(10),60,
  ID_NUM_3,de2pos_MX28(8),40,
  ID_NUM_4,de2pos_MX28(47),150,
  ID_NUM_5,de2pos_MX28(83),150,
  ID_NUM_6,de2pos_MX28(40),150,
  ID_NUM_7,de2pos_MX28(4),40,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(-6),40,
  ID_NUM_10,de2pos_MX28(-32),150,
  ID_NUM_11,de2pos_MX28(-55),150,
  ID_NUM_12,de2pos_MX28(-30),150,
  ID_NUM_13,de2pos_MX28(-3),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word KANAN_2[60]=
{ 
  ID_NUM_2,de2pos_AX12(10),60,
  ID_NUM_3,de2pos_MX28(8),40,
  ID_NUM_4,de2pos_MX28(50),150,
  ID_NUM_5,de2pos_MX28(63),150,
  ID_NUM_6,de2pos_MX28(38),150,
  ID_NUM_7,de2pos_MX28(4),40,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(-6),40,
  ID_NUM_10,de2pos_MX28(-32),150,
  ID_NUM_11,de2pos_MX28(-55),150,
  ID_NUM_12,de2pos_MX28(-30),150,
  ID_NUM_13,de2pos_MX28(-3),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word KANAN_3[60]=
{ 
  ID_NUM_2,de2pos_AX12(10),60,
  ID_NUM_3,de2pos_MX28(6),40,
  ID_NUM_4,de2pos_MX28(42),150,
  ID_NUM_5,de2pos_MX28(55),150,
  ID_NUM_6,de2pos_MX28(37),150,
  ID_NUM_7,de2pos_MX28(-1),40,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(-8),40,
  ID_NUM_10,de2pos_MX28(-32),150,
  ID_NUM_11,de2pos_MX28(-55),150,
  ID_NUM_12,de2pos_MX28(-30),150,
  ID_NUM_13,de2pos_MX28(-5),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word KANAN_4[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(6),40,
  ID_NUM_4,de2pos_MX28(32),150,
  ID_NUM_5,de2pos_MX28(55),150,
  ID_NUM_6,de2pos_MX28(30),150,
  ID_NUM_7,de2pos_MX28(3),40,
  ID_NUM_8,de2pos_AX12(10),60,
  ID_NUM_9,de2pos_MX28(-8),40,
  ID_NUM_10,de2pos_MX28(-47),150,
  ID_NUM_11,de2pos_MX28(-83),150,
  ID_NUM_12,de2pos_MX28(-40),150,
  ID_NUM_13,de2pos_MX28(-10),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word KANAN_5[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(6),40,
  ID_NUM_4,de2pos_MX28(32),150,
  ID_NUM_5,de2pos_MX28(55),150,
  ID_NUM_6,de2pos_MX28(30),150,
  ID_NUM_7,de2pos_MX28(3),40,
  ID_NUM_8,de2pos_AX12(10),60,
  ID_NUM_9,de2pos_MX28(-8),40,
  ID_NUM_10,de2pos_MX28(-50),150,
  ID_NUM_11,de2pos_MX28(-63),150,
  ID_NUM_12,de2pos_MX28(-38),150,
  ID_NUM_13,de2pos_MX28(-10),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,};
word KANAN_6[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(8),40,
  ID_NUM_4,de2pos_MX28(32),150,
  ID_NUM_5,de2pos_MX28(55),150,
  ID_NUM_6,de2pos_MX28(30),150,
  ID_NUM_7,de2pos_MX28(5),40,
  ID_NUM_8,de2pos_AX12(10),60,
  ID_NUM_9,de2pos_MX28(-6),40,
  ID_NUM_10,de2pos_MX28(-42),150,
  ID_NUM_11,de2pos_MX28(-55),150,
  ID_NUM_12,de2pos_MX28(-37),150,
  ID_NUM_13,de2pos_MX28(-8),40,
  ID_NUM_15,de2pos_AX12(0),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(50),60,
  ID_NUM_18,de2pos_AX12(0),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-50),60,
};
/////////////////////////////////////////////
word bangun_Terlentang_1[60]=
{   
  ID_NUM_2,500,50,  
  ID_NUM_3,2067,50,
  ID_NUM_4,2947,50,
  ID_NUM_5,3181,50,
  ID_NUM_6,2466,50,
  ID_NUM_7,2049,50,
  ID_NUM_8,500,50,
  ID_NUM_9,1994,50,
  ID_NUM_10,1160,50,
  ID_NUM_11,1014,50,
  ID_NUM_12,1900,50,
  ID_NUM_13,2097,50,
  ID_NUM_15,de2pos_AX12(-40),50,
  ID_NUM_16,de2pos_AX12(14),50,
  ID_NUM_17,de2pos_AX12(49),50,
  ID_NUM_18,de2pos_AX12(40),50,
  ID_NUM_19,de2pos_AX12(-14),50,
  ID_NUM_20,de2pos_AX12(-49),50,
  ID_NUM_21,511,50,
  ID_NUM_22,607,50,}; 
word bangun_Terlentang_2[60]=
{ 
  ID_NUM_2,500,60,
  ID_NUM_3,2069,60,
  ID_NUM_4,2839,60,
  ID_NUM_5,3179,60,
  ID_NUM_6,1735,60,
  ID_NUM_7,2058,60,
  ID_NUM_8,500,60,
  ID_NUM_9,2005,60,
  ID_NUM_10,1240,60,
  ID_NUM_11,922,60,
  ID_NUM_12,2365,60,
  ID_NUM_13,2072,60,
  ID_NUM_15,de2pos_AX12(-80),50,
  ID_NUM_16,de2pos_AX12(14),50,
  ID_NUM_17,de2pos_AX12(80),50,
  ID_NUM_18,de2pos_AX12(80),50,
  ID_NUM_19,de2pos_AX12(-14),50,
  ID_NUM_20,de2pos_AX12(-80),50,
  ID_NUM_21,511,60,
  ID_NUM_22,607,60,};
word bangun_Terlentang_3[60]=
{ 
  ID_NUM_2,500,60,
  ID_NUM_3,2059,60,
  ID_NUM_4,2836,60,
  ID_NUM_5,3162,60,
  ID_NUM_6,1697,60,
  ID_NUM_7,2039,60,
  ID_NUM_8,500,60,
  ID_NUM_9,1996,60,
  ID_NUM_10,1232,60,
  ID_NUM_11,922,60,
  ID_NUM_12,2381,60,
  ID_NUM_13,2087,60,
  ID_NUM_15,de2pos_AX12(-120),50,
  ID_NUM_16,de2pos_AX12(14),50,
  ID_NUM_17,de2pos_AX12(120),50,
  ID_NUM_18,de2pos_AX12(120),50,
  ID_NUM_19,de2pos_AX12(-14),50,
  ID_NUM_20,de2pos_AX12(-120),50,
  ID_NUM_21,511,60,
  ID_NUM_22,607,60,};
word bangun_Terlentang_4[60]=
{ 
  ID_NUM_2,500,60,
  ID_NUM_3,2077,60,
  ID_NUM_4,2032,60,
  ID_NUM_5,3093,60,
  ID_NUM_6,2087,60,
  ID_NUM_7,2019,60,
  ID_NUM_8,500,60,
  ID_NUM_9,2012,60,
  ID_NUM_10,2079,60,
  ID_NUM_11,1076,60,
  ID_NUM_12,2059,60,
  ID_NUM_13,1978,60,
  ID_NUM_15,0,60,
  ID_NUM_16,519,60,
  ID_NUM_17,de2pos_AX12(62),60,
  ID_NUM_18,1023,60,
  ID_NUM_19,489,60,
  ID_NUM_20,de2pos_AX12(-62),60,
  ID_NUM_21,511,60,
  ID_NUM_22,607,60,};

word bangun_Terlentang_5[60]=
{ 
  ID_NUM_2,500,60,
  ID_NUM_3,2057,60,
  ID_NUM_4,1649,60,
  ID_NUM_5,3264,60,
  ID_NUM_6,2945,60,
  ID_NUM_7,2046,60,
  ID_NUM_8,500,60,
  ID_NUM_9,2046,60,
  ID_NUM_10,2479,60,
  ID_NUM_11,857,60,
  ID_NUM_12,1162,60,
  ID_NUM_13,2034,60,
  ID_NUM_15,219,60,
  ID_NUM_16,531,60,
  ID_NUM_17,de2pos_AX12(27),60,
  ID_NUM_18,801,60,
  ID_NUM_19,506,60,
  ID_NUM_20,de2pos_AX12(-27),60,
  ID_NUM_21,511,60,
  ID_NUM_22,606,60,};
word bangun_Terlentang_6[60]=
{ 
  ID_NUM_2,500,80,
  ID_NUM_3,2108,80,
  ID_NUM_4,2147,80,
  ID_NUM_5,3251,80,
  ID_NUM_6,3038,80,
  ID_NUM_7,2063,80,
  ID_NUM_8,500,80,
  ID_NUM_9,2038,80,
  ID_NUM_10,1978,80,
  ID_NUM_11,888,80,
  ID_NUM_12,1083,80,
  ID_NUM_13,1995,80,
  ID_NUM_15,512,80,
  ID_NUM_16,529,80,
  ID_NUM_17,de2pos_AX12(27),80,
  ID_NUM_18,512,80,
  ID_NUM_19,505,80,
  ID_NUM_20,de2pos_AX12(-27),80,
  ID_NUM_21,511,80,
  ID_NUM_22,606,80,};
word bangun_Terlentang_7[60]=
{ 
  ID_NUM_2,512,50,
  ID_NUM_3,2048,50,
  ID_NUM_4,de2pos_MX28(43),49,
  ID_NUM_5,de2pos_MX28(59),42,
  ID_NUM_6,de2pos_MX28(29),62,
  ID_NUM_7,2048,50,
  ID_NUM_8,512,50,
  ID_NUM_9,2048,50,
  ID_NUM_10,de2pos_MX28(-43),49,
  ID_NUM_11,de2pos_MX28(-59),42,
  ID_NUM_12,de2pos_MX28(-29),62,
  ID_NUM_13,2048,50,
  ID_NUM_15,489,50,
  ID_NUM_16,554,50,
  ID_NUM_17,de2pos_AX12(27),50,
  ID_NUM_18,533,50,
  ID_NUM_19,502,50,
  ID_NUM_20,de2pos_AX12(-27),50,
  ID_NUM_21,511,50,
  ID_NUM_22,606,50,};
///////////////////////////////////////////bangun jika tengkurap/////////////////////////
word bangun_Tengkurap_1[60]=
{   
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(2),60,
  ID_NUM_4,de2pos_MX28(-6),60,
  ID_NUM_5,de2pos_MX28(30),60,
  ID_NUM_6,de2pos_MX28(19),60,
  ID_NUM_7,de2pos_MX28(0),60,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(-2),60,
  ID_NUM_10,de2pos_MX28(6),60,
  ID_NUM_11,de2pos_MX28(-30),60,
  ID_NUM_12,de2pos_MX28(-19),60,
  ID_NUM_13,de2pos_MX28(0),60,
  ID_NUM_15,de2pos_AX12(5),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(-5),60,
  ID_NUM_18,de2pos_AX12(-5),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(4),60,
  ID_NUM_21,de2pos_AX12(0),60,
  ID_NUM_22,de2pos_AX12(-25),60,};
word bangun_Tengkurap_2[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(0),60,
  ID_NUM_4,de2pos_MX28(21),60,
  ID_NUM_5,de2pos_MX28(49),60,
  ID_NUM_6,de2pos_MX28(49),60,
  ID_NUM_7,de2pos_MX28(0),60,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(0),60,
  ID_NUM_10,de2pos_MX28(-21),60,
  ID_NUM_11,de2pos_MX28(-49),60,
  ID_NUM_12,de2pos_MX28(-49),60,
  ID_NUM_13,de2pos_MX28(0),60,
  ID_NUM_15,de2pos_AX12(32),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(0),60,
  ID_NUM_18,de2pos_AX12(-32),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(0),60,
  ID_NUM_21,de2pos_AX12(0),60,
  ID_NUM_22,de2pos_AX12(14),60,};
word bangun_Tengkurap_3[60]=
{ 
    ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(0),50,
  ID_NUM_4,de2pos_MX28(105),50,
  ID_NUM_5,de2pos_MX28(108),50,
  ID_NUM_6,de2pos_MX28(47),50,
  ID_NUM_7,de2pos_MX28(0),50,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(0),50,
  ID_NUM_10,de2pos_MX28(-105),50,
  ID_NUM_11,de2pos_MX28(-108),50,
  ID_NUM_12,de2pos_MX28(-47),50,
  ID_NUM_13,de2pos_MX28(0),60,
  ID_NUM_15,de2pos_AX12(32),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(0),60,
  ID_NUM_18,de2pos_AX12(-32),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(0),60,
  ID_NUM_21,de2pos_AX12(0),60,
  ID_NUM_22,de2pos_AX12(14),60,};
word bangun_Tengkurap_4[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(0),50,
  ID_NUM_4,de2pos_MX28(109),50,
  ID_NUM_5,de2pos_MX28(111),50,
  ID_NUM_6,de2pos_MX28(35),50,
  ID_NUM_7,de2pos_MX28(0),50,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(0),50,
  ID_NUM_10,de2pos_MX28(-109),50,
  ID_NUM_11,de2pos_MX28(-111),50,
  ID_NUM_12,de2pos_MX28(-35),50,
  ID_NUM_13,de2pos_MX28(0),50,
  ID_NUM_15,de2pos_AX12(-36),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(148),60,
  ID_NUM_18,de2pos_AX12(36),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-148),60,
  ID_NUM_21,de2pos_AX12(0),60,
  ID_NUM_22,de2pos_AX12(14),60,};
word bangun_Tengkurap_5[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(0),50,
  ID_NUM_4,de2pos_MX28(109),50,
  ID_NUM_5,de2pos_MX28(111),50,
  ID_NUM_6,de2pos_MX28(35),50,
  ID_NUM_7,de2pos_MX28(0),50,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(0),50,
  ID_NUM_10,de2pos_MX28(-109),50,
  ID_NUM_11,de2pos_MX28(-111),50,
  ID_NUM_12,de2pos_MX28(-35),50,
  ID_NUM_13,de2pos_MX28(0),50,
  ID_NUM_15,de2pos_AX12(36),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(148),60,
  ID_NUM_18,de2pos_AX12(-36),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-148),60,
  ID_NUM_21,de2pos_AX12(0),60,
  ID_NUM_22,de2pos_AX12(14),60,};
word bangun_Tengkurap_6[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(0),50,
  ID_NUM_4,de2pos_MX28(109),50,
  ID_NUM_5,de2pos_MX28(105),50,
  ID_NUM_6,de2pos_MX28(35),50,
  ID_NUM_7,de2pos_MX28(0),50,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(0),50,
  ID_NUM_10,de2pos_MX28(-109),50,
  ID_NUM_11,de2pos_MX28(-105),50,
  ID_NUM_12,de2pos_MX28(-35),50,
  ID_NUM_13,de2pos_MX28(0),50,
  ID_NUM_15,de2pos_AX12(36),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(130),60,
  ID_NUM_18,de2pos_AX12(-36),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(-130),60,
  ID_NUM_21,de2pos_AX12(0),60,
  ID_NUM_22,de2pos_AX12(14),60,};
word bangun_Tengkurap_7[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(0),50,
  ID_NUM_4,de2pos_MX28(109),50,
  ID_NUM_5,de2pos_MX28(105),50,
  ID_NUM_6,de2pos_MX28(35),50,
  ID_NUM_7,de2pos_MX28(0),50,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(0),50,
  ID_NUM_10,de2pos_MX28(-109),50,
  ID_NUM_11,de2pos_MX28(-105),50,
  ID_NUM_12,de2pos_MX28(-35),50,
  ID_NUM_13,de2pos_MX28(0),50,
  ID_NUM_15,de2pos_AX12(50),255,
  ID_NUM_16,de2pos_AX12(10),255,
  ID_NUM_17,de2pos_AX12(0),255,
  ID_NUM_18,de2pos_AX12(-50),255,
  ID_NUM_19,de2pos_AX12(-10),255,
  ID_NUM_20,de2pos_AX12(0),255,
  ID_NUM_21,de2pos_AX12(0),255,
  ID_NUM_22,de2pos_AX12(14),255,};
word bangun_Tengkurap_8[60]=
{ 
  ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(0),30,
  ID_NUM_4,de2pos_MX28(85),50,
  ID_NUM_5,de2pos_MX28(105),50,
  ID_NUM_6,de2pos_MX28(35),50,
  ID_NUM_7,de2pos_MX28(0),50,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(0),50,
  ID_NUM_10,de2pos_MX28(-85),50,
  ID_NUM_11,de2pos_MX28(-105),50,
  ID_NUM_12,de2pos_MX28(-35),50,
  ID_NUM_13,de2pos_MX28(0),30,
  ID_NUM_15,de2pos_AX12(50),60,
  ID_NUM_16,de2pos_AX12(10),60,
  ID_NUM_17,de2pos_AX12(0),60,
  ID_NUM_18,de2pos_AX12(-50),60,
  ID_NUM_19,de2pos_AX12(-10),60,
  ID_NUM_20,de2pos_AX12(0),60,
  ID_NUM_21,de2pos_AX12(0),60,
  ID_NUM_22,de2pos_AX12(14),60,};
word bangun_Tengkurap_9[60]=
{ 
    ID_NUM_2,de2pos_AX12(0),60,
  ID_NUM_3,de2pos_MX28(0),30,
  ID_NUM_4,de2pos_MX28(55),25,
  ID_NUM_5,de2pos_MX28(61),45,
  ID_NUM_6,de2pos_MX28(25),15,
  ID_NUM_7,de2pos_MX28(0),50,
  ID_NUM_8,de2pos_AX12(0),60,
  ID_NUM_9,de2pos_MX28(0),50,
  ID_NUM_10,de2pos_MX28(-55),25,
  ID_NUM_11,de2pos_MX28(-61),45,
  ID_NUM_12,de2pos_MX28(-25),15,
  ID_NUM_13,de2pos_MX28(0),30,
  ID_NUM_15,de2pos_AX12(0),50,
  ID_NUM_16,de2pos_AX12(10),50,
  ID_NUM_17,de2pos_AX12(0),50,
  ID_NUM_18,de2pos_AX12(0),50,
  ID_NUM_19,de2pos_AX12(-10),50,
  ID_NUM_20,de2pos_AX12(0),50,
  ID_NUM_21,de2pos_AX12(0),50,
  ID_NUM_22,de2pos_AX12(14),50,};
  
void lurus(){
  Dxl.syncWrite(30,2,LURUS_1,60);
  delay(150);
  Dxl.syncWrite(30,2,LURUS_2,60);
  delay(50);
  Dxl.syncWrite(30,2,LURUS_3,60);
  delay(30);
  Dxl.syncWrite(30,2,LURUS_4,60);
  delay(150);
  Dxl.syncWrite(30,2,LURUS_5,60);
  delay(50);
  Dxl.syncWrite(30,2,LURUS_6,60);
  delay(30);
}
void kiri(){
  Dxl.syncWrite(30,2,KIRI_1,60);
  delay(150);
  Dxl.syncWrite(30,2,KIRI_2,60);
  delay(50);
  Dxl.syncWrite(30,2,KIRI_3,60);
  delay(30);
  Dxl.syncWrite(30,2,KIRI_4,60);
  delay(150);
  Dxl.syncWrite(30,2,KIRI_5,60);
  delay(50);
  Dxl.syncWrite(30,2,KIRI_6,60);
  delay(30);
}
void kanan(){
  Dxl.syncWrite(30,2,KANAN_1,60);
  delay(150);
  Dxl.syncWrite(30,2,KANAN_2,60);
  delay(50);
  Dxl.syncWrite(30,2,KANAN_3,60);
  delay(30);
  Dxl.syncWrite(30,2,KANAN_4,60);
  delay(150);
  Dxl.syncWrite(30,2,KANAN_5,60);
  delay(50);
  Dxl.syncWrite(30,2,KANAN_6,60);
  delay(30);
} 
void terlentang(){
  Dxl.syncWrite(30,2,bangun_Terlentang_1,60);
  delay(2000);
  Dxl.syncWrite(30,2,bangun_Terlentang_2,60);
  delay(2000);
  Dxl.syncWrite(30,2,bangun_Terlentang_3,60);
  delay(2000);
  Dxl.syncWrite(30,2,bangun_Terlentang_4,60);
  delay(2000);
  Dxl.syncWrite(30,2,bangun_Terlentang_5,60);
  delay(3000);
  Dxl.syncWrite(30,2,bangun_Terlentang_6,60);
  delay(3000);
  Dxl.syncWrite(30,2,bangun_Terlentang_7,60);
  delay(3000);
}
void tengkurap(){
  Dxl.syncWrite(30,2,bangun_Tengkurap_1,60);
  delay(2000);
  Dxl.syncWrite(30,2,bangun_Tengkurap_2,60);
  delay(2000);
  Dxl.syncWrite(30,2,bangun_Tengkurap_3,60);
  delay(3000);
  Dxl.syncWrite(30,2,bangun_Tengkurap_4,60);
  delay(3000);
  Dxl.syncWrite(30,2,bangun_Tengkurap_5,60);
  delay(3000);
  Dxl.syncWrite(30,2,bangun_Tengkurap_6,60);
  delay(3000);
  Dxl.syncWrite(30,2,bangun_Tengkurap_7,60);
  delay(4500);
  Dxl.syncWrite(30,2,bangun_Tengkurap_8,60);
  delay(3000);
  Dxl.syncWrite(30,2,bangun_Tengkurap_9,60);
  delay(5000);
}

int roll,pitch,yaw;
boolean ngeroll;
int coba,hitung1,hitung2;
long millisdelay1,millisdelay2;
int posisix,posisiy,nilaiX,posisiz,previous_posisix,previous_posisiy;
char data_sebelumnya;
int jeda[]={150,50,30,150,50,30};
unsigned long lastMillis,millisDelay;
int i;

void setup() {
  // put your setup code here, to run once:
  Serial3.begin(115200);
  Dxl.begin(3);
  Dxl.jointMode(ID_NUM_2);
  Dxl.jointMode(ID_NUM_3);
  Dxl.jointMode(ID_NUM_4);
  Dxl.jointMode(ID_NUM_5);
  Dxl.jointMode(ID_NUM_6);
  Dxl.jointMode(ID_NUM_7);
  Dxl.jointMode(ID_NUM_8);
  Dxl.jointMode(ID_NUM_9);
  Dxl.jointMode(ID_NUM_10);
  Dxl.jointMode(ID_NUM_11);
  Dxl.jointMode(ID_NUM_12);
  Dxl.jointMode(ID_NUM_13);
  Dxl.jointMode(ID_NUM_15);
  Dxl.jointMode(ID_NUM_16);
  Dxl.jointMode(ID_NUM_17);
  Dxl.jointMode(ID_NUM_18);
  Dxl.jointMode(ID_NUM_19);
  Dxl.jointMode(ID_NUM_20);
  Dxl.jointMode(ID_NUM_21);
  Dxl.jointMode(ID_NUM_22);
  
//  delay(1000);
//  Dxl.syncWrite(30,2,keadaan_awal,60);
//  delay(1000);
//  for(int i=0;i<35;i++){
//    lurus();
//  }
}

void loop() {
  if(Serial3.available()>0) {
    char data = Serial3.read();
//    SerialUSB.print("Arduino: Received ");
//    SerialUSB.println(data);	
  
    switch (data){
      case 'a' : posisix+=7;break;
      case 'b' : posisix+=5;break;
      case 'c' : posisix+=3;break;
      case 'd' : posisix+=1;break;
      case 'e' : posisix+=0;break;
      case 'f' : posisix+=0;break;
      case 'g' : posisix-=1;break;
      case 'h' : posisix-=3;break;
      case 'i' : posisix-=5;break;
      case 'j' : posisix-=7;break;
      case 'k' : posisiy-=7;break;
      case 'l' : posisiy-=5;break;
      case 'm' : posisiy-=3;break;
      case 'n' : posisiy-=1;break;
      case 'o' : posisiy-=0;break;
      case 'p' : posisiy+=0;break;
      case 'q' : posisiy+=0;break;
      case 'r' : posisiy+=1;break;
      case 's' : posisiy+=3;break;
      case 't' : posisiy+=5;break;
      case 'u' : posisiy+=7;break;
      case 'x' : nilaiX=0;posisix=0;posisiy=0;break;
      case 'v' : ;posisix=-30;break;
//      case 'z' : kanan();break;
      default : posisix = previous_posisix;
                posisiy = previous_posisiy;
    }
    previous_posisix = posisix;
    previous_posisiy = posisiy;
  }
//  SerialUSB.print(posisix);
  if(posisix<-90){posisix=-90;}
  if(posisix>90){posisix=90;}
//  if(posisiy<0){posisiy=0;}
//  if(posisiy>90){posisiy=90;}
  posisiz = map(posisix,-90,90,-15,15);
  
  Dxl.writeWord(ID_NUM_21, P_GOAL_POSITION, de2pos_AX12(posisix));
 //////////////////////////////////////////////
  nilaiX=-posisiz;
  lastMillis=millis();
  if(i==0){
    Dxl.setPosition(ID_NUM_2,de2pos_AX12(nilaiX),60);
    Dxl.syncWrite(30,2,LURUS_1,60);
  }else if(i==1){
    Dxl.setPosition(ID_NUM_2,de2pos_AX12(nilaiX),60);
    Dxl.syncWrite(30,2,LURUS_2,60);
  }else if(i==2){
    Dxl.setPosition(ID_NUM_2,de2pos_AX12(nilaiX),60);
    Dxl.syncWrite(30,2,LURUS_3,60);
  }else if(i==3){
    Dxl.setPosition(ID_NUM_8,de2pos_AX12(nilaiX),60);
    Dxl.syncWrite(30,2,LURUS_4,60);
  }else if(i==4){
    Dxl.setPosition(ID_NUM_8,de2pos_AX12(nilaiX),60);
    Dxl.syncWrite(30,2,LURUS_5,60);
  }else if(i==5){
    Dxl.setPosition(ID_NUM_8,de2pos_AX12(nilaiX),60);
    Dxl.syncWrite(30,2,LURUS_6,60);
  }
  if((lastMillis-millisDelay)>=jeda[i]){
  i++;
  if(i>5)i=0;
  millisDelay=lastMillis;  
  }
  
  
  /////////////////////////////////////Keseimbangan/////////////////////////////////
  gyro.update();
  roll = gyro.getAngle(GY25_ROLL);
  pitch = gyro.getAngle(GY25_PITCH);
  yaw = gyro.getAngle(GY25_YAW);  
//  SerialUSB.print(gyro.getAngle(GY25_ROLL));
//  SerialUSB.print("  ");
//  SerialUSB.print(gyro.getAngle(GY25_PITCH));
//  SerialUSB.print("  ");
//  SerialUSB.println(gyro.getAngle(GY25_YAW));
//  delay(100);

  if(roll>50 && roll<114) {coba = 1;}else{coba = 0;hitung1=0;}
  if((millis()-millisdelay1)>=500){
    millisdelay1=millis();
    if (coba==1){
      hitung1++;
      if(hitung1>1){
        hitung1=0;
        terlentang();
      }  
    }   
  }
  
  if(roll<-50 && roll>-114){coba = 1;}else{coba = 0;hitung2=0;}
  if((millis()-millisdelay2)>=500){
    millisdelay2=millis();
    if (coba==1){
      hitung2++;
      if(hitung2>1){
        hitung2=0;
        tengkurap();
      }
    }
  } 
}
//void badan(int posisix){
//  if(posisix>15){
//    kiri();
//  }
//  if((posisix>-15)&&(posisix<15)){
//    lurus();
//  }
//  if(posisix<-15){
//    kanan();
//  }
//}
//void kepala(int posisix){
//  Dxl.writeWord(ID_NUM_21, P_GOAL_POSITION, de2pos_AX12(posisix));
////  delay(15);
//  Dxl.writeWord(ID_NUM_22, P_GOAL_POSITION, de2pos_AX12(posisiy));
////  delay(15);
//}
