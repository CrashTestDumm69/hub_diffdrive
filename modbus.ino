 
HardwareSerial modbus(1);

int arg = 0;
int ind = 0;
char chr;
char cmd;
char argv1[16];
char argv2[16];
long arg1;
long arg2;

uint8_t find_crc(uint8_t * data)
{
  uint8_t crc = 0x00;
  uint8_t inbyte, mix;
  for(int i = 0;i < 9;i++)
  { 
    inbyte = data[i];
    for(int j = 0;j < 8;j++)
    {
      mix = (crc ^ inbyte) & 0x01;
      crc = crc >> 1;
      if(mix)
          crc = crc ^ 0x8C;
      inbyte = inbyte >> 1;
      } 
  }
  return crc;
}


int mapper(int val, int inmin, int inmax, int outmin, int outmax)
{
  return ((val - inmin) * (outmax - outmin) / (inmax - inmin)) + outmin;
}

void read_motor_values()
{
  int flag = 0,vel;
  uint8_t crc;
  uint8_t response[10] = {0};
  uint8_t data1[10] = {0x01, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};
  uint8_t data2[10] = {0x02, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF1};
  modbus.write(data1,10);
  modbus.readBytes(response,10);
  crc = find_crc(response);
  if(crc != response[9])
    return;
  if((response[4] & 0x80) != 0x00)
  {
    flag = 1;
    response[4] = ~response[4];
    response[5] = ~response[5];
    response[5] = response[5] | 0x01;
  }

  vel = unsigned(response[4])*255 + unsigned(response[5]);
  if(flag == 1 and vel != 0)
    vel = -1 * vel;
  Serial0.print(vel);
  Serial0.print(" ");
  Serial0.print(360 - mapper(unsigned(response[7]),0,255,0,360));
  Serial0.print(" ");
  
  flag = 0;
  modbus.write(data2,10);
  modbus.readBytes(response,10);
  crc = find_crc(response);
  if(crc != response[9])
    return;
  if((response[4] & 0x80) != 0x00)
  {
    flag = 1;
    response[4] = ~response[4];
    response[5] = ~response[5];
    response[5] = (response[5] | 0x01);
  }
  vel = unsigned(response[4])*255 + unsigned(response[5]);
  if(flag == 0 and vel != 0)
    vel = -1 * vel;
  Serial0.print(vel);
  Serial0.print(" ");
  Serial0.println(mapper(unsigned(response[7]),0,255,0,360));
}

void set_motor_values(int val_l, int val_r)
{
  int val_1 = int(val_l);
  int val_2 = int(val_r);
  val_2 = -1 * val_2;
  uint8_t data1[10],data2[10];
  uint8_t response[10] = {0};
  uint8_t hex[10] = {0};
  data1[0] = 0x01;
  data1[1] = 0x64;
  data1[2] = 0x00;
  data2[0] = 0x02;
  data2[1] = 0x64;
  data2[2] = 0x00;
  for(int i = 4;i < 10;i++)
  {
    data1[i] = 0x00;
    data2[i] = 0x00;
  }
  memcpy((char*)hex,(char*)&val_1,sizeof(int));
  data1[3] = hex[0];
  data1[2] = hex[1];
  memcpy((char*)hex,(char*)&val_2,sizeof(int));
  data2[3] = hex[0];
  data2[2] = hex[1];
  data1[9] = find_crc(data1);
  data2[9] = find_crc(data2);
  modbus.write(data1,10);
  modbus.readBytes(response,10);
  modbus.write(data2,10);
  modbus.readBytes(response,10);
}

/*void set_zero()
{
  uint8_t data[10] = {0x01, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03};
  uint8_t response[10] = {0};
  modbus.write(data,10);
  
  delay(100);
  
  data[0] = 0x02;
  modbus.write(data,10);

  delay(5000);
  
  data[1] = 0x64;
  data[9] = 0xA5;
  modbus.write(data,10);
  modbus.readBytes(response,10);
  
  delay(100);
  
  data[0] = 0x01;
  data[9] = 0x50;
  modbus.write(data,10);
  modbus.readBytes(response,10);
  
  delay(5000);

  data[1] = 0xA0;
  data[9] = 0x02;
  modbus.write(data,10);

  delay(100);
  
  data[0] = 0x02;
  modbus.write(data,10);

  Serial0.println("OK");
}*/

void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  ind = 0;
}

void runCommand() {
  int i = 0;
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  
  case 'r':
    read_motor_values();
    break;
  case 's':
    set_motor_values(arg1,arg2); 
    break;
  /*case 'n':
    set_zero();
    break;*/
  }
}

void setup() {

  // put your setup code here, to run once:
  Serial0.begin(115200);
  modbus.begin(115200, SERIAL_8N1, 17, 18);
  set_motor_values(0,0);
  read_motor_values();
  resetCommand();
}

 

void loop() 
{  
  while (Serial0.available() > 0) 
  {
    // Read the next character
    chr = Serial0.read();
    // Terminate a command with a CR
    if (chr == 13) 
    {
      if (arg == 1) argv1[ind] = NULL;
      else if (arg == 2) argv2[ind] = NULL;
      /*Serial0.println(cmd);
      Serial0.println(atoi(argv1));
      Serial0.println(atoi(argv2));*/
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') 
    {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)
      {
        argv1[ind] = NULL;
        arg = 2;
        ind = 0;
      }
      continue;
    }
    else
    {
      if (arg == 0)
      {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1)
      {
        // Subsequent arguments can be more than one character
        argv1[ind] = chr;
        ind++;
      }
      else if (arg == 2)
      {
        argv2[ind] = chr;
        ind++;
      }
    }
  }
}
