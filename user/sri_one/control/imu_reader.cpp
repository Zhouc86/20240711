#include "imu_reader.h"

ImuReader::ImuReader() {

  angle_degree.reserve(3);
  magnetometer.reserve(3);
  acceleration.reserve(3);
  angularVelocity.reserve(3);
  mag_enabled = false;

  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
 	if (fd == -1) {
	    std::cout << "open_port: Unable to open /dev/ttyUSB0 - " << std::endl;
    } 
  else 
    	fcntl(fd, F_SETFL, FNDELAY);

  struct termios options;

  if  (tcgetattr(fd, &options) < 0) {
  }
  options.c_cflag  |=  CLOCAL;
  options.c_cflag  |=  CREAD;
  options.c_cflag &= ~CRTSCTS;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~PARENB;
  options.c_iflag &= ~INPCK; 

  options.c_iflag &= ~(INLCR|ICRNL);

  options.c_cflag &= ~CSTOPB;
  
  options.c_oflag &= ~OPOST;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  cfsetispeed(&options, B921600); 
  cfsetospeed(&options, B921600);

  tcflush(fd, TCIFLUSH);
  if(tcsetattr(fd, TCSANOW, &options) < 0) {
  }

  open_flag = true;

  pub_buff.resize(9);
  clearpos = false;
  clearyaw.store(false);
}

ImuReader::~ImuReader(){

  clearpos = false;
  clearyaw.store(false);
  pub_buff.clear();
  close(fd);
}

void ImuReader::run() {

   
   unsigned char Source[256];
  if (open_flag) {

      memset(Source,0,256);
      size_t len = read(fd, Source, 256);

      if( (len == -1) && errno == EAGAIN) {
      }
      else if (len) {

          std::vector<unsigned char> buff(Source,Source + len);
          for (size_t i = 0; i < len; ++i) {
            handleSerialData(buff[i]);
          }
          buff.clear();
        } 
    }
 // }
}

bool ImuReader::checkSum(const std::vector<uint8_t> &list_data,
                         uint16_t check_data) {
  uint16_t sum = 0x0000;
  for (auto pos : list_data) {
    sum += pos;
  }
  return sum == check_data;
}

std::vector<int16_t> ImuReader::hex_to_int16(
    const std::vector<uint8_t> &raw_data) {

  if (raw_data.size() % 2 != 0) {
    throw std::runtime_error("raw_data length must be a multiple of 2.");
  }

  std::vector<int16_t> trans_data;
  std::vector<uint8_t> raw_data_input(raw_data);

  for (size_t i = 0; i < raw_data_input.size(); i += 2) {

    int16_t int_data = (static_cast<uint16_t>(raw_data_input[i + 1]) << 8) |
                       static_cast<uint16_t>(raw_data_input[i]);
    if (int_data >= 0x8000) int_data -= 0xffff;

    trans_data.push_back(int_data);
  }

  return trans_data;
}

void ImuReader::printSensorData() {
  // std::cout << "\n加速度(m/s²)：\n";
  acc_x = acceleration[0] * 0.001 * 9.8;
  acc_y = acceleration[1] * 0.001 * 9.8;
  acc_z = acceleration[2] * 0.001 * 9.8;
//   std::cout << "    x轴：" << acc_x << "\n";
//   std::cout << "    y轴：" << acc_y << "\n";
//   std::cout << "    z轴：" << acc_z << "\n\n";

  pub_buff[0] = acc_x;
  pub_buff[1] = acc_y;
  pub_buff[2] = acc_z;

  // std::cout << "角速度(rad/s)：\n";
  angV_x = angularVelocity[0] * 0.015625 * Deg2Rad;  // 1/64 = 0.015625
  angV_y = angularVelocity[1] * 0.015625 * Deg2Rad;
  angV_z = angularVelocity[2] * 0.015625 * Deg2Rad;
  // std::cout << "    x轴：" << angV_x << "\n";
  // std::cout << "    y轴：" << angV_y << "\n";
  // std::cout << "    z轴：" << angV_z << "\n\n";

  pub_buff[3] = angV_x;
  pub_buff[4] = angV_y;
  pub_buff[5] = angV_z;

  // std::cout << "欧拉角(°)：\n";
  angle_x = angle_degree[0] * 0.01;
  angle_y = angle_degree[1] * 0.01;
  angle_z = angle_degree[2] * 0.01;
  // std::cout << "    x轴：" << angle_x << "\n";
  // std::cout << "    y轴：" << angle_y << "\n";
  // std::cout << "    z轴：" << angle_z << "\n\n";

  pub_buff[6] = angle_x;
  pub_buff[7] = angle_y;
  pub_buff[8] = angle_z;
}

void ImuReader::CheckZero(int type)
{
  unsigned char buff[8];
  unsigned char zerocheck[] = {0xFF,0x1C,0x00,0x00,0x00,0x00,0xC5,0xD6};
  unsigned char zeroYaw[] = {0xFF,0x1E,0x00,0x00,0x00,0x00,0xBC,0x16};
  unsigned char imureset[] = {0xFF,0x1A,0x00,0x00,0x00,0x00,0x4D,0xD6};

  if(!type)
    memcpy(buff,zerocheck,8);
  else if(type == 1)
    memcpy(buff,zeroYaw,8);
  else
    memcpy(buff,imureset,8);

  int ret = write(fd,buff,8);
}

void ImuReader::handleSerialData(uint8_t raw_data) {
  buff.push_back(raw_data);
  key++;

  
  if (buff.size() < 2) return;

  if (buff[0] != 0x80 || buff[1] != 0x01) {
    buff.clear();
    buff.push_back(raw_data);
    key = 1;
    return;
  }
     
  if (!mag_enabled) 
  {
    
    if (key < 30) return;  

    std::vector<uint8_t> data_buff(buff.begin(), buff.end());  // 复制缓冲区

    uint16_t check = (static_cast<uint16_t>(buff[27]) << 8) |
                     static_cast<uint16_t>(buff[26]);
    
    if (pub_flag[0] &&
        checkSum(
            std::vector<uint8_t>(data_buff.begin() + 2, data_buff.begin() + 26),
            check) &&
        buff[28] == 0x0d && buff[29] == 0x0a) {

        if(((data_buff.at(4) & 0x00ff) == 0x03) && !clearpos)
        {
            CheckZero(0);
            clearpos = true;
            printf("clear pos\r\n");
        }
        else if(((data_buff.at(4) & 0x00ff) == 0x0b) && !clearyaw && clearpos)
        {
            CheckZero(1);
            clearyaw.store(true);
            printf("clear yaw\r\n");
        }
        else if(((data_buff.at(4) & 0x00ff) == 0x0b) && !clearyaw && !clearpos)
        {
            CheckZero(2);
            printf("imu reset\r\n");
        }
            

      auto data = hex_to_int16(
          std::vector<uint8_t>(data_buff.begin() + 6, data_buff.begin() + 26));
      angularVelocity = std::vector<int16_t>(data.begin(), data.begin() + 3);
      acceleration = std::vector<int16_t>(data.begin() + 3, data.begin() + 6);
      angle_degree = std::vector<int16_t>(data.begin() + 6, data.begin() + 9);
      pub_flag[0] = false;
    } else {

      buff.clear();
      key = 0;
      pub_flag[0] = true;
      return;
    }
    printSensorData();

  } else  
  {
    if (key < 38) return;  
    
    std::vector<uint8_t> data_buff(buff.begin(), buff.end()); 

    uint16_t check = (static_cast<uint16_t>(buff[35]) << 8) |
                     static_cast<uint16_t>(buff[34]);

    if (pub_flag[0] && buff[36] == 0x0d && buff[37] == 0x0a) {

      auto data = hex_to_int16(
          std::vector<uint8_t>(data_buff.begin() + 6, data_buff.begin() + 34));
      angularVelocity = std::vector<int16_t>(data.begin(), data.begin() + 3);
      acceleration = std::vector<int16_t>(data.begin() + 3, data.begin() + 6);
      angle_degree = std::vector<int16_t>(data.begin() + 6, data.begin() + 9);
      magnetometer = std::vector<int16_t>(data.begin() + 9, data.begin() + 12);
      pub_flag[0] = false;
    } else {

      buff.clear();
      key = 0;
      pub_flag[0] = true;
      return;
    }
    printSensorData();
  }

  buff.clear();
  key = 0;
  pub_flag[0] = true;
}
