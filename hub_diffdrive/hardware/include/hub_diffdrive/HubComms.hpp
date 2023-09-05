#ifndef HUB_DIFFDRIVE__HUB_COMMS_HPP_
#define HUB_DIFFDRIVE__HUB_COMMS_HPP_

#ifdef _WIN64
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
	switch (baud_rate) 
	{
		case 1200: return LibSerial::BaudRate::BAUD_1200;
		case 1800: return LibSerial::BaudRate::BAUD_1800;
		case 2400: return LibSerial::BaudRate::BAUD_2400;
		case 4800: return LibSerial::BaudRate::BAUD_4800;
		case 9600: return LibSerial::BaudRate::BAUD_9600;
		case 19200: return LibSerial::BaudRate::BAUD_19200;
		case 38400: return LibSerial::BaudRate::BAUD_38400;
		case 57600: return LibSerial::BaudRate::BAUD_57600;
		case 115200: return LibSerial::BaudRate::BAUD_115200;
		case 230400: return LibSerial::BaudRate::BAUD_230400;
	default:
		std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 115200" << std::endl;
		return LibSerial::BaudRate::BAUD_115200;
	}
}

class HubComms 
{

	int count_l = 0;
	int count_r = 0; 
	public:

		HubComms() = default;

		void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
		{  
			timeout_ms_ = timeout_ms;
			serial_conn_.Open(serial_device);
			serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
			serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
			serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
		}

		void disconnect()
		{
			serial_conn_.Close();
		}

		bool connected() const
		{
			return serial_conn_.IsOpen();
		}

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

		void send_empty_msg()
		{	
			uint8_t data[10] = {0};
			for(int i = 0;i < 10;i++)
				serial_conn_.WriteByte(data[i]);
		}

		double mapper(int val, int inmin, int inmax, int outmin, int outmax)
		{
			return ((val - inmin) * (outmax - outmin) / (inmax - inmin)) + outmin;
		}

		void read_motor(double &vval_1, double &vval_2, double &pval_1, double &pval_2)
		{
			int flag = 0;
			uint8_t crc;
			uint8_t data1[10] = {0x01, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04};
			uint8_t data2[10] = {0x02, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF1};
			uint8_t response[10] = {0};

			for(int i = 0;i < 10;i++)
				serial_conn_.WriteByte(data1[i]);
			for(int i = 0;i < 10;i++)
			{	
				try
				{
					serial_conn_.ReadByte(response[i],50);
				}
				catch (...)
				{
					continue;
				}
			}
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

			vval_1 = unsigned(response[4])*255 + unsigned(response[5]);
			if(flag == 1 and vval_1 != 0)
				vval_1 = -1 * vval_1;
			//pval_1 = 360 - mapper(unsigned(response[7]),0,255,0,359);

			flag = 0;
			for(int i = 0;i < 10;i++)
				serial_conn_.WriteByte(data2[i]);
			for(int i = 0;i < 10;i++)
			{	
				try
				{
					serial_conn_.ReadByte(response[i],50);
				}
				catch (...)
				{
					continue;
				}
			}
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
			vval_2 = unsigned(response[4])*255 + unsigned(response[5]);
			if(flag == 0 and vval_2 != 0)
				vval_2 = -1 * vval_2;
			//pval_2 = 360 - mapper(unsigned(response[7]),0,255,0,359);
		}

		void set_motor_values(double val_l, double val_r, double &v_l, double &v_r)
		{
			int flag = 0;
			int val_1 = int(val_l);
			int val_2 = int(val_r);
			val_2 = -1 * val_2;
			uint8_t crc;
			uint8_t response[10];
			uint8_t data1[10],data2[10];
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
			for(int i = 0;i < 10;i++)
				serial_conn_.WriteByte(data1[i]);
			for(int i = 0;i < 10;i++)
			{	
				try
				{
					serial_conn_.ReadByte(response[i],50);
				}
				catch (...)
				{
					continue;
				}
			}
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

			v_l = unsigned(response[4])*255 + unsigned(response[5]);
			if(flag == 1 and v_l != 0)
				v_l = -1 * v_l;

			flag = 0;
    		for(int i = 0;i < 10;i++)
				serial_conn_.WriteByte(data2[i]);
			for(int i = 0;i < 10;i++)
			{	
				try
				{
					serial_conn_.ReadByte(response[i],50);
				}
				catch (...)
				{
					continue;
				}
			}
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
			v_r = unsigned(response[4])*255 + unsigned(response[5]);
			if(flag == 0 and v_r != 0)
				v_r = -1 * v_r;
    	}

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP