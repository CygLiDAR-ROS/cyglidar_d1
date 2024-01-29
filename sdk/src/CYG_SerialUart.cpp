#include "CYG_SerialUart.h"

CYG_SerialUart::CYG_SerialUart() {}

CYG_SerialUart::~CYG_SerialUart()
{
    if (serial_port)
	{
		serial_port->cancel();
		serial_port->close();
		serial_port.reset();
	}

	io_service.stop();
	io_service.reset();
}

void CYG_SerialUart::openSerialPort(const std::string &_port, const uint8_t _baudrate)
{
	if (serial_port)
	{
		std::cout << "[ERRPR] PORT IS ALREADY OPENED..." << std::endl;
	}

    serial_port = std::make_shared<boost::asio::serial_port>(io_service);

	serial_port->open(_port, error_code);

	if (error_code)
	{
		std::cout << "[BOOST SERIAL ERROR] TRIED TO CONNECT WITH \"port_number=" << _port
				  << "\", " << error_code.message().c_str() << std::endl;
	}

    baud_rate = getBaudRate(_baudrate);

	serial_port->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	serial_port->set_option(boost::asio::serial_port_base::character_size(8));
	serial_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	serial_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	serial_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
}

uint16_t CYG_SerialUart::getPacketLength(uint8_t* _received_buffer)
{
    if (serial_port.get() == NULL || !serial_port->is_open()) return 0;

    number_of_packet = serial_port->read_some(boost::asio::buffer(_received_buffer, D1_Const::SCAN_MAX_SIZE), error_code);

	if (error_code) return 0;

    return number_of_packet;
}

void CYG_SerialUart::requestRunMode(const uint8_t _run_mode, std::string &_notice)
{
    payload_buffer.clear();

    switch (_run_mode)
    {
        case ROS_Const::MODE_2D:
            payload_buffer.push_back(D1_Const::SEND_DEPTH_2D);
            _notice = "RUN 2D MODE";
            break;
        case ROS_Const::MODE_3D:
            payload_buffer.push_back(D1_Const::SEND_DEPTH_3D);
            _notice = "RUN 3D MODE";
            break;
        case ROS_Const::MODE_DUAL:
            payload_buffer.push_back(D1_Const::SEND_DEPTH_DUAL);
            _notice = "RUN DUAL MODE";
            break;
    }
    payload_buffer.push_back(D1_Const::COMMAND_DATA);

    transferPacketCommand(payload_buffer);
}

void CYG_SerialUart::requestDurationControl(const uint8_t _run_mode, const uint8_t _duration_mode, const uint16_t _duration_value)
{
    payload_buffer.clear();

    if(_run_mode == ROS_Const::MODE_2D || _duration_value > D1_Const::INTEGRATION_MAX_VALUE)
    {
        return;
    }

    uint8_t MSB = (_duration_value & 0xFF00) >> 8;
    uint8_t LSB = _duration_value & 0x00FF;

    if(_duration_mode == ROS_Const::PULSE_MANUAL)
    {
        MSB |= (1 << 6); // 2nd bit, set Fixed(1) or Auto
    }

    payload_buffer.push_back(D1_Const::INTEGRATION_TIME);
    payload_buffer.push_back(LSB);
    payload_buffer.push_back(MSB);

    transferPacketCommand(payload_buffer);
}

void CYG_SerialUart::requestFrequencyChannel(const uint8_t _channel_number)
{
    payload_buffer.clear();
    payload_buffer.push_back(D1_Const::FREQUENCY_CHANNEL);
    payload_buffer.push_back(_channel_number);

    transferPacketCommand(payload_buffer);
}

void CYG_SerialUart::requestDeviceInfo()
{
    payload_buffer.clear();
    payload_buffer.push_back(D1_Const::DEVICE_INFO);
    payload_buffer.push_back(D1_Const::COMMAND_DATA);

    transferPacketCommand(payload_buffer);
}

void CYG_SerialUart::requestSerialBaudRate(const uint8_t _select_baud_rate)
{
    payload_buffer.clear();
    payload_buffer.push_back(D1_Const::SET_BAUDRATE);
    payload_buffer.push_back(_select_baud_rate);

    transferPacketCommand(payload_buffer);
}

uint32_t CYG_SerialUart::getBaudRate(uint8_t _baud_rate_mode)
{
    uint32_t result = 0;

    switch(_baud_rate_mode)
    {
        case 0:
            result = 3000000;
            break;
        case 1:
            result = 921600;
            break;
        case 2:
            result = 115200;
            break;
        case 3:
            result = 57600;
            break;
    }

    return result;
}

void CYG_SerialUart::closeSerialPort()
{
    payload_buffer.clear();
    payload_buffer.push_back(D1_Const::SEND_STOP);
    payload_buffer.push_back(D1_Const::COMMAND_DATA);

    transferPacketCommand(payload_buffer);
}

void CYG_SerialUart::transferPacketCommand(const std::vector<uint8_t> &payload)
{
    uint8_t check_sum = 0;

    command_buffer.clear();
    command_buffer.push_back(D1_Const::NORMAL_MODE);
    command_buffer.push_back(D1_Const::PRODUCT_CODE);
    command_buffer.push_back(D1_Const::DEFAULT_ID);

    command_buffer.push_back(payload.size());
    check_sum ^= payload.size();

    command_buffer.push_back(0x00);
    check_sum ^= 0x00;

    for(uint8_t i = 0; i < payload.size(); i++)
    {
        uint8_t buffer_index = D1_Const::PAYLOAD_HEADER + i;

        command_buffer.push_back(payload[i]);
        check_sum ^= command_buffer[buffer_index];
    }

    command_buffer.push_back(check_sum);

    serial_port->write_some(boost::asio::buffer(command_buffer, command_buffer.size()), error_code);
}
