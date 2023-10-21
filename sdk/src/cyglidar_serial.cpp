#include "cyglidar_serial.h"

CyglidarSerial::CyglidarSerial() {}

CyglidarSerial::~CyglidarSerial()
{
    if (serial)
	{
		serial->cancel();
		serial->close();
		serial.reset();
	}

	io_service.stop();
	io_service.reset();
}

void CyglidarSerial::openSerial(const std::string &_port, const uint8_t _baudrate)
{
	if (serial)
	{
		std::cout << "[ERRPR] PORT IS ALREADY OPENED..." << std::endl;
	}

    serial = std::make_shared<boost::asio::serial_port>(io_service);

	serial->open(_port, error_code);

	if (error_code)
	{
		std::cout << "[BOOST SERIAL ERROR] TRIED TO CONNECT WITH \"port=" << _port
				  << "\", " << error_code.message().c_str() << std::endl;
	}

    baud_rate = getBaudRate(_baudrate);

	serial->set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
	serial->set_option(boost::asio::serial_port_base::character_size(8));
	serial->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
	serial->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
	serial->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
}

uint16_t CyglidarSerial::getPacketLength(uint8_t* _received_buffer, const uint16_t _buffer_size)
{
    if (serial.get() == NULL || !serial->is_open()) return 0;

    number_of_packet = serial->read_some(boost::asio::buffer(_received_buffer, _buffer_size), error_code);

	if (error_code) return 0;

    return number_of_packet;
}

void CyglidarSerial::requestRunMode(const eRunMode _run_mode, std::string &_notice)
{
    payload_buffer.clear();

    switch (_run_mode)
    {
        case eRunMode::Mode2D:
            payload_buffer.push_back(Payload::Run::PayloadHeader::Run2D);
            _notice = "RUN 2D MODE";
            break;
        case eRunMode::Mode3D:
            payload_buffer.push_back(Payload::Run::PayloadHeader::Run3D);
            _notice = "RUN 3D MODE";
            break;
        case eRunMode::ModeDual:
            payload_buffer.push_back(Payload::Run::PayloadHeader::RunDual);
            _notice = "RUN DUAL MODE";
            break;
    }
    payload_buffer.push_back(Payload::Run::PayloadData::data);

    sendCommand(payload_buffer);
}

void CyglidarSerial::requestDurationControl(const eRunMode _run_mode, const uint8_t _duration_mode, const uint16_t _duration_value)
{
    payload_buffer.clear();

    if(_run_mode == eRunMode::Mode2D || _duration_value > Payload::Duration::MaximumDurationValue)
    {
        return;
    }

    uint8_t MSB = (_duration_value & 0xFF00) >> 8;
    uint8_t LSB = _duration_value & 0x00FF;

    if(_run_mode == eRunMode::ModeDual)
    {
        MSB |= (1 << 7); // 1st bit, set Dual(1)  or 3D
    }

    if(_duration_mode == PULSE_MANUAL)
    {
        MSB |= (1 << 6); // 2nd bit, set Fixed(1) or Auto
    }

    payload_buffer.push_back(Payload::Duration::PayloadHeader::Duration);
    payload_buffer.push_back(LSB);
    payload_buffer.push_back(MSB);

    sendCommand(payload_buffer);
}

void CyglidarSerial::requestFrequencyChannel(const uint8_t _channel_number)
{
    payload_buffer.clear();
    payload_buffer.push_back(Payload::Frequency::PayloadHeader::SetFreqeuncy);
    payload_buffer.push_back(_channel_number);

    sendCommand(payload_buffer);
}

void CyglidarSerial::requestDeviceInfo()
{
    payload_buffer.clear();
    payload_buffer.push_back(Payload::DeviceInfo::PayloadHeader::Version);
    payload_buffer.push_back(Payload::DeviceInfo::PayloadData::data);

    sendCommand(payload_buffer);
}

void CyglidarSerial::requestSerialBaudRate(const uint8_t _select_baud_rate)
{
    payload_buffer.clear();
    payload_buffer.push_back(Payload::Baudrate::PayloadHeader::SetSerialbaudrate);
    payload_buffer.push_back(_select_baud_rate);

    sendCommand(payload_buffer);
}

uint32_t CyglidarSerial::getBaudRate(uint8_t _baud_rate_mode)
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

void CyglidarSerial::closeSerial()
{
    payload_buffer.clear();
    payload_buffer.push_back(Payload::Stop::PayloadHeader::Stop);
    payload_buffer.push_back(Payload::Stop::PayloadData::data);

    sendCommand(payload_buffer);
}

// make the intended request packet to command buffer
void CyglidarSerial::sendCommand(const std::vector<uint8_t> &payload)
{
    uint8_t check_sum = 0;

    command_buffer.clear();
    command_buffer.push_back(Header::Header1);
    command_buffer.push_back(Header::Header2);
    command_buffer.push_back(Header::Header3);

    command_buffer.push_back(payload.size());
    check_sum ^= payload.size();

    command_buffer.push_back(0x00);
    check_sum ^= 0x00;

    for(uint8_t i = 0; i < payload.size(); i++)
    {
        uint8_t buffer_index = PAYLOAD_HEADER + i;

        command_buffer.push_back(payload[i]);
        check_sum ^= command_buffer[buffer_index];
    }

    command_buffer.push_back(check_sum);

    serial->write_some(boost::asio::buffer(command_buffer, command_buffer.size()), error_code);
}
