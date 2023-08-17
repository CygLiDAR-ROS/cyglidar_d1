#include "cyglidar_serial.h"

using namespace Constant_D1::Command;

cyglidar_serial::cyglidar_serial(const std::string& _port, uint32_t _baudrate,
                                 boost::asio::io_service &_io_service) : serial(_io_service)
{
    serial.open(_port, error_code);

    if (error_code)
    {
        std::cout << "[BOOST SERIAL ERROR] TRIED TO CONNECT WITH \"port=" << _port << "\", "
                  << error_code.message().c_str() << std::endl;
    }

    serial.set_option(boost::asio::serial_port_base::baud_rate(_baudrate));
    serial.set_option(boost::asio::serial_port_base::character_size(8));
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
}

cyglidar_serial::~cyglidar_serial()
{
    serial.close();
}

uint16_t cyglidar_serial::getPacketLength(uint8_t* _received_buffer, const uint16_t _buffer_size)
{
    number_of_packet = boost::asio::read(serial,
                                         boost::asio::buffer(_received_buffer, _buffer_size),
                                         boost::asio::transfer_at_least(1), error_code);

	if (error_code)
    {
        return 0;
    }

    return number_of_packet;
}

void cyglidar_serial::requestRunMode(const eRunMode _run_mode, std::string &_notice)
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

void cyglidar_serial::requestDurationControl(const eRunMode _run_mode, uint8_t _duration_mode, uint16_t _duration_value)
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

void cyglidar_serial::requestFrequencyChannel(const uint8_t _channel_number)
{
    payload_buffer.clear();
    payload_buffer.push_back(Payload::Frequency::PayloadHeader::SetFreqeuncy);
    payload_buffer.push_back(_channel_number);

    sendCommand(payload_buffer);
}

void cyglidar_serial::requestDeviceInfo()
{
    payload_buffer.clear();
    payload_buffer.push_back(Payload::DeviceInfo::PayloadHeader::Version);
    payload_buffer.push_back(Payload::DeviceInfo::PayloadData::data);

    sendCommand(payload_buffer);
}

void cyglidar_serial::requestSerialBaudRate(const uint8_t _select_baud_rate)
{
    payload_buffer.clear();
    payload_buffer.push_back(Payload::Baudrate::PayloadHeader::SetSerialbaudrate);
    payload_buffer.push_back(_select_baud_rate);

    sendCommand(payload_buffer);
}

void cyglidar_serial::close()
{
    payload_buffer.clear();
    payload_buffer.push_back(Payload::Stop::PayloadHeader::Stop);
    payload_buffer.push_back(Payload::Stop::PayloadData::data);

    sendCommand(payload_buffer);
}

// make the intended request packet to command buffer
void cyglidar_serial::sendCommand(const std::vector<uint8_t> &payload)
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

    boost::asio::write(serial, boost::asio::buffer(command_buffer, command_buffer.size()));
}
