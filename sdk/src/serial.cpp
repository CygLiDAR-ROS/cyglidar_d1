#include "serial.h"

using namespace Constant_D1::Command;

cyglidar_serial::cyglidar_serial(const std::string& port_, uint32_t baud_rate_, boost::asio::io_service& io_)
    :port(port_), baud_rate(baud_rate_), serial(io_, port_)
{
    serial.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
}

uint16_t cyglidar_serial::getPacketLength(uint8_t* output_buffer_, const int buffer_size_)
{
    boost::system::error_code error_code;

    using namespace boost::asio;
    int number_of_received_data = read(serial, buffer(output_buffer_, buffer_size_), transfer_at_least(1), error_code);

	if (error_code) return 0;

    return number_of_received_data;
}

std::string cyglidar_serial::requestRunMode(const eRunMode run_mode_)
{
    const char *notice;

    switch (run_mode_)
    {
        case eRunMode::Mode2D:
            payload_buffer[0] = Payload::Run::PayloadHeader::Run2D;
            notice = "[PACKET REQUEST] RUN 2D MODE";
            break;
        case eRunMode::Mode3D:
            payload_buffer[0] = Payload::Run::PayloadHeader::Run3D;
            notice = "[PACKET REQUEST] RUN 3D MODE";
            break;
        case eRunMode::ModeDual:
            payload_buffer[0] = Payload::Run::PayloadHeader::RunDual;
            notice = "[PACKET REQUEST] RUN DUAL MODE";
            break;
    }
    payload_buffer[1] = 0x00;
    makeCommand(command_buffer, payload_buffer, Payload::Run::PayloadTotalSize);

    uint16_t runmode_command_totalsize = (Payload::Run::PayloadTotalSize) + (Header::HeaderTotalSize) + 1;

    boost::asio::write(serial, boost::asio::buffer(command_buffer, runmode_command_totalsize));

    return notice;
}

void cyglidar_serial::requestDurationControl(const eRunMode run_mode_, const int duration_mode_, const uint16_t duration_value_)
{
    if(run_mode_ == eRunMode::Mode2D || duration_value_ > Payload::Duration::MaximumDurationValue)
        return;
    uint8_t MSB = (duration_value_ & 0xFF00) >> 8;
    uint8_t LSB = duration_value_ & 0x00FF;

    if(run_mode_ == eRunMode::ModeDual) MSB |= (1 << 7); // 1st bit, set Dual(1)  or 3D
    if(duration_mode_ == PULSE_MANUAL)  MSB |= (1 << 6); // 2nd bit, set Fixed(1) or Auto

    payload_buffer[0] = Payload::Duration::PayloadHeader::Duration;
    payload_buffer[1] = LSB;
    payload_buffer[2] = MSB;
    makeCommand(command_buffer, payload_buffer, Payload::Duration::PayloadTotalSize);

    uint16_t duration_command_totalsize = (Payload::Duration::PayloadTotalSize) + (Header::HeaderTotalSize) + 1;

    boost::asio::write(serial, boost::asio::buffer(command_buffer, duration_command_totalsize));
}

void cyglidar_serial::requestFrequencyChannel(const uint8_t channel_number_)
{
    payload_buffer[0] = Payload::Frequency::PayloadHeader::SetFreqeuncy;
    payload_buffer[1] = channel_number_;
    makeCommand(command_buffer, payload_buffer, Payload::Frequency::PayloadTotalSize);

    uint16_t frequency_command_totalsize = (Payload::Frequency::PayloadTotalSize) + (Header::HeaderTotalSize) + 1;

    boost::asio::write(serial, boost::asio::buffer(command_buffer, frequency_command_totalsize));
}

void cyglidar_serial::requestDeviceInfo()
{
    payload_buffer[0] = Payload::DeviceInfo::PayloadHeader::Version;
    payload_buffer[1] = 0x00;
    makeCommand(command_buffer, payload_buffer, Payload::DeviceInfo::PayloadTotalSize);

    uint16_t device_info_command_totalsize = (Payload::DeviceInfo::PayloadTotalSize) + (Header::HeaderTotalSize) + 1;

    boost::asio::write(serial, boost::asio::buffer(command_buffer, device_info_command_totalsize));
}

void cyglidar_serial::close()
{
    payload_buffer[0] = Payload::Stop::PayloadHeader::Stop;
    payload_buffer[1] = 0x00;
    makeCommand(command_buffer, payload_buffer, Payload::Stop::PayloadTotalSize);

    uint16_t close_command_totalsize = (Payload::Stop::PayloadTotalSize) + (Header::HeaderTotalSize) + 1;

    boost::asio::write(serial, boost::asio::buffer(command_buffer, close_command_totalsize));
}

// make the intended request packet to command buffer
void cyglidar_serial::makeCommand(uint8_t* command_buffer_, uint8_t* payload_, const uint16_t payload_size_)
{
    int buffidx = 0;
    uint8_t checksum = 0;
    command_buffer_[buffidx++] = Header::Header1;
    command_buffer_[buffidx++] = Header::Header2;
    command_buffer_[buffidx++] = Header::Header3;

    command_buffer_[buffidx] = (payload_size_ & 0x00FF);
    checksum ^= command_buffer_[buffidx++];

    command_buffer_[buffidx] = (payload_size_ & 0xFF00) >> 8;
    checksum ^= command_buffer_[buffidx++];

    for(uint16_t i = 0; i < payload_size_; i++)
    {
        command_buffer_[buffidx] = payload_[i];
        checksum ^= command_buffer_[buffidx++];
    }
    command_buffer_[buffidx] = checksum;
}
