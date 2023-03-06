#include "serial.h"
#include "d_series_constant.h"

using namespace CygLiDARD1::Command;

cyglidar_pcl::cyglidar_pcl(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
    :port_(port), baud_rate_(baud_rate), serial_(io, port_)
{
    serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
}

uint16_t cyglidar_pcl::getPacketLength(uint8_t* output_buffer, const int buffer_size)
{
    boost::system::error_code error_code_;

    using namespace boost::asio;
    int number_of_received_data = read(serial_, buffer(output_buffer, buffer_size), transfer_at_least(1), error_code_);

	if (error_code_) return 0;

    return number_of_received_data;
}

std::string cyglidar_pcl::requestRunMode(const eRunMode run_mode)
{
    const char *notice;

    switch (run_mode)
    {
        case eRunMode::Mode2D:
            payload_buffer[0] = Payload::Run::PayloadHeader::Run2D;
            notice = "[PACKET UPDATED] RUN 2D MODE";
            break;
        case eRunMode::Mode3D:
            payload_buffer[0] = Payload::Run::PayloadHeader::Run3D;
            notice = "[PACKET UPDATED] RUN 3D MODE";
            break;
        case eRunMode::ModeDual:
            payload_buffer[0] = Payload::Run::PayloadHeader::RunDual;
            notice = "[PACKET UPDATED] RUN DUAL MODE";
            break;
    }
    payload_buffer[1] = 0x00;
    makeCommand(command_buffer, payload_buffer, Payload::Run::PayloadTotalSize);

    uint16_t runmode_command_totalsize = (Payload::Run::PayloadTotalSize) + (Header::HeaderTotalSize) + 1;

    boost::asio::write(serial_, boost::asio::buffer(command_buffer, runmode_command_totalsize));

    return notice;
}

void cyglidar_pcl::requestDurationControl(const eRunMode run_mode_, const int duration_mode_, const uint16_t duration_value_)
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

    boost::asio::write(serial_, boost::asio::buffer(command_buffer, duration_command_totalsize));
}

void cyglidar_pcl::requestFrequencyChannel(const uint8_t channel_number_)
{
    payload_buffer[0] = Payload::Frequency::PayloadHeader::SetFreqeuncy;
    payload_buffer[1] = channel_number_;
    makeCommand(command_buffer, payload_buffer, Payload::Frequency::PayloadTotalSize);

    uint16_t frequency_command_totalsize = (Payload::Frequency::PayloadTotalSize) + (Header::HeaderTotalSize) + 1;

    boost::asio::write(serial_, boost::asio::buffer(command_buffer, frequency_command_totalsize));
}

void cyglidar_pcl::requestSensitivity(const uint8_t sensitivity_value)
{
    payload_buffer[0] = Payload::Sensitivity::PayloadHeader::SetSensitivity;
    payload_buffer[1] = sensitivity_value;
    makeCommand(command_buffer, payload_buffer, Payload::Sensitivity::PayloadTotalSize);

    uint16_t sensitivity_command_totalsize = (Payload::Sensitivity::PayloadTotalSize) + (Header::HeaderTotalSize) + 1;

    boost::asio::write(serial_, boost::asio::buffer(command_buffer, sensitivity_command_totalsize));
}

void cyglidar_pcl::close()
{
    payload_buffer[0] = Payload::Stop::PayloadHeader::Stop;
    payload_buffer[1] = 0x00;
    makeCommand(command_buffer, payload_buffer, Payload::Stop::PayloadTotalSize);

    uint16_t close_command_totalsize = (Payload::Stop::PayloadTotalSize) + (Header::HeaderTotalSize) + 1;

    boost::asio::write(serial_, boost::asio::buffer(command_buffer, close_command_totalsize));
}

// make the intended request packet to command buffer
void cyglidar_pcl::makeCommand(uint8_t* command_buffer_, uint8_t* payload, const uint16_t payloadSize)
{
    int buffidx = 0;
    uint8_t checksum = 0;
    command_buffer_[buffidx++] = Header::Header1;
    command_buffer_[buffidx++] = Header::Header2;
    command_buffer_[buffidx++] = Header::Header3;

    command_buffer_[buffidx] = (payloadSize & 0x00FF);
    checksum ^= command_buffer_[buffidx++];

    command_buffer_[buffidx] = (payloadSize & 0xFF00) >> 8;
    checksum ^= command_buffer_[buffidx++];

    for(uint16_t i = 0; i < payloadSize; i++)
    {
        command_buffer_[buffidx] = payload[i];
        checksum ^= command_buffer_[buffidx++];
    }
    command_buffer_[buffidx] = checksum;
}
