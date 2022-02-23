#include <cyglidar_pcl.h>
#include "Constants_CygLiDAR_D1.h"

using namespace CygLiDARD1::Command;

cyglidar_pcl::cyglidar_pcl(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
    :port_(port), baud_rate_(baud_rate), serial_(io, port_)
{
    try
    {
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Error Caught: %s", e.what());
        return;
    }       
}

// serial receive
uint16_t cyglidar_pcl::rvData(uint8_t* outputBuffer, const int BufferSize)
{
    boost::system::error_code errorCode;

    using namespace boost::asio;
    int numOfRvData = read(serial_, buffer(outputBuffer, BufferSize), transfer_at_least(1), errorCode);

	if (errorCode) return 0;
        
    return numOfRvData;
}


void cyglidar_pcl::packet_run(const eRunMode mode)
{
    // set payload header
    switch (mode)
    {
        case eRunMode::Mode2D:
            payloadBuffer[0] = Payload::Run::PayloadHeader::Run2D;
            ROS_INFO("PACKET SENT (START, 2D)");
            break;
        case eRunMode::Mode3D:
            payloadBuffer[0] = Payload::Run::PayloadHeader::Run3D;
            ROS_INFO("PACKET SENT (START, 3D)");
            break;            
        case eRunMode::ModeDual:
            payloadBuffer[0] = Payload::Run::PayloadHeader::RunDual;
            ROS_INFO("PACKET SENT (START, Dual)");
            break;
    }
    payloadBuffer[1] = 0x00;

    // make packet to CommandBuffer
    makeCommand(CommandBuffer, payloadBuffer, Payload::Run::PayloadTotalSize);

    // Header Size + Payload Size + Checksum Size
    uint16_t CommandTotalSize = (Payload::Run::PayloadTotalSize) 
    + (Header::HeaderTotalSize) + 1;

    // serial write
    boost::asio::write(serial_, boost::asio::buffer(CommandBuffer, CommandTotalSize));
}

void cyglidar_pcl::packet_duration(const eRunMode mode, const bool setAutoDuration, const uint16_t Duration)
{
    if(mode == eRunMode::Mode2D || Duration > Payload::Duration::MaximumDurationValue) 
        return;
    uint8_t MSB = (Duration & 0xFF) >> 8;
    uint8_t LSB = Duration & 0xFF;

    // 1st bit, set 3D or Dual 
    if(mode == eRunMode::Mode3D) MSB |= (1 << 4);
    // 2nd bit, set Auto or Fixed
    if(setAutoDuration == false) MSB |= (1 << 3);

    // set payload header
    payloadBuffer[0] = Payload::Duration::PayloadHeader::Duration;

    // set payload data
    payloadBuffer[1] = LSB;
    payloadBuffer[2] = MSB;

    // make packet to CommandBuffer
    makeCommand(CommandBuffer, payloadBuffer, Payload::Duration::PayloadTotalSize);

    // Header Size + Payload Size + Checksum Size
    uint16_t CommandTotalSize = (Payload::Duration::PayloadTotalSize) + (Header::HeaderTotalSize) + 1;

    // serial write
    boost::asio::write(serial_, boost::asio::buffer(CommandBuffer, CommandTotalSize));
	ROS_INFO("PACKET_DURATION HAS BEEN APPLIED [%d]", Duration);
    packet_confirmation(CommandBuffer, CommandTotalSize);
}

void cyglidar_pcl::packet_frequency(const uint8_t channel)
{
    // set payload header
    payloadBuffer[0] = Payload::Frequency::PayloadHeader::SetFreqeuncy;
    // set payload data
    payloadBuffer[1] = channel;
    // make packet to CommandBuffer
    makeCommand(CommandBuffer, payloadBuffer, Payload::Frequency::PayloadTotalSize);
    // Header Size + Payload Size + Checksum Size
    uint16_t CommandTotalSize = (Payload::Frequency::PayloadTotalSize) + (Header::HeaderTotalSize) + 1;
    // serial write
    boost::asio::write(serial_, boost::asio::buffer(CommandBuffer, CommandTotalSize));
    ROS_INFO("PACKET_FREQUENCY HAS BEEN UPDATED [%d]", channel);
    packet_confirmation(&CommandBuffer[0], CommandTotalSize);
}

void cyglidar_pcl::close()
{
    payloadBuffer[0] = Payload::Stop::PayloadHeader::Stop;
    payloadBuffer[1] = 0x00;

    // make packet to CommandBuffer
    makeCommand(CommandBuffer, payloadBuffer, Payload::Stop::PayloadTotalSize);

    // Header Size + Payload Size + Checksum Size
    uint16_t CommandTotalSize = (Payload::Stop::PayloadTotalSize) + (Header::HeaderTotalSize) + 1;

    // serial write
    boost::asio::write(serial_, boost::asio::buffer(CommandBuffer, CommandTotalSize));
    ROS_INFO("PACKET SENT (STOP)");
    packet_confirmation(&CommandBuffer[0], CommandTotalSize);
}

void cyglidar_pcl::makeCommand(uint8_t* commandBuffer, uint8_t* Payload, const uint16_t payloadSize)
{
    int buffidx = 0;
    uint8_t checksum = 0;
    // set Packet Header
    commandBuffer[buffidx++] = Header::Header1;
    commandBuffer[buffidx++] = Header::Header2;
    commandBuffer[buffidx++] = Header::Header3;

    commandBuffer[buffidx] = (payloadSize & 0x00FF);
    checksum ^= commandBuffer[buffidx++];   // calc Checksum

    commandBuffer[buffidx] = (payloadSize & 0xFF00) >> 8;
    checksum ^= commandBuffer[buffidx++];   // calc Checksum

    for(uint16_t i = 0; i < payloadSize; i++)
    {
        commandBuffer[buffidx] = Payload[i];
        checksum ^= commandBuffer[buffidx++];   // calc Checksum
    }

    commandBuffer[buffidx] = checksum;
}

void cyglidar_pcl::packet_confirmation(uint8_t* buffer, int count)
{
    printf("\tPACKET: ");
    for (int buf = 0; buf < count; buf++)
    {
        printf("%x", buffer[buf]);
        if (buf < (count - 1)) printf(",");
    }
    printf("\n");
}