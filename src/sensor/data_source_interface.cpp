#include "uavloc/sensor/data_source_interface.h"

namespace uavloc {
namespace sensor {

DataSourceInterface::AttitudeSlot& DataSourceInterface::attitudeChannel() {
    return attitude_channel_;
}

DataSourceInterface::GimbalSlot& DataSourceInterface::gimbalChannel() {
    return gimbal_channel_;
}

DataSourceInterface::GnssSlot& DataSourceInterface::gnssChannel() {
    return gnss_channel_;
}

DataSourceInterface::ImageSlot& DataSourceInterface::imageChannel() {
    return image_channel_;
}

DataSourceInterface::EventSlot& DataSourceInterface::eventChannel() {
    return event_channel_;
}

void DataSourceInterface::clearCallbacks() {
    attitude_channel_.clear();
    gimbal_channel_.clear();
    gnss_channel_.clear();
    image_channel_.clear();
    event_channel_.clear();
}

}  // namespace sensor
}  // namespace uavloc
