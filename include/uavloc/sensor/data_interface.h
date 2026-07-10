#ifndef DATA_INTERFACE_H
#define DATA_INTERFACE_H

#include "uavloc/sensor/frame_data.h"

namespace uavloc::sensor {
class DataInterface {
public:
    virtual ~DataInterface() = default;

    virtual bool open() = 0; 
    virtual FrameStatus read(FrameData& frame) = 0; 
    virtual void close() = 0;

    virtual bool isOpened() const = 0;
    virtual std::string name() const = 0;
};
}

#endif  // DATA_INTERFACE_H