//
//  stream_delegate.h
//  CameraSDK
//
//  Created by capjason on 2020/3/18.
//

#ifndef stream_delegate_h
#define stream_delegate_h
#include "stream_types.h"
#include <camera/ins_types.h>

namespace ins_camera {
class CAMERASDK_API StreamDelegate {
public:
    virtual void OnAudioData(const uint8_t* data, size_t size, int64_t timestamp) = 0;
    virtual void OnVideoData(const uint8_t* data,size_t size, int64_t timestamp, uint8_t streamType, int stream_index = 0) = 0;
    virtual void OnGyroData(const std::vector<GyroData>& data) = 0;
    virtual void OnExposureData(const ExposureData& data) = 0;
};
}

#endif /* stream_delegate_h */
