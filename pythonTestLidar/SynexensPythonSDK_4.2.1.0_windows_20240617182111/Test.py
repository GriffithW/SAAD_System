import os
import time
import numpy as np
from ctypes import *
from SynexensPythonSDK import *

# Error printing function
def PrintErrorCode(func_name, errorCode):
    error_map = {v[0] if isinstance(v, tuple) else v: k for k, v in SYErrorCode.__dict__.items() if isinstance(v, (int, tuple))}
    error_name = error_map.get(errorCode, "Unknown Error")
    print(f"Error in {func_name}: {errorCode} ({error_name})")

# Initialize SDK and device
def initialize_device():
    error = InitSDK()
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("InitSDK", error)
        exit(1)

    nDeviceCount = c_int32()
    error = FindDevice(byref(nDeviceCount), None)
    if error != SYErrorCode.SYERRORCODE_SUCCESS or nDeviceCount.value <= 0:
        PrintErrorCode("FindDevice - Initial Check", error)
        UnInitSDK()
        exit(1)

    print(f"Found {nDeviceCount.value} device(s)")
    pDeviceInfo = (SYDeviceInfo * nDeviceCount.value)()
    error = FindDevice(nDeviceCount, pDeviceInfo)
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("FindDevice - Fetch Info", error)
        UnInitSDK()
        exit(1)

    for i in range(nDeviceCount.value):
        device_type = pDeviceInfo[i].m_deviceType.value
        device_id = pDeviceInfo[i].m_nDeviceID
        print(f"Device {i}: ID={device_id}, Type={device_type} (CS20_SINGLE={SYDeviceTypeEnum.SYDEVICETYPE_CS20_SINGLE})")
        for attempt in range(3):
            error = OpenDevice(pDeviceInfo[i])
            if error == SYErrorCode.SYERRORCODE_SUCCESS:
                break
            PrintErrorCode(f"OpenDevice (attempt {attempt + 1})", error)
            time.sleep(1)
        else:
            print("Failed to open device after retries")
            UnInitSDK()
            exit(1)
        print(f"Opened Device ID: {device_id}, SN: {GetDeviceSN(device_id)}")
        return device_id, pDeviceInfo[i]

    print("No compatible device found")
    UnInitSDK()
    exit(1)

# Get point cloud data
def get_point_cloud(device_id):
    frame_data = POINTER(SYFrameData)()  # Match test script's pointer approach
    for attempt in range(3):  # Retry to handle intermittent failures
        error = GetLastFrameData(device_id, byref(frame_data))
        if error == SYErrorCode.SYERRORCODE_SUCCESS and frame_data:
            break
        PrintErrorCode(f"GetLastFrameData (attempt {attempt + 1})", error)
        time.sleep(0.5)
    else:
        print("Failed to get frame data after retries")
        return None

    obj_frame_data = frame_data.contents
    if obj_frame_data.m_nFrameCount <= 0 or obj_frame_data.m_nFrameCount > 1000:
        print(f"Invalid frame count: {obj_frame_data.m_nFrameCount}")
        return None

    if not obj_frame_data.m_pFrameInfo:
        print("m_pFrameInfo is NULL")
        return None

    nPos = 0
    for i in range(obj_frame_data.m_nFrameCount):
        frame_info = obj_frame_data.m_pFrameInfo[i]
        if frame_info.m_frameType == SYFrameTypeEnum.SYFRAMETYPE_DEPTH:
            nCount = frame_info.m_nFrameHeight * frame_info.m_nFrameWidth
            pDepth = (c_ushort * nCount)()
            ptr_void = c_void_p(obj_frame_data.m_pData + nPos)
            memmove(pDepth, cast(ptr_void, POINTER(c_ushort)), nCount * sizeof(c_ushort))

            point_cloud_data = (SYPointCloudData * nCount)()
            error = GetDepthPointCloud(device_id, frame_info.m_nFrameWidth, frame_info.m_nFrameHeight, pDepth, point_cloud_data)
            if error != SYErrorCode.SYERRORCODE_SUCCESS:
                PrintErrorCode("GetDepthPointCloud", error)
                return None

            points = np.array([[p.m_fltX, p.m_fltY, p.m_fltZ] for p in point_cloud_data], dtype=np.float32)
            return points
        nPos += frame_info.m_nFrameHeight * frame_info.m_nFrameWidth * sizeof(c_ushort)

    print("No depth frame found")
    return None

# Main execution
def main():
    device_id, device_info = initialize_device()

    error = SetFrameResolution(device_id, SYFrameTypeEnum.SYFRAMETYPE_DEPTH, SYResolutionEnum.SYRESOLUTION_640_480)
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("SetFrameResolution", error)
        CloseDevice(device_id)
        UnInitSDK()
        exit(1)
    print("Resolution set to 640x480")

    # Try DEPTHIR first (from test script), then DEPTH
    for stream_type in [SYStreamTypeEnum.SYSTREAMTYPE_DEPTHIR, SYStreamTypeEnum.SYSTREAMTYPE_DEPTH]:
        error = StartStreaming(device_id, stream_type)
        if error == SYErrorCode.SYERRORCODE_SUCCESS:
            print(f"Streaming started with {stream_type} mode")
            break
        PrintErrorCode(f"StartStreaming ({stream_type})", error)
    else:
        print("Both stream types failed")
        CloseDevice(device_id)
        UnInitSDK()
        exit(1)

    time.sleep(2)  # Wait for streaming to stabilize

    try:
        iteration = 0
        while True:
            print(f"\nIteration {iteration}")
            point_cloud = get_point_cloud(device_id)
            if point_cloud is None:
                print("No point cloud data retrieved")
            else:
                print(f"Retrieved {point_cloud.shape[0]} points")
            time.sleep(0.1)
            iteration += 1

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        StopStreaming(device_id)
        CloseDevice(device_id)
        UnInitSDK()

if __name__ == "__main__":
    main()