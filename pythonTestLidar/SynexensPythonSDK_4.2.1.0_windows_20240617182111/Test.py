from SynexensPythonSDK import *

def PrintErrorCode(func_name, errorCode):
    error_map = {v[0] if isinstance(v, tuple) else v: k for k, v in SYErrorCode.__dict__.items() if isinstance(v, (int, tuple))}
    error_name = error_map.get(errorCode, "Unknown Error")
    print(f"Error in {func_name}: {errorCode} ({error_name})")

def main():
    print("Starting InitSDK")
    error = InitSDK()
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("InitSDK", error)
        exit(1)
    print("InitSDK succeeded")
    nDeviceCount = c_int32()
    print("Calling FindDevice")
    error = FindDevice(byref(nDeviceCount), None)
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("FindDevice", error)
        UnInitSDK()
        exit(1)
    print(f"Device count: {nDeviceCount.value}")
    if nDeviceCount.value <= 0:
        print("No devices found")
        UnInitSDK()
        exit(1)
    pDeviceInfo = (SYDeviceInfo * nDeviceCount.value)()
    print("Fetching device info")
    error = FindDevice(nDeviceCount, pDeviceInfo)
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("FindDevice - Fetch Info", error)
        UnInitSDK()
        exit(1)
    device_id = pDeviceInfo[0].m_nDeviceID
    print(f"Opening device ID: {device_id}")
    error = OpenDevice(pDeviceInfo[0])
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("OpenDevice", error)
        UnInitSDK()
        exit(1)
    print(f"Opened Device ID: {device_id}")
    error = SetFrameResolution(device_id, SYFrameTypeEnum.SYFRAMETYPE_DEPTH, SYResolutionEnum.SYRESOLUTION_640_480)
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("SetFrameResolution", error)
        exit(1)
    error = StartStreaming(device_id, SYStreamTypeEnum.SYSTREAMTYPE_DEPTH)
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("StartStreaming", error)
        exit(1)
    print("Streaming started")
    time.sleep(5.0)
    for _ in range(10):
        frame_data = POINTER(SYFrameData)()
        error = GetLastFrameData(device_id, byref(frame_data))
        if error == SYErrorCode.SYERRORCODE_SUCCESS and frame_data:
            print("Got a frame")
        else:
            PrintErrorCode("GetLastFrameData", error)
        time.sleep(0.5)
    StopStreaming(device_id)
    CloseDevice(device_id)
    UnInitSDK()

if __name__ == "__main__":
    main()