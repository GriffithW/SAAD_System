import os
import time
import numpy as np
from sklearn.cluster import DBSCAN
from SynexensPythonSDK import *

# Error printing function
def PrintErrorCode(func_name, errorCode):
    error_map = {v[0] if isinstance(v, tuple) else v: k for k, v in SYErrorCode.__dict__.items() if isinstance(v, (int, tuple))}
    error_name = error_map.get(errorCode, "Unknown Error")
    print(f"Error in {func_name}: {errorCode} ({error_name})")

# Initialize device
def initialize_device():
    error = InitSDK()
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("InitSDK", error)
        exit(1)

    nDeviceCount = c_int32()
    error = FindDevice(byref(nDeviceCount), None)
    if error != SYErrorCode.SYERRORCODE_SUCCESS or nDeviceCount.value <= 0:
        PrintErrorCode("FindDevice", error)
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
        device_id = pDeviceInfo[i].m_nDeviceID
        for attempt in range(3):
            error = OpenDevice(pDeviceInfo[i])
            if error == SYErrorCode.SYERRORCODE_SUCCESS:
                break
            PrintErrorCode(f"OpenDevice (attempt {attempt + 1})", error)
            time.sleep(1)
        else:
            UnInitSDK()
            exit(1)
        print(f"Opened Device ID: {device_id}")
        return device_id, pDeviceInfo[i]

# Get point cloud data
def get_point_cloud(device_id):
    frame_data = POINTER(SYFrameData)()
    for attempt in range(3):
        error = GetLastFrameData(device_id, byref(frame_data))
        if error == SYErrorCode.SYERRORCODE_SUCCESS and frame_data:
            break
        PrintErrorCode(f"GetLastFrameData (attempt {attempt + 1})", error)
        time.sleep(0.5)
    else:
        return None

    obj_frame_data = frame_data.contents
    if obj_frame_data.m_nFrameCount <= 0:
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
    return None

# Downsample point cloud
def downsample_point_cloud(points, voxel_size=0.5):
    if points is None or len(points) == 0:
        return None
    min_bound = points.min(axis=0)
    voxel_index = ((points - min_bound) // voxel_size).astype(int)
    unique_voxels, indices = np.unique(voxel_index, axis=0, return_index=True)
    return points[indices]



# Detect objects with DBSCAN
def detect_objects(point_cloud, eps=0.2, min_samples=10):
    if point_cloud is None or len(point_cloud) == 0:
        print("No point cloud data")
        return 0

    # Filter invalid points
    valid_points = point_cloud[~np.all(point_cloud == 0, axis=1)]
    if len(valid_points) < min_samples:
        print(f"Too few valid points: {len(valid_points)}")
        return 0

    # Downsample
    downsampled_points = downsample_point_cloud(valid_points, voxel_size=0.05)
    if downsampled_points is None or len(downsampled_points) < min_samples:
        print(f"Too few points after downsampling: {len(downsampled_points)}")
        return 0

    # Cluster with DBSCAN
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(downsampled_points)
    labels = clustering.labels_
    num_objects = len(set(labels)) - (1 if -1 in labels else 0)  # Exclude noise
    return num_objects

# def detect_objects(point_cloud, eps=0.2, min_samples=10):
#     if point_cloud is None or len(point_cloud) == 0:
#         print("No point cloud data")
#         return 0

#     # Filter invalid points
#     valid_points = point_cloud[~np.all(point_cloud == 0, axis=1)]
#     if len(valid_points) < min_samples:
#         print(f"Too few valid points: {len(valid_points)}")
#         return 0

#     # Downsample with a 1 cm voxel size
#     downsampled_points = downsample_point_cloud(valid_points, voxel_size=0.10)
#     if downsampled_points is None or len(downsampled_points) < min_samples:
#         print(f"Too few points after downsampling: {len(downsampled_points)}")
#         return 0

#     # Cluster with DBSCAN
#     clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(downsampled_points)
#     labels = clustering.labels_
#     num_objects = len(set(labels)) - (1 if -1 in labels else 0)  # Exclude noise
#     return num_objects

# Main execution
# def main():
#     device_id, device_info = initialize_device()

#     error = SetFrameResolution(device_id, SYFrameTypeEnum.SYFRAMETYPE_DEPTH, SYResolutionEnum.SYRESOLUTION_640_480)
#     if error != SYErrorCode.SYERRORCODE_SUCCESS:
#         PrintErrorCode("SetFrameResolution", error)
#         CloseDevice(device_id)
#         UnInitSDK()
#         exit(1)

#     for stream_type in [SYStreamTypeEnum.SYSTREAMTYPE_DEPTHIR, SYStreamTypeEnum.SYSTREAMTYPE_DEPTH]:
#         error = StartStreaming(device_id, stream_type)
#         if error == SYErrorCode.SYERRORCODE_SUCCESS:
#             print(f"Streaming started with {stream_type}")
#             break
#         PrintErrorCode(f"StartStreaming ({stream_type})", error)
#     else:
#         CloseDevice(device_id)
#         UnInitSDK()
#         exit(1)

#     time.sleep(2)  # Stabilize streaming

#     try:
#         iteration = 0
#         while True:
#             print(f"\nIteration {iteration}")
#             point_cloud = get_point_cloud(device_id)
#             if point_cloud is None:
#                 time.sleep(0.1)
#                 iteration += 1
#                 continue

#             # Process every other frame
#             if iteration % 2 == 0:
#                 num_objects = detect_objects(point_cloud, eps=0.2, min_samples=10)
#                 print(f"Detected {num_objects} object(s)")
#             else:
#                 print("Skipping detection this frame")

#             time.sleep(0.1)
#             iteration += 1

#     except KeyboardInterrupt:
#         print("Shutting down...")
#     finally:
#         StopStreaming(device_id)
#         CloseDevice(device_id)
#         UnInitSDK()

# if __name__ == "__main__":
#     main()

def main():
    device_id, device_info = initialize_device()  # Assume this function exists

    # Set resolution (ensure it matches your stream type)
    error = SetFrameResolution(device_id, SYFrameTypeEnum.SYFRAMETYPE_DEPTH, SYResolutionEnum.SYRESOLUTION_640_480)
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        print(f"SetFrameResolution failed: {error}")
        CloseDevice(device_id)
        UnInitSDK()
        exit(1)

    # Start streaming with a simpler type
    stream_type = SYStreamTypeEnum.SYSTREAMTYPE_DEPTH
    print(f"Trying stream type {stream_type}")
    error = StartStreaming(device_id, stream_type)
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        print(f"StartStreaming failed: {error}")
        CloseDevice(device_id)
        UnInitSDK()
        exit(1)
    print(f"Streaming started with {stream_type}")

    time.sleep(5)  # Give the device time to stabilize

    try:
        iteration = 0
        while True:
            print(f"\nIteration {iteration}")
            point_cloud = get_point_cloud(device_id)  # With debug output added
            if point_cloud is None:
                print("No point cloud retrieved, retrying...")
                time.sleep(0.1)
                iteration += 1
                continue

            # Your object detection logic here
            num_objects = detect_objects(point_cloud)
            print(f"Detected {num_objects} object(s)")
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