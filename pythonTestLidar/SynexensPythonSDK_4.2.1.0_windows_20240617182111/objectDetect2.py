import threading
import queue
import time
import numpy as np
from sklearn.cluster import DBSCAN
from SynexensPythonSDK import *

def PrintErrorCode(func_name, errorCode):
    error_map = {v[0] if isinstance(v, tuple) else v: k for k, v in SYErrorCode.__dict__.items() if isinstance(v, (int, tuple))}
    error_name = error_map.get(errorCode, "Unknown Error")
    print(f"Error in {func_name}: {errorCode} ({error_name})")

def initialize_device():
    error = InitSDK()
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("InitSDK", error)
        exit(1)
    nDeviceCount = c_int32()
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
    error = FindDevice(nDeviceCount, pDeviceInfo)
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("FindDevice - Fetch Info", error)
        UnInitSDK()
        exit(1)
    for i in range(nDeviceCount.value):
        try:
            name = pDeviceInfo[i].m_szDeviceName.decode('utf-8') if pDeviceInfo[i].m_szDeviceName else "Unnamed"
        except Exception as e:
            name = f"Unknown (Error: {str(e)})"
        print(f"Device {i}: ID = {pDeviceInfo[i].m_nDeviceID}, Name = {name}")
    device_id = pDeviceInfo[0].m_nDeviceID
    error = OpenDevice(pDeviceInfo[0])
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("OpenDevice", error)
        UnInitSDK()
        exit(1)
    print(f"Opened Device ID: {device_id}")
    return device_id, pDeviceInfo[0]

def get_point_cloud(device_id, width=640, height=480):
    frame_data = POINTER(SYFrameData)()
    error = GetLastFrameData(device_id, byref(frame_data))
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("GetLastFrameData", error)
        return None
    if not frame_data:
        print("GetLastFrameData returned null frame_data")
        return None

    obj_frame_data = frame_data.contents
    print(f"Frame count: {obj_frame_data.m_nFrameCount}")
    if obj_frame_data.m_nFrameCount <= 0:
        print("No frames available in frame_data")
        return None

    nPos = 0
    for i in range(obj_frame_data.m_nFrameCount):
        frame_info = obj_frame_data.m_pFrameInfo[i]
        print(f"Frame {i}: Type = {frame_info.m_frameType}")
        if frame_info.m_frameType == SYFrameTypeEnum.SYFRAMETYPE_DEPTH:
            nCount = width * height
            pDepth = (c_ushort * nCount)()
            ptr_void = c_void_p(obj_frame_data.m_pData + nPos)
            memmove(pDepth, cast(ptr_void, POINTER(c_ushort)), nCount * sizeof(c_ushort))
            point_cloud_data = (SYPointCloudData * nCount)()
            error = GetDepthPointCloud(device_id, width, height, pDepth, point_cloud_data)
            if error != SYErrorCode.SYERRORCODE_SUCCESS:
                PrintErrorCode("GetDepthPointCloud", error)
                return None
            points = np.empty((nCount, 3), dtype=np.float32)
            for j, p in enumerate(point_cloud_data):
                points[j] = [p.m_fltX, p.m_fltY, p.m_fltZ]
            print(f"Successfully retrieved {nCount} points")
            print(f"Point cloud z-range: {np.min(points[:, 2]):.3f} to {np.max(points[:, 2]):.3f} meters")
            return points
        nPos += frame_info.m_nFrameHeight * frame_info.m_nFrameWidth * sizeof(c_ushort)
    print("No depth frame found in frame_data")
    return None

def downsample_point_cloud(points, voxel_size=0.005):
    if points is None or len(points) == 0:
        return None
    min_bound = points.min(axis=0)
    voxel_index = ((points - min_bound) // voxel_size).astype(np.int32)
    unique_voxels, indices = np.unique(voxel_index, axis=0, return_index=True)
    return points[indices]

def detect_objects(point_cloud, eps=.4, min_samples=10, depth_range=(0.1, 10.0), fov=(-45, 45)):
    if point_cloud is None or len(point_cloud) == 0:
        return 0, []
    valid_mask = (point_cloud[:, 2] > depth_range[0]) & (point_cloud[:, 2] < depth_range[1])
    angles = np.arctan2(point_cloud[:, 0], point_cloud[:, 2]) * 180 / np.pi
    valid_mask &= (angles > fov[0]) & (angles < fov[1])
    valid_points = point_cloud[valid_mask]
    if len(valid_points) < min_samples:
        return 0, []
    downsampled_points = downsample_point_cloud(valid_points, voxel_size=0.002)
    if downsampled_points is None or len(downsampled_points) < min_samples:
        return 0, []
    clustering = DBSCAN(eps=eps, min_samples=min_samples, n_jobs=-1).fit(downsampled_points)
    labels = clustering.labels_
    num_objects = len(set(labels)) - (1 if -1 in labels else 0)
    centroids = [np.mean(downsampled_points[labels == label], axis=0) for label in set(labels) - {-1}]
    return num_objects, centroids

def frame_producer(device_id, frame_queue, stop_event):
    while not stop_event.is_set():
        point_cloud = get_point_cloud(device_id)
        if point_cloud is not None:
            try:
                frame_queue.put_nowait(point_cloud)
                print(f"Producer: Added point cloud with {len(point_cloud)} points")
            except queue.Full:
                print("Producer: Queue full, dropping frame")
        else:
            print("Producer: Failed to get point cloud")
        time.sleep(0.5)

def main():
    device_id, _ = initialize_device()
    error = SetFrameResolution(device_id, SYFrameTypeEnum.SYFRAMETYPE_DEPTH, SYResolutionEnum.SYRESOLUTION_640_480)
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("SetFrameResolution", error)
        CloseDevice(device_id)
        UnInitSDK()
        exit(1)
    stream_type = SYStreamTypeEnum.SYSTREAMTYPE_DEPTH
    error = StartStreaming(device_id, stream_type)
    if error != SYErrorCode.SYERRORCODE_SUCCESS:
        PrintErrorCode("StartStreaming", error)
        CloseDevice(device_id)
        UnInitSDK()
        exit(1)
    print("Streaming started successfully")
    time.sleep(5.0)
    print("Streaming stabilization period complete")

    point_cloud = get_point_cloud(device_id)
    if point_cloud is not None:
        print(f"Initial test frame retrieved with {len(point_cloud)} points")
    else:
        print("Failed to retrieve initial test frame")
        CloseDevice(device_id)
        UnInitSDK()
        exit(1)

    frame_queue = queue.Queue(maxsize=5)
    stop_event = threading.Event()
    producer_thread = threading.Thread(target=frame_producer, args=(device_id, frame_queue, stop_event))
    producer_thread.start()

    try:
        while True:
            start_time = time.perf_counter()
            try:
                point_cloud = frame_queue.get(timeout=1.0)
                num_objects, centroids = detect_objects(point_cloud)
                latency = (time.perf_counter() - start_time) * 1000
                print(f"Consumer: Detected {num_objects} objects | Latency: {latency:.2f}ms")
                if num_objects > 0:
                    for i, centroid in enumerate(centroids, 1):
                        x, y, z = centroid
                        print(f"  Object {i}: Location (x, y, z) = ({x:.3f}, {y:.3f}, {z:.3f}) meters")
                else:
                    print("  No objects detected")
                frame_queue.task_done()
            except queue.Empty:
                print("Consumer: Queue empty, waiting for producer...")
    except KeyboardInterrupt:
        stop_event.set()
        producer_thread.join()
    finally:
        StopStreaming(device_id)
        CloseDevice(device_id)
        UnInitSDK()

if __name__ == "__main__":
    main()