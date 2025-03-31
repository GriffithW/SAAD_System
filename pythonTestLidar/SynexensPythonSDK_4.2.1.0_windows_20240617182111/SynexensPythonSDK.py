import os
from typing import Any
from SYPythonDataDefine import *
import platform
from ctypes import c_uint, c_int, c_ushort, POINTER, c_bool

absPath = os.path.abspath(os.path.dirname(__file__))
print(absPath)

# if platform.system() == 'Windows':
#     dllLibrary = absPath + '/dll/SynexensSDK.dll'
# elif platform.system() == 'Linux':
dllLibrary = '/home/saad-sys/workspace/pythonTestLidar/SynexensPythonSDK_4.2.1.0_windows_20240617182111/so/libSynexensSDK.so'

dllLib = cdll.LoadLibrary(dllLibrary)
if dllLib == 0:
    print(" Could not open file DLL")


# 用于打印错误代码
def PrintErrorCode(func_name, errorCode):
    # 实现打印错误代码的逻辑
    print(f"Error occurred in {func_name}: {errorCode}")


# 获取SDK版本
# @ return 版本号
def GetSDKVersion() -> str:
    nSDKVersionLength = c_int(0)
    errorCodeGetSDKVersion: SYErrorCode = dllLib.GetSDKVersion(byref(nSDKVersionLength), None)
    if errorCodeGetSDKVersion == SYErrorCode.SYERRORCODE_SUCCESS:
        if nSDKVersionLength.value > 0:
            pSDKVersion = (c_char * nSDKVersionLength.value)()
            errorCodeGetSDKVersion: SYErrorCode = dllLib.GetSDKVersion(byref(nSDKVersionLength), pSDKVersion)
            if errorCodeGetSDKVersion == SYErrorCode.SYERRORCODE_SUCCESS:
                return pSDKVersion.value.decode('utf-8')
            else:
                PrintErrorCode("GetSDKVersion2", errorCodeGetSDKVersion)
                return str()


# 初始化SDK
# @ return 错误码
def InitSDK() -> SYErrorCode:
    return dllLib.InitSDK()


# 反初始化SDK
# @ return 错误码
def UnInitSDK() -> SYErrorCode:
    return dllLib.UnInitSDK()


# 查找设备
# @ param [in/out] nCount 设备数量
# @ param [in/out] pDevice 设备信息，由外部分配内存，pDevice传入nullptr时仅获取nCount
# @ return 错误码
def FindDevice(nCount: c_uint32, pDeviceInfo: POINTER(SYDeviceInfo)) -> SYErrorCode:
    dllLib.FindDevice.argtypes = [POINTER(c_int32), POINTER(SYDeviceInfo)]
    return dllLib.FindDevice(nCount, pDeviceInfo)


# 打开设备
# @ param [in] deviceInfo 设备信息
# @ return 错误码
def OpenDevice(pDevice: SYDeviceInfo) -> SYErrorCode:
    return dllLib.OpenDevice(byref(pDevice))


# 关闭设备
# @ param [in] nDeviceID 设备ID
# @ return 错误码
def CloseDevice(nDeviceID: c_uint32) -> SYErrorCode:
    return dllLib.CloseDevice(nDeviceID)


# 查询设备支持类型
# @ param [in] nDeviceID 设备ID
# @ param [in/out] supportTypeList返回支持的帧类型列表
# @ return 错误码
def QueryDeviceSupportFrameType(nDeviceID: c_uint32, supportTypeList: list) -> SYErrorCode:
    nSupportTypeCount = c_int(0)
    errorCodeQueryFrameType = dllLib.QueryDeviceSupportFrameType(nDeviceID, byref(nSupportTypeCount), None)
    if errorCodeQueryFrameType == SYErrorCode.SYERRORCODE_SUCCESS and nSupportTypeCount.value > 0:
        pSupportType = (SYSupportTypeEnum * nSupportTypeCount.value)()
        errorCodeQueryFrameType = dllLib.QueryDeviceSupportFrameType(nDeviceID, byref(nSupportTypeCount), pSupportType)
        if errorCodeQueryFrameType == SYErrorCode.SYERRORCODE_SUCCESS and nSupportTypeCount.value > 0:
            for i in range(nSupportTypeCount.value):
                supportTypeList.append(pSupportType[i].value)
            return SYErrorCode.SYERRORCODE_SUCCESS
        else:
            return SYErrorCode.SYERRORCODE_UNKOWNFRAMETYPE
    else:
        return SYErrorCode.SYERRORCODE_UNKOWNFRAMETYPE


# 查询设备支持类型
# @ param [in] nDeviceID 设备ID
# @ param [in/out] sdkSupportType 帧类型
# @ param [in/out] resolutionList 对应帧类型的可用分辨率列表
# @ return 错误码
def QueryDeviceSupportResolution(nDeviceID: c_uint32, sdkSupportType: SYSupportTypeEnum, resolutionList: list) -> SYErrorCode:
    nResolutionCount = c_int(0)
    errorCodeQueryResolution = dllLib.QueryDeviceSupportResolution(nDeviceID, sdkSupportType, byref(nResolutionCount), None)
    if errorCodeQueryResolution == SYErrorCode.SYERRORCODE_SUCCESS and nResolutionCount.value > 0:
        pResolution = (SYResolutionEnum * nResolutionCount.value)()
        errorCodeQueryResolution = dllLib.QueryDeviceSupportResolution(nDeviceID, sdkSupportType, byref(nResolutionCount), pResolution)
        if errorCodeQueryResolution == SYErrorCode.SYERRORCODE_SUCCESS and nResolutionCount.value > 0:
            for i in range(nResolutionCount.value):
                resolutionList.append(pResolution[i].value)
            return SYErrorCode.SYERRORCODE_SUCCESS
        else:
            return SYErrorCode.SYERRORCODE_UNKOWNRESOLUTION
    return SYErrorCode.SYERRORCODE_UNKOWNRESOLUTION


# 查询设备当前的数据流类型
# @ param [in] nDeviceID 设备ID
# @ return 当前流类型
def GetCurrentStreamType(nDeviceID: c_uint32) -> SYStreamTypeEnum:
    return dllLib.GetCurrentStreamType(nDeviceID)


# 启动数据流
# @ param [in] nDeviceID 设备ID
# @ param [in] streamType 数据流类型
# @ return 错误码
def StartStreaming(nDeviceID: c_uint32, streamType: SYStreamTypeEnum) -> SYErrorCode:
    return dllLib.StartStreaming(nDeviceID, streamType)


# 停止数据流
# @ param [in] nDeviceID 设备ID
# @ return 错误码
def StopStreaming(nDeviceID: c_uint32) -> SYErrorCode:
    return dllLib.StopStreaming(nDeviceID)


# 切换数据流
# @ param [in] nDeviceID 设备ID
# @ param [in] streamType 数据流类型
# @ return 错误码
def ChangeStreaming(nDeviceID: c_uint32, streamType: SYStreamTypeEnum) -> SYErrorCode:
    return dllLib.ChangeStreaming(nDeviceID, streamType)


# 设置分辨率（如果已启动数据流，内部会执行关流->设置分辨率->重新开流的操作流程）
# @ param [in] nDeviceID 设备ID
# @ param [in] frameType 帧类型
# @ param [in] resolution 帧分辨率
# @ return 错误码
def SetFrameResolution(nDeviceID: c_uint32, frameType: SYFrameTypeEnum, resolution: SYResolutionEnum) -> SYErrorCode:
    return dllLib.SetFrameResolution(nDeviceID, frameType, resolution)


# 获取设备帧分辨率
# @ param [in] nDeviceID 设备ID
# @ param [in] frameType 帧类型
# @ param [out] resolution 帧分辨率
# @ return 错误码
def GetFrameResolution(nDeviceID: c_uint32, frameType: SYFrameTypeEnum, resolution: SYResolutionEnum) -> SYErrorCode:
    return dllLib.GetFrameResolution(nDeviceID, frameType, byref(resolution))


# 获取滤波开启状态
# @ param [in] nDeviceID 设备ID
# @ param [out] bFilter 滤波开启状态，true-已开启滤波，false-未开启滤波
# @ return 错误码
def GetFilter(nDeviceID: c_uint32, bFilter: c_bool) -> SYErrorCode:
    return dllLib.GetFilter(nDeviceID, byref(bFilter))


# 开启/关闭滤波
# @ param [in] nDeviceID 设备ID
# @ param [in] bFilter 滤波开关，true-开启滤波，false-关闭滤波
# @ return 错误码
def SetFilter(nDeviceID: c_uint32, bFilter: c_bool) -> SYErrorCode:
    return dllLib.SetFilter(nDeviceID, bFilter)


# 获取滤波列表
# @ param [in] nDeviceID 设备ID
# @ param [in/out] nCount 滤波列表长度
# @ param [in/out] pFilterType 滤波列表
# @ return 错误码
def GetFilterList(nDeviceID: c_uint32, nCount: c_int, pFilterType: POINTER(SYFilterTypeEnum)) -> SYErrorCode:
    return dllLib.GetFilterList(nDeviceID, byref(nCount), pFilterType)


# 设置默认滤波
# @ param [in] nDeviceID 设备ID
# @ return 错误码
def SetDefaultFilter(nDeviceID: c_uint32) -> SYErrorCode:
    return dllLib.SetDefaultFilter(nDeviceID)


# 增加滤波
# @ param [in] nDeviceID 设备ID
# @ param [in] filterType 滤波类型
# @ return 错误码
def AddFilter(nDeviceID: c_uint32, filterType: SYFilterTypeEnum) -> SYErrorCode:
    return dllLib.AddFilter(nDeviceID, filterType)


# 移除滤波
# @ param [in] nDeviceID 设备ID
# @ param [in] nIndex 滤波列表中的索引
# @ return 错误码
def DeleteFilter(nDeviceID: c_uint32, nIndex: c_int) -> SYErrorCode:
    return dllLib.DeleteFilter(nDeviceID, nIndex)


# 清除滤波
# @ param [in] nDeviceID 设备ID
# @ return 错误码
def ClearFilter(nDeviceID: c_uint32) -> SYErrorCode:
    return dllLib.ClearFilter(nDeviceID)


# 设置滤波参数
# @ param [in] nDeviceID 设备ID
# @ param [in] filterType 滤波类型
# @ param [in] nParamCount 滤波参数个数
# @ param [in] pFilterParam 滤波参数
# @ return 错误码
def SetFilterParam(nDeviceID: c_uint32, filterType: SYFilterTypeEnum, nParamCount: c_int, pFilterParam: POINTER(c_float)) -> SYErrorCode:
    return dllLib.SetFilterParam(nDeviceID, filterType, nParamCount, pFilterParam)


# 获取滤波参数
# @ param [in] nDeviceID 设备ID
# @ param [in] filterType 滤波类型
# @ param [in/out] nParamCount 滤波参数个数
# @ param [in/out] pFilterParam 滤波参数
# @ return 错误码
def GetFilterParam(nDeviceID: c_uint32, filterType: SYFilterTypeEnum, nParamCount: c_int, pFilterParam: POINTER(c_float)) -> SYErrorCode:
    return dllLib.GetFilterParam(nDeviceID, filterType, byref(nParamCount), pFilterParam)


# 获取水平镜像状态
# @ param [in] nDeviceID 设备ID
# @ param [out] bMirror 水平镜像状态，true-已开启水平镜像，false-未开启水平镜像
# @ return 错误码
def GetMirror(nDeviceID: c_uint32, bMirror: c_bool) -> SYErrorCode:
    return dllLib.GetMirror(nDeviceID, byref(bMirror))


# 开启/关闭水平镜像
# @ param [in] nDeviceID 设备ID
# @ param [in] bMirror 水平镜像开关，true-开启水平镜像，false-关闭水平镜像
# @ return 错误码
def SetMirror(nDeviceID: c_uint32, bMirror: c_bool) -> SYErrorCode:
    return dllLib.SetMirror(nDeviceID, bMirror)


# 获取垂直翻转状态
# @ param [in] nDeviceID 设备ID
# @ param [out] bFlip 垂直翻转状态，true-已开启垂直翻转，false-未开启垂直翻转
# @ return 错误码
def GetFlip(nDeviceID: c_uint32, bFilp: c_bool) -> SYErrorCode:
    return dllLib.GetFlip(nDeviceID, byref(bFilp))


# 开启/关闭垂直翻转
# @ param [in] nDeviceID 设备ID
# @ param [in] bFlip 垂直翻转开关，true-开启垂直翻转，false-关闭垂直翻转
# @ return 错误码
def SetFlip(nDeviceID: c_uint32, bFilp: c_bool) -> SYErrorCode:
    return dllLib.SetFlip(nDeviceID, bFilp)


# 获取积分时间
# @ param [in] nDeviceID 设备ID
# @ param [out] nIntegralTime 积分时间
# @ return 错误码
def GetIntegralTime(nDeviceID: c_uint32, nIntegralTime: c_int) -> SYErrorCode:
    return dllLib.GetIntegralTime(nDeviceID, byref(nIntegralTime))


# 设置积分时间
# @ param [in] nDeviceID 设备ID
# @ param [in] nIntegralTime 积分时间
# @ return 错误码
def SetIntegralTime(nDeviceID: c_uint32, nIntegralTime: c_int) -> SYErrorCode:
    return dllLib.SetIntegralTime(nDeviceID, nIntegralTime)


# 获取积分时间调节范围
# @ param [in] nDeviceID 设备ID
# @ param [in] depthResolution depth分辨率
# @ param [out] nMin 积分时间最小值
# @ param [out] nMax 积分时间最大值
# @ return 错误码
def GetIntegralTimeRange(nDeviceID: c_uint32, depthResolution: SYResolutionEnum, nMin: c_int, nMax: c_int) -> SYErrorCode:
    return dllLib.GetIntegralTimeRange(nDeviceID, depthResolution, byref(nMin), byref(nMax))


# 获取测距量程
# param [in] nDeviceID 设备ID
# param [out] nMin 量程最小值
# param [out] nMax 量程最大值
# return 错误码
def GetDistanceMeasureRange(nDeviceID: c_uint32, nMin: c_int, nMax: c_int) -> SYErrorCode:
    return dllLib.GetDistanceMeasureRange(nDeviceID, byref(nMin), byref(nMax))


# 获取用户测距范围
# param [in] nDeviceID 设备ID
# param [out] nMin 测距范围最小值
# param [out] nMax 测距范围最大值
# return 错误码
def GetDistanceUserRange(nDeviceID: c_uint32, nMin: c_int, nMax: c_int) -> SYErrorCode:
    return dllLib.GetDistanceUserRange(nDeviceID, byref(nMin), byref(nMax))


# 设置用户测距范围
# param [in] nDeviceID 设备ID
# param [in] nMin 测距范围最小值
# param [in] nMax 测距范围最大值
# return 错误码
def SetDistanceUserRange(nDeviceID: c_uint32, nMin: c_int, nMax: c_int) -> SYErrorCode:
    return dllLib.SetDistanceUserRange(nDeviceID, nMin, nMax)


# 读取设备sn号
# param [in] nDeviceID 设备ID
# return 错误码
def GetDeviceSN(nDeviceID: c_uint32) -> None | str:
    nStringLength = c_int(0)
    errorCodeGetSN: SYErrorCode = dllLib.GetDeviceSN(nDeviceID, byref(nStringLength), None)
    if errorCodeGetSN == SYErrorCode.SYERRORCODE_SUCCESS:
        if nStringLength.value > 0:
            pSerialNumber = (c_char * nStringLength.value)()
            errorCodeGetSN: SYErrorCode = dllLib.GetDeviceSN(nDeviceID, byref(nStringLength), pSerialNumber)
            if errorCodeGetSN == SYErrorCode.SYERRORCODE_SUCCESS:
                return pSerialNumber.value.decode('utf-8')
            else:
                PrintErrorCode("GetDeviceSN", errorCodeGetSN)
                return None
        else:
            errorCodeGetSN = SYErrorCode.SYERRORCODE_STRINGLENGTHOUTRANGE
            PrintErrorCode("GetDeviceSN", errorCodeGetSN)
            return None


# 写入设备sn号
# param [in] nDeviceID 设备ID
# param [in] pstrSN 设备sn号字符串指针
# return 错误码
def SetDeviceSN(nDeviceID: c_uint32, deviceSN: str) -> SYErrorCode:
    return dllLib.SetDeviceSN(nDeviceID, deviceSN.encode('utf-8'))


# 读取设备固件版本号
# param [in] nDeviceID 设备ID
# return 设备固件版本
def GetDeviceHWVersion(nDeviceID: c_uint32) -> str | None:
    nStringLength = c_int(0)
    errorCodeGetHWVersion = dllLib.GetDeviceHWVersion(nDeviceID, byref(nStringLength), None)
    if errorCodeGetHWVersion == SYErrorCode.SYERRORCODE_SUCCESS:
        if nStringLength.value > 0:
            pStringFWVersion = (c_char * nStringLength.value)()
            errorCodeGetHWVersion = dllLib.GetDeviceHWVersion(nDeviceID, byref(nStringLength), pStringFWVersion)
            if errorCodeGetHWVersion == SYErrorCode.SYERRORCODE_SUCCESS:
                return pStringFWVersion.value.decode('utf-8')
            else:
                errorCodeRet = SYErrorCode.SYERRORCODE_UNKOWNSN
                PrintErrorCode("GetDeviceHWVersion", errorCodeRet)
                return None
        else:
            return None
    else:
        return None


# 获取深度对应伪彩色
# param [in] nDeviceID 设备ID
# param [in] nCount 数据量(内存空间pDepth需要nCount*2字节，pColor需要nCount*3字节)
# param [in] pDepth 深度数据
# param [in/out] pColor 深度对应伪彩色(24位RGB格式)
# return 错误码
def GetDepthColor(nDeviceID: c_uint32, nCount: c_int, pDepth: POINTER(c_ushort), pColor: POINTER(c_ubyte)):
    dllLib.GetDepthColor.argtypes = [c_uint32, c_int, POINTER(c_ushort), POINTER(c_ubyte)]
    return dllLib.GetDepthColor(nDeviceID, nCount, pDepth, pColor)


# 获取深度对应点云数据
# param [in] nDeviceID 设备ID
# param [in] nWidth 宽度
# param [in] nHeight 高度
# param [in] pDepth 深度数据
# param [in/out] pPointCloud 深度对应点云数据,由外部分配内存
# param [in] bUndistort 裁剪标志，true-裁剪 false-不裁剪
# # return 错误码
# def GetDepthPointCloud(nDeviceID: int, nWidth: int, nHeight: int, pDepth: POINTER(c_ushort),
#                        pPointCloud: POINTER(SYPointCloudData), bUndistort: bool = False) -> SYErrorCode:
#     dllLib.GetDepthPointCloud.argtypes = [c_uint, c_int, c_int, POINTER(c_ushort), POINTER(SYPointCloudData)]
#     return dllLib.GetDepthPointCloud(nDeviceID, c_int(nWidth), c_int(nHeight), pDepth, pPointCloud, c_bool(bUndistort))
def GetDepthPointCloud(nDeviceID: int, nWidth: int, nHeight: int, pDepth: POINTER(c_ushort),
                       pPointCloud: POINTER(SYPointCloudData), bUndistort: bool = False) -> SYErrorCode:
    dllLib.GetDepthPointCloud.argtypes = [c_uint, c_int, c_int, POINTER(c_ushort), POINTER(SYPointCloudData), c_bool]
    return dllLib.GetDepthPointCloud(nDeviceID, c_int(nWidth), c_int(nHeight), pDepth, pPointCloud, c_bool(bUndistort))

# 获取RGBD
# param [in] nDeviceID 设备ID
# param [in] nSourceDepthWidth 源深度数据宽度
# param [in] nSourceDepthHeight 源深度数据高度
# param [in] pSourceDepth 源深度数据
# param [in] nSourceRGBWidth 源RGB数据宽度
# param [in] nSourceRGBHeight 源RGB数据高度
# param [in] pSourceRGB 源RGB数据
# param [in] nTargetWidth RGBD数据宽度
# param [in] nTargetHeight RGBD数据高度
# param [in/out] pTargetDepth RGBD中的深度数据,由外部分配内存,数据长度与源RGB长度一致
# param [in/out] pTargetRGB RGBD中的RGB数据,由外部分配内存,数据长度与源RGB长度一致
# return 错误码
def GetRGBD(nDeviceID: c_uint32, nSourceDepthWidth: int, nSourceDepthHeight: int, pSourceDepth: POINTER(c_ushort),
            nSourceRGBWidth: int, nSourceRGBHeight: int, pSourceRGB: POINTER(c_ubyte), nTargetWidth: int,
            nTargetHeight: int, pTargetDepth: POINTER(c_ushort),
            pTargetRGB: POINTER(c_ubyte)) -> SYErrorCode:
    dllLib.GetRGBD.argtypes = [c_uint32, c_int, c_int, POINTER(c_ushort), c_int, c_int, POINTER(c_ubyte), c_int, c_int,
                               POINTER(c_ushort), POINTER(c_ubyte)]
    return dllLib.GetRGBD(nDeviceID, c_int(nSourceDepthWidth), c_int(nSourceDepthHeight), pSourceDepth,
                          c_int(nSourceRGBWidth), c_int(nSourceRGBHeight), pSourceRGB, c_int(nTargetWidth),
                          c_int(nTargetHeight), pTargetDepth, pTargetRGB)


# 获取最新一帧数据
# param [in] nDeviceID 设备ID
# param [in/out] pFrameData 最后一帧数据
# return 错误码
def GetLastFrameData(nDeviceID: c_uint32, pFrameData: POINTER(SYFrameData)) -> SYErrorCode:
    return dllLib.GetLastFrameData(nDeviceID, pFrameData)


# 去畸变
# param [in] nDeviceID 设备ID
# param [in] pSource  待去畸变数据指针
# param [in] nWidth 图像宽度
# param [in] nHeight 图像高度
# param [in] bDepth 是否是深度数据/RGB数据
# param [out] pTarget  去畸变结果数据指针，由外部分配内存,数据长度与待去畸变数据指针长度一致
def Undistort(nDeviceID: c_uint32, pSource: POINTER(c_ushort), nWidth: c_int, nHeight: c_int, bDepth: c_bool, pTarget: POINTER(c_ushort)) -> SYErrorCode:
    return dllLib.Undistort(nDeviceID, pSource, nWidth, nHeight, bDepth, pTarget)


# 获取相机参数
# @ param [in] nDeviceID 设备ID
# @ param [in] resolution  分辨率
# @ param [in/out] intrinsics 相机参数
def GetIntric(nDeviceID: c_uint32, resolution: SYResolutionEnum, intrinsics: SYIntrinsics):
    return dllLib.GetIntric(nDeviceID, resolution, byref(intrinsics))


# 获取拖影滤波开启状态
# @param[in] nDeviceID 设备ID
# @param[out] bFilter 拖影滤波开启状态，true - 已开启滤波，false - 未开启滤波
# @return 错误码
def GetTrailFilter(nDeviceID: c_uint32, bFilter: c_bool) -> SYErrorCode:
    return dllLib.GetTrailFilter(nDeviceID, byref(bFilter))


# 设置拖影滤波开启状态
# @param[in] nDeviceID 设备ID
# @param[in] bFilter 拖影滤波开启状态，true - 已开启滤波，false - 未开启滤波
# @return 错误码
def SetTrailFilter(nDeviceID: c_uint32, bFilter: c_bool) -> SYErrorCode:
    return dllLib.SetTrailFilter(nDeviceID, bFilter)
