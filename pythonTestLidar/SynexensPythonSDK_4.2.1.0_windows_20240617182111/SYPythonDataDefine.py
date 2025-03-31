from ctypes import *


class SYErrorCode(c_int):
    SYERRORCODE_SUCCESS = 0  # 成功
    SYERRORCODE_FAILED = 1,  # 失败
    SYERRORCODE_DEVICENOTEXIST = 2,  # 设备不存在
    SYERRORCODE_DEVICENOTOPENED = 3,  # 设备未打开
    SYERRORCODE_UNKOWNRESOLUTION = 4,  # 不支持的分辨率
    SYERRORCODE_DEVICEHANDLEEMPTY = 5,  # 设备指针句柄为空
    SYERRORCODE_SETOUTPUTFORMATFAILED = 6,  # 设备输出数据格式设置失败
    SYERRORCODE_GETSTREAMCTRLFAILED = 7,  # 获取视频流控制指针失败
    SYERRORCODE_STARTSTREAMINGFAILED = 8,  # 启动视频流失败
    SYERRORCODE_COMMUNICATEOBJECTEMPTY = 9,  # 通讯指针为空
    SYERRORCODE_UNKOWNSN = 10,  # 无效的SN号
    SYERRORCODE_STRINGLENGTHOUTRANGE = 11,  # 字符串长度溢出
    SYERRORCODE_UNKOWNFRAMETYPE = 12,  # 无效帧类型
    SYERRORCODE_UNKOWNDEVICETYPE = 13,  # 无效设备类型
    SYERRORCODE_DEVICEOBJECTEMPTY = 14,  # 设备对象指针为空
    SYERRORCODE_OBSERVEREMPTY = 15,  # 通知对象指针为空
    SYERRORCODE_OBSERVERNOTFOUND = 16,  # 通知对象未找到
    SYERRORCODE_COUNTOUTRANGE = 17,  # 数量溢出
    SYERRORCODE_UVCINITFAILED = 18,  # UVC初始化失败
    SYERRORCODE_UVCFINDDEVICEFAILED = 19,  # UVC查找设备失败
    SYERRORCODE_NOFRAME = 20,  # 无数据帧
    SYERRORCODE_GETAPPFOLDERPATHFAILED = 21,  # 程序路径获取失败
    SYERRORCODE_NOSTREAMING = 22,  # 视频流未启动
    SYERRORCODE_RECONSTRUCTIONEMPTY = 23,  # 算法指针为空
    SYERRORCODE_STREAMINGEXIST = 24,  # 视频流已开启
    SYERRORCODE_UNKOWNSTREAMTYPE = 25,  # 未知的流类型
    SYERRORCODE_DATABUFFEREMPTY = 26,  # 数据指针为空


class SYDeviceTypeEnum(c_int):
    SYDEVICETYPE_NULL = 0  # 无效
    SYDEVICETYPE_CS30_DUAL = 1  # CS30双频
    SYDEVICETYPE_CS30_SINGLE = 2  # CS30单频
    SYDEVICETYPE_CS20_DUAL = 3  # CS20双频
    SYDEVICETYPE_CS20_SINGLE = 4  # CS20单频
    SYDEVICETYPE_CS20_P = 5  # CS20_P
    SYDEVICETYPE_CS40 = 6  # CS40
    SYDEVICETYPE_CS40PRO = 7


class SYStreamTypeEnum(c_int):
    SYSTREAMTYPE_NULL = 0  # 无效
    SYSTREAMTYPE_RAW = 1  # RAW
    SYSTREAMTYPE_DEPTH = 2  # 深度
    SYSTREAMTYPE_RGB = 3  # RGB
    SYSTREAMTYPE_DEPTHIR = 4  # 深度+IR
    SYSTREAMTYPE_DEPTHRGB = 5  # 深度+RGB
    SYSTREAMTYPE_DEPTHIRRGB = 6  # 深度+IR+RGB
    SYSTREAMTYPE_RGBD = 7  # RGBD(mapping后的深度+RGB)
    SYSTREAMTYPE_RAWRGB = 8  # RAW_RGB


class SYFrameTypeEnum(c_int):
    SYFRAMETYPE_NULL = 0  # 无效
    SYFRAMETYPE_RAW = 1  # RAW
    SYFRAMETYPE_DEPTH = 2  # 深度
    SYFRAMETYPE_IR = 3  # IR
    SYFRAMETYPE_RGB = 4  # RGB


class SYSupportTypeEnum(c_int):
    SYSUPPORTTYPE_NULL = 0
    SYSUPPORTTYPE_DEPTH = 1
    SYSUPPORTTYPE_RGB = 2
    SYSUPPORTTYPE_RGBD = 3


class SYResolutionEnum(c_int):
    SYRESOLUTION_NULL = 0
    SYRESOLUTION_320_240 = 1
    SYRESOLUTION_640_480 = 2
    SYRESOLUTION_960_540 = 3
    SYRESOLUTION_1920_1080 = 4
    SYRESOLUTION_1600_1200 = 5


class SYFilterTypeEnum(c_int):
    SYFILTERTYPE_NULL = 0  # 无效
    SYFILTERTYPE_MEDIAN = 1  # 中值
    SYFILTERTYPE_AMPLITITUD = 2  # 幅值
    SYFILTERTYPE_EDGE = 3  # 边界
    SYFILTERTYPE_SPECKLE = 4  # 斑点
    SYFILTERTYPE_OKADA = 5  # 大金阈值
    SYFILTERTYPE_EDGE_MAD = 6  # 边界2
    SYFILTERTYPE_GAUSS = 7  # 高斯
    SYFILTERTYPE_EXTRA = 8  # 备用
    SYFILTERTYPE_EXTRA2 = 9  # 备用2


class SYDeviceInfo(Structure):
    _pack_ = 1
    _fields_ = [
        ('m_nDeviceID', c_uint),
        ('m_deviceType', SYDeviceTypeEnum),
        ('m_nUsbBus', c_uint),
        ('m_nUsbPorts', c_uint * 7),
        ('m_nUsbPortsNumber', c_uint),
        ('m_nUsbDeviceAddress', c_uint),
    ]


class SYFrameInfo(Structure):
    _pack_ = 1
    _fields_ = [
        ('m_frameType', c_int),
        ('m_nFrameHeight', c_int),
        ('m_nFrameWidth', c_int)
    ]


class SYFrameData(Structure):
    _pack_ = 1
    _fields_ = [
        ('m_nFrameCount', c_int),
        ('m_pFrameInfo', POINTER(SYFrameInfo)),
        ('m_pData', c_void_p),
        ('m_nBuffferLength', c_int),
    ]


class SYPointCloudData(Structure):
    _pack_ = 1
    _fields_ = [
        ('m_fltX', c_float),
        ('m_fltY', c_float),
        ('m_fltZ', c_float)
    ]


class SYIntrinsics(Structure):
    _pack_ = 1
    _fields_ = [
        ('m_fltFOV', c_float * 2),
        ('m_fltCoeffs', c_float * 5),
        ('m_fltFocalDistanceX', c_float),
        ('m_fltFocalDistanceY', c_float),
        ('m_fltCenterPointX', c_float),
        ('m_fltCenterPointY', c_float),
        ('m_nWidth', c_int),
        ('m_nHeight', c_int),
    ]
