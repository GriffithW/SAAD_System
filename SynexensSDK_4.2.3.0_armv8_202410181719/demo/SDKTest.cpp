// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <pthread.h>
// #include <unistd.h>
// #include <math.h>
// #include <signal.h>
// #include "SYSDKInterface.h"  // Main SDK header

// #define WIDTH 640
// #define HEIGHT 480
// #define VOXEL_SIZE 0.05  // 5 cm
// #define EPS 0.2          // 20 cm
// #define MIN_SAMPLES 10

// // Point structure
// typedef struct {
//     float x, y, z;
// } Point;

// // Object structure
// typedef struct {
//     int cluster_id;
//     float centroid[3];
//     int size;
// } Object;

// // Shared buffer for point cloud
// typedef struct {
//     Point* points;
//     int count;
//     int ready;
//     pthread_mutex_t mutex;
//     pthread_cond_t cond;
// } Buffer;

// // Global variables
// static volatile int running = 1;
// Buffer buffer = {NULL, 0, 0, PTHREAD_MUTEX_INITIALIZER, PTHREAD_COND_INITIALIZER};

// // Signal handler for clean exit
// void signal_handler(int sig) {
//     running = 0;
// }

// // Error printing
// void print_error(const char* func_name, Synexens::SYErrorCode error) {
//     printf("Error in %s: %d\n", func_name, error);
// }

// // Initialize device
// Synexens::SYDeviceHandle initialize_device() {
//     Synexens::SYErrorCode error = Synexens::InitSDK();
//     if (error != Synexens::SYERRORCODE_SUCCESS) {
//         print_error("InitSDK", error);
//         exit(1);
//     }

//     int32_t device_count = 0;
//     error = Synexens::FindDevice(device_count, NULL);
//     if (error != Synexens::SYERRORCODE_SUCCESS || device_count <= 0) {
//         print_error("FindDevice", error);
//         Synexens::UnInitSDK();
//         exit(1);
//     }

//     Synexens::SYDeviceInfo* devices = (Synexens::SYDeviceInfo*)malloc(device_count * sizeof(Synexens::SYDeviceInfo));
//     error = Synexens::FindDevice(device_count, devices);
//     if (error != Synexens::SYERRORCODE_SUCCESS) {
//         print_error("FindDevice - Fetch Info", error);
//         free(devices);
//         Synexens::UnInitSDK();
//         exit(1);
//     }

//     Synexens::SYDeviceHandle device = devices[0];
//     for (int i = 0; i < 3; i++) {
//         error = Synexens::OpenDevice(device);
//         if (error == Synexens::SYERRORCODE_SUCCESS) break;
//         print_error("OpenDevice", error);
//         sleep(1);
//     }
//     if (error != Synexens::SYERRORCODE_SUCCESS) {
//         free(devices);
//         Synexens::UnInitSDK();
//         exit(1);
//     }

//     printf("Opened Device ID: %u\n", device.m_nDeviceID);
//     free(devices);
//     return device;
// }

// // Capture point cloud
// Point* get_point_cloud(Synexens::SYDeviceHandle device, int* count) {
//     Synexens::SYFrameData* frame_data = NULL;
//     Synexens::SYErrorCode error;
//     for (int i = 0; i < 3; i++) {
//         error = Synexens::GetLastFrameData(device.m_nDeviceID, frame_data);
//         if (error == Synexens::SYERRORCODE_SUCCESS && frame_data) break;
//         print_error("GetLastFrameData", error);
//         usleep(500000);  // 0.5s retry
//     }
//     if (error != Synexens::SYERRORCODE_SUCCESS || !frame_data) return NULL;

//     if (frame_data->m_nFrameCount <= 0) {
//         free(frame_data);
//         return NULL;
//     }

//     int pos = 0;
//     for (int i = 0; i < frame_data->m_nFrameCount; i++) {
//         Synexens::SYFrameInfo frame_info = frame_data->m_pFrameInfo[i];
//         if (frame_info.m_frameType == Synexens::SYFRAMETYPE_DEPTH) {
//             int n_count = frame_info.m_nFrameHeight * frame_info.m_nFrameWidth;
//             uint16_t* depth = (uint16_t*)malloc(n_count * sizeof(uint16_t));
//             memcpy(depth, (uint8_t*)frame_data->m_pData + pos, n_count * sizeof(uint16_t));

//             Synexens::SYPointCloudData* cloud_data = (Synexens::SYPointCloudData*)malloc(n_count * sizeof(Synexens::SYPointCloudData));
//             error = Synexens::GetDepthPointCloud(device.m_nDeviceID, frame_info.m_nFrameWidth, 
//                                                  frame_info.m_nFrameHeight, depth, cloud_data, false);
//             free(depth);
//             if (error != Synexens::SYERRORCODE_SUCCESS) {
//                 print_error("GetDepthPointCloud", error);
//                 free(cloud_data);
//                 free(frame_data);
//                 return NULL;
//             }

//             Point* points = (Point*)malloc(n_count * sizeof(Point));
//             for (int j = 0; j < n_count; j++) {
//                 points[j].x = cloud_data[j].m_fltX;
//                 points[j].y = cloud_data[j].m_fltY;
//                 points[j].z = cloud_data[j].m_fltZ;
//             }
//             free(cloud_data);
//             *count = n_count;
//             free(frame_data);
//             return points;
//         }
//         pos += frame_info.m_nFrameHeight * frame_info.m_nFrameWidth * sizeof(uint16_t);
//     }
//     free(frame_data);
//     return NULL;
// }

// // Downsample point cloud
// Point* downsample_points(Point* points, int count, int* new_count) {
//     if (!points || count == 0) return NULL;

//     Point* downsampled = (Point*)malloc(count * sizeof(Point));
//     int* voxel_map = (int*)calloc(count, sizeof(int));
//     int ds_count = 0;

//     float min[3] = {points[0].x, points[0].y, points[0].z};
//     for (int i = 1; i < count; i++) {
//         if (points[i].x < min[0]) min[0] = points[i].x;
//         if (points[i].y < min[1]) min[1] = points[i].y;
//         if (points[i].z < min[2]) min[2] = points[i].z;
//     }

//     for (int i = 0; i < count; i++) {
//         if (points[i].x == 0 && points[i].y == 0 && points[i].z == 0) continue;
//         int vx = (int)((points[i].x - min[0]) / VOXEL_SIZE);
//         int vy = (int)((points[i].y - min[1]) / VOXEL_SIZE);
//         int vz = (int)((points[i].z - min[2]) / VOXEL_SIZE);
//         int voxel_idx = vx + vy * 1000 + vz * 1000000;

//         if (!voxel_map[i]) {
//             downsampled[ds_count] = points[i];
//             voxel_map[i] = 1;
//             ds_count++;
//         }
//     }

//     downsampled = (Point*)realloc(downsampled, ds_count * sizeof(Point));
//     free(voxel_map);
//     *new_count = ds_count;
//     return downsampled;
// }

// // Simplified DBSCAN
// int detect_objects(Point* points, int count, Object** objects) {
//     if (!points || count < MIN_SAMPLES) return 0;

//     int* labels = (int*)calloc(count, sizeof(int));
//     int cluster_id = 0;

//     for (int i = 0; i < count; i++) {
//         if (labels[i] != 0) continue;

//         int neighbors = 0;
//         for (int j = 0; j < count; j++) {
//             float dx = points[i].x - points[j].x;
//             float dy = points[i].y - points[j].y;
//             float dz = points[i].z - points[j].z;
//             if (sqrtf(dx * dx + dy * dy + dz * dz) < EPS) neighbors++;
//         }

//         if (neighbors >= MIN_SAMPLES) {
//             cluster_id++;
//             labels[i] = cluster_id;
//             for (int j = 0; j < count; j++) {
//                 if (labels[j] == 0) {
//                     float dist = sqrtf(powf(points[i].x - points[j].x, 2) +
//                                       powf(points[i].y - points[j].y, 2) +
//                                       powf(points[i].z - points[j].z, 2));
//                     if (dist < EPS) labels[j] = cluster_id;
//                 }
//             }
//         } else {
//             labels[i] = -1;
//         }
//     }

//     int num_objects = cluster_id;
//     *objects = (Object*)malloc(num_objects * sizeof(Object));
//     for (int cid = 1; cid <= num_objects; cid++) {
//         float centroid[3] = {0, 0, 0};
//         int size = 0;
//         for (int i = 0; i < count; i++) {
//             if (labels[i] == cid) {
//                 centroid[0] += points[i].x;
//                 centroid[1] += points[i].y;
//                 centroid[2] += points[i].z;
//                 size++;
//             }
//         }
//         centroid[0] /= size; centroid[1] /= size; centroid[2] /= size;
//         (*objects)[cid-1].cluster_id = cid;
//         (*objects)[cid-1].centroid[0] = centroid[0];
//         (*objects)[cid-1].centroid[1] = centroid[1];
//         (*objects)[cid-1].centroid[2] = centroid[2];
//         (*objects)[cid-1].size = size;
//     }

//     free(labels);
//     return num_objects;
// }

// // Capture thread
// void* capture_thread(void* arg) {
//     Synexens::SYDeviceHandle* device = (Synexens::SYDeviceHandle*)arg;
//     while (running) {
//         int count;
//         Point* points = get_point_cloud(*device, &count);
//         if (!points) {
//             usleep(10000);  // 10ms retry
//             continue;
//         }

//         pthread_mutex_lock(&buffer.mutex);
//         if (buffer.points) free(buffer.points);
//         buffer.points = points;
//         buffer.count = count;
//         buffer.ready = 1;
//         pthread_cond_signal(&buffer.cond);
//         pthread_mutex_unlock(&buffer.mutex);
//     }
//     return NULL;
// }

// // Process thread
// void* process_thread(void* arg) {
//     int iteration = 0;
//     while (running) {
//         pthread_mutex_lock(&buffer.mutex);
//         while (!buffer.ready && running) pthread_cond_wait(&buffer.cond, &buffer.mutex);

//         if (!running) {
//             pthread_mutex_unlock(&buffer.mutex);
//             break;
//         }

//         Point* points = buffer.points;
//         int count = buffer.count;
//         buffer.points = NULL;
//         buffer.ready = 0;
//         pthread_mutex_unlock(&buffer.mutex);

//         if (!points) continue;

//         printf("\nIteration %d\n", iteration++);
//         int ds_count;
//         Point* downsampled = downsample_points(points, count, &ds_count);
//         free(points);

//         Object* objects;
//         int num_objects = detect_objects(downsampled, ds_count, &objects);
//         printf("Detected %d object(s)\n", num_objects);
//         for (int i = 0; i < num_objects; i++) {
//             printf("Object %d:\n", objects[i].cluster_id);
//             printf("  Position: x=%.3f, y=%.3f, z=%.3f\n",
//                    objects[i].centroid[0], objects[i].centroid[1], objects[i].centroid[2]);
//             printf("  Size: %d points\n", objects[i].size);
//         }
//         free(downsampled);
//         free(objects);
//     }
//     return NULL;
// }

// int main() {
//     // Set up signal handler
//     signal(SIGINT, signal_handler);

//     Synexens::SYDeviceHandle device = initialize_device();

//     Synexens::SYErrorCode error = Synexens::SetFrameResolution(device.m_nDeviceID, 
//                                                                Synexens::SYFRAMETYPE_DEPTH, 
//                                                                Synexens::SYRESOLUTION_640_480);
//     if (error != Synexens::SYERRORCODE_SUCCESS) {
//         print_error("SetFrameResolution", error);
//         Synexens::CloseDevice(device.m_nDeviceID);
//         Synexens::UnInitSDK();
//         return 1;
//     }

//     error = Synexens::StartStreaming(device.m_nDeviceID, Synexens::SYSTREAMTYPE_DEPTH);
//     if (error != Synexens::SYERRORCODE_SUCCESS) {
//         print_error("StartStreaming", error);
//         Synexens::CloseDevice(device.m_nDeviceID);
//         Synexens::UnInitSDK();
//         return 1;
//     }
//     printf("Streaming started\n");
//     sleep(5);  // Stabilize streaming

//     pthread_t capture_t, process_t;
//     pthread_create(&capture_t, NULL, capture_thread, &device);
//     pthread_create(&process_t, NULL, process_thread, NULL);

//     // Wait for threads to finish (on Ctrl+C)
//     pthread_join(capture_t, NULL);
//     pthread_join(process_t, NULL);

//     // Cleanup
//     if (buffer.points) free(buffer.points);
//     pthread_mutex_destroy(&buffer.mutex);
//     pthread_cond_destroy(&buffer.cond);

//     Synexens::StopStreaming(device.m_nDeviceID);
//     Synexens::CloseDevice(device.m_nDeviceID);
//     Synexens::UnInitSDK();
//     printf("Shutdown complete\n");
//     return 0;
// }
#include <iostream>
#include <mutex>
#include <map>
#include <atomic>
#include <thread>
#include <unistd.h>
#include <sys/stat.h>
#include <string.h>
#include <chrono>
#include "SYSDKInterface.h"
#include "opencv2/opencv.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "SYDataDefine.h"
#include <string>

std::atomic_bool g_is_start(false);
std::atomic_bool g_bBreak(false);
std::atomic_bool g_bRefreshFPS(false);
std::map<unsigned int, int> g_mapFPS;
std::map<unsigned int, int> g_mapFrameCount;
std::map<unsigned int, Synexens::SYStreamType> g_mapStreamType;

std::map<unsigned int, bool> g_mapSavePCL;
std::map<unsigned int, bool> g_mapSaveDataAll;
std::map<unsigned int, bool> g_mapShowConterValue;

double g_last_time = 0;
std::thread fpsThread;

void sprintf_s(char *const str, size_t const leng, char const *const _Format, unsigned int id) {
    sprintf(str, _Format, id);
}

void calculate_framerate() {
    while (g_is_start) {
        double cur_time = cv::getTickCount() / cv::getTickFrequency() * 1000;
        if (cur_time - g_last_time >= 1000) {
            g_bRefreshFPS = true;
            g_last_time = cur_time;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

static cv::Point cur_mouse_point = {-1, -1};
void on_mouse(int event, int x, int y, int flags, void *ustc) {
    if (event == cv::EVENT_LBUTTONDOWN || (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON))) {
        printf("x:%d y:%d\n", x, y);
        cur_mouse_point = cv::Point(x, y);
    } else if (event == cv::EVENT_LBUTTONUP) {
        cur_mouse_point = cv::Point(-1, -1);
    }
}

std::string createCatalogue(std::string filePathIn) {
    char *pFilePath = getcwd(NULL, 0);
    std::string strFileName = pFilePath ? pFilePath : "";
    free(pFilePath);
    strFileName += "/" + filePathIn + "/";
    mkdir(strFileName.c_str(), S_IRWXU);
    return strFileName;
}

void DumpRaw(unsigned int nDeviceID, uint8_t *saveData, Synexens::SYFrameType streamType, int w, int h) {
    std::string sTyepName = "";
    if (streamType == Synexens::SYFRAMETYPE_RAW) sTyepName = "raw";
    else if (streamType == Synexens::SYFRAMETYPE_DEPTH) sTyepName = "depth";
    else if (streamType == Synexens::SYFRAMETYPE_IR) sTyepName = "ir";

    std::string savePath = createCatalogue(sTyepName);
    std::string pre = std::to_string(nDeviceID + 1) + "_" + std::to_string(w) + "x" + std::to_string(h) + "-" + sTyepName + "_" + std::to_string(time(0)) + ".raw";
    std::string filePath = savePath + pre;

    FILE *fp = fopen(filePath.c_str(), "wb+");
    if (fp) {
        fwrite(saveData, sizeof(uint16_t), w * h, fp);
        fflush(fp);
        fclose(fp);
    }
}

void IsImgWindow(char *cWindowName, cv::Mat imgData) {
    cv::imshow(cWindowName, imgData);
}

void ProcessFrameData(unsigned int nDeviceID, Synexens::SYFrameData *pFrameData) {
    auto itStreamFind = g_mapStreamType.find(nDeviceID);
    if (itStreamFind == g_mapStreamType.end()) return;

    if (itStreamFind->second == Synexens::SYSTREAMTYPE_RGBD) {
        std::map<Synexens::SYFrameType, int> mapIndex, mapPos;
        int nPos = 0;
        for (int nFrameIndex = 0; nFrameIndex < pFrameData->m_nFrameCount; nFrameIndex++) {
            mapIndex[pFrameData->m_pFrameInfo[nFrameIndex].m_frameType] = nFrameIndex;
            mapPos[pFrameData->m_pFrameInfo[nFrameIndex].m_frameType] = nPos;
            nPos += pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameHeight * pFrameData->m_pFrameInfo[nFrameIndex].m_nFrameWidth * sizeof(short);
        }
        auto itDepthIndex = mapIndex.find(Synexens::SYFRAMETYPE_DEPTH);
        auto itRGBIndex = mapIndex.find(Synexens::SYFRAMETYPE_RGB);
        int nRGBDWidth = pFrameData->m_pFrameInfo[itRGBIndex->second].m_nFrameWidth;
        int nRGBDHeight = pFrameData->m_pFrameInfo[itRGBIndex->second].m_nFrameHeight;
        unsigned short *pRGBDDepth = new unsigned short[nRGBDWidth * nRGBDHeight];
        memset(pRGBDDepth, 0, sizeof(unsigned short) * nRGBDWidth * nRGBDHeight);
        unsigned char *pRGBDRGB = new unsigned char[nRGBDWidth * nRGBDHeight * 3];
        memset(pRGBDRGB, 0, sizeof(unsigned char) * nRGBDWidth * nRGBDHeight * 3);
        if (itDepthIndex != mapIndex.end() && itRGBIndex != mapIndex.end()) {
            if (GetRGBD(nDeviceID, pFrameData->m_pFrameInfo[itDepthIndex->second].m_nFrameWidth, pFrameData->m_pFrameInfo[itDepthIndex->second].m_nFrameHeight, (unsigned short *)pFrameData->m_pData + mapPos[Synexens::SYFRAMETYPE_DEPTH],
                        pFrameData->m_pFrameInfo[itRGBIndex->second].m_nFrameWidth, pFrameData->m_pFrameInfo[itRGBIndex->second].m_nFrameHeight, (unsigned char *)pFrameData->m_pData + mapPos[Synexens::SYFRAMETYPE_RGB],
                        nRGBDWidth, nRGBDHeight, pRGBDDepth, pRGBDRGB) == SYERRORCODE_SUCCESS) {
                g_mapFrameCount[nDeviceID]++;
                cv::Mat gray16(nRGBDHeight, nRGBDWidth, CV_16UC1, pRGBDDepth);
                int nCount = nRGBDHeight * nRGBDWidth;
                unsigned char *pColor = new unsigned char[nCount * 3];
                cv::Mat rgbimg(nRGBDHeight, nRGBDWidth, CV_8UC3);
                if (GetDepthColor(nDeviceID, nCount, pRGBDDepth, pColor) == SYERRORCODE_SUCCESS) {
                    memcpy(rgbimg.data, pColor, nCount * 3);
                } else {
                    cv::Mat tmp, gray8(nRGBDHeight, nRGBDWidth, CV_8U);
                    cv::normalize(gray16, tmp, 0, 255, cv::NORM_MINMAX);
                    cv::convertScaleAbs(tmp, gray8);
                    cv::cvtColor(gray8, rgbimg, cv::COLOR_GRAY2RGB);
                }
                delete[] pColor;

                std::string msg = std::to_string(nRGBDHeight) + "x" + std::to_string(nRGBDWidth) + " fps:" + std::to_string(g_mapFPS[nDeviceID]);
                int font_face = cv::FONT_HERSHEY_COMPLEX;
                double font_scale = 1;
                int thickness = 2;
                int baseline;
                cv::Size text_size = cv::getTextSize(msg, font_face, font_scale, thickness, &baseline);
                cv::Point origin(rgbimg.cols / 2 - text_size.width / 2, text_size.height);
                cv::putText(rgbimg, msg, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 2, 0);

                char cTemp[16];
                sprintf_s(cTemp, 16, "RGBD_depth_%d", nDeviceID);
                IsImgWindow(cTemp, rgbimg);

                cv::Mat bgrImgRGB(nRGBDHeight, nRGBDWidth, CV_8UC3, pRGBDRGB);
                sprintf_s(cTemp, 16, "RGBD_RGB_%d", nDeviceID);
                IsImgWindow(cTemp, bgrImgRGB);
            }
        }
        delete[] pRGBDDepth;
        delete[] pRGBDRGB;
    }
    // Add other stream types as needed (simplified for brevity)
}

void CreateOpencvWindow(unsigned int nDeviceID, Synexens::SYStreamType streamType, bool bDestoryOld = false) {
    char cTemp[16];
    switch (streamType) {
    case SYSTREAMTYPE_DEPTH:
        if (bDestoryOld) {
            sprintf_s(cTemp, 16, "raw_%d", nDeviceID);
            cv::destroyWindow(cTemp);
        }
        sprintf_s(cTemp, 16, "depth_%d", nDeviceID);
        cv::namedWindow(cTemp);
        cv::setMouseCallback(cTemp, on_mouse, 0);
        break;
    // Add other cases as needed
    }
}

class FrameObserver : public Synexens::ISYFrameObserver {
public:
    void OnFrameNotify(unsigned int nDeviceID, Synexens::SYFrameData *pFrameData = nullptr) override {
        ProcessFrameData(nDeviceID, pFrameData);
    }
};

FrameObserver g_FrameObserver;

void PrintErrorCode(std::string strFunc, Synexens::SYErrorCode errorCode) {
    printf("%s errorcode:%d\n", strFunc.c_str(), errorCode);
}

int main() {
    printf("SynexensSDK Test Demo\n");
    int nSDKVersionLength = 0;
    SYErrorCode errorCodeGetSDKVersion = GetSDKVersion(nSDKVersionLength, nullptr);
    if (errorCodeGetSDKVersion == SYERRORCODE_SUCCESS) {
        char *pStringSDKVersion = new char[nSDKVersionLength];
        errorCodeGetSDKVersion = GetSDKVersion(nSDKVersionLength, pStringSDKVersion);
        if (errorCodeGetSDKVersion == SYERRORCODE_SUCCESS) {
            printf("SDKVersion:%s\n", pStringSDKVersion);
        }
        delete[] pStringSDKVersion;
    }

    SYErrorCode errorCodeInitSDK = InitSDK();
    if (errorCodeInitSDK != SYERRORCODE_SUCCESS) {
        PrintErrorCode("InitSDK", errorCodeInitSDK);
    }

    int nCount = 0;
    SYErrorCode errorCode = FindDevice(nCount);
    if (errorCode == SYERRORCODE_SUCCESS && nCount > 0) {
        Synexens::SYDeviceInfo *pDeviceInfo = new Synexens::SYDeviceInfo[nCount];
        errorCode = FindDevice(nCount, pDeviceInfo);
        if (errorCode == SYERRORCODE_SUCCESS) {
            for (int i = 0; i < nCount; i++) {
                g_mapFrameCount[pDeviceInfo[i].m_nDeviceID] = 0;
                g_mapFPS[pDeviceInfo[i].m_nDeviceID] = 0;
                SYErrorCode errorCodeOpenDevice = OpenDevice(pDeviceInfo[i]);
                if (errorCodeOpenDevice == SYERRORCODE_SUCCESS) {
                    errorCode = SetFrameResolution(pDeviceInfo[i].m_nDeviceID, SYFRAMETYPE_DEPTH, SYRESOLUTION_640_480);
                    if (errorCode == SYERRORCODE_SUCCESS) {
                        errorCode = StartStreaming(pDeviceInfo[i].m_nDeviceID, SYSTREAMTYPE_DEPTH);
                        if (errorCode == SYERRORCODE_SUCCESS) {
                            g_mapStreamType[pDeviceInfo[i].m_nDeviceID] = SYSTREAMTYPE_DEPTH;
                            CreateOpencvWindow(pDeviceInfo[i].m_nDeviceID, SYSTREAMTYPE_DEPTH);
                        }
                    }
                }
            }

            g_is_start = true;
            fpsThread = std::thread(calculate_framerate);
            while (true) {
                if (g_bRefreshFPS) {
                    for (int i = 0; i < nCount; i++) {
                        g_mapFPS[pDeviceInfo[i].m_nDeviceID] = g_mapFrameCount[pDeviceInfo[i].m_nDeviceID];
                        g_mapFrameCount[pDeviceInfo[i].m_nDeviceID] = 0;
                    }
                    g_bRefreshFPS = false;
                }
                for (int i = 0; i < nCount; i++) {
                    Synexens::SYFrameData *pLastFrameData = nullptr;
                    SYErrorCode errorCodeLastFrame = GetLastFrameData(pDeviceInfo[i].m_nDeviceID, pLastFrameData);
                    if (errorCodeLastFrame == SYERRORCODE_SUCCESS) {
                        ProcessFrameData(pDeviceInfo[i].m_nDeviceID, pLastFrameData);
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                if (cv::waitKey(30) == 27) break; // ESC to exit
            }

            for (int i = 0; i < nCount; i++) {
                StopStreaming(pDeviceInfo[i].m_nDeviceID);
            }
            delete[] pDeviceInfo;
        }
    }

    UnInitSDK();
    g_is_start = false;
    if (fpsThread.joinable()) fpsThread.join();
    return 0;
}