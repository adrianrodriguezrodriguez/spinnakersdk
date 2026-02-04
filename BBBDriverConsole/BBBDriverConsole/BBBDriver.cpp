#include "pch.h"
#include "BBBDriver.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <fstream>
#include <iostream>
#include <cstdint>

#ifndef NOMINMAX
#define NOMINMAX
#endif

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

using namespace Spinnaker;
using namespace Spinnaker::GenApi;

static float Percentile(std::vector<float>& v, float q)
{
    if (v.empty()) return std::numeric_limits<float>::quiet_NaN();
    std::sort(v.begin(), v.end());
    float idx = q * (v.size() - 1);
    size_t i0 = (size_t)idx;
    size_t i1 = (std::min)(i0 + 1, v.size() - 1);
    float t = idx - (float)i0;
    return v[i0] * (1.f - t) + v[i1] * t;
}

BBBDriver::BBBDriver()
{
    system = System::GetInstance();
}

BBBDriver::~BBBDriver()
{
    Close();
    if (system)
        system->ReleaseInstance();
}

CameraPtr BBBDriver::GetCamera() const
{
    return cam;
}

bool BBBDriver::SetEnumAsString(INodeMap& nodeMap, const char* name, const char* value)
{
    CEnumerationPtr node = nodeMap.GetNode(name);
    if (!IsReadable(node) || !IsWritable(node)) return false;

    CEnumEntryPtr entry = node->GetEntryByName(value);
    if (!IsReadable(entry)) return false;

    node->SetIntValue(entry->GetValue());
    return true;
}

bool BBBDriver::GetFloatNode(INodeMap& nodeMap, const char* name, float& out)
{
    CFloatPtr node = nodeMap.GetNode(name);
    if (!IsReadable(node)) return false;
    out = (float)node->GetValue();
    return true;
}

bool BBBDriver::GetBoolNode(INodeMap& nodeMap, const char* name, bool& out)
{
    CBooleanPtr node = nodeMap.GetNode(name);
    if (!IsReadable(node)) return false;
    out = node->GetValue();
    return true;
}

bool BBBDriver::OpenFirstStereo()
{
    CameraList cams = system->GetCameras();
    if (cams.GetSize() == 0) return false;

    for (unsigned int i = 0; i < cams.GetSize(); i++)
    {
        CameraPtr c = cams.GetByIndex(i);
        c->Init();

        bool ok = ImageUtilityStereo::IsStereoCamera(c);
        if (ok)
        {
            cam = c;
            return true;
        }

        c->DeInit();
    }

    return false;
}

bool BBBDriver::OpenBySerial(const std::string& serial)
{
    CameraList cams = system->GetCameras();
    if (cams.GetSize() == 0) return false;

    for (unsigned int i = 0; i < cams.GetSize(); i++)
    {
        CameraPtr c = cams.GetByIndex(i);
        c->Init();

        std::string s = c->TLDevice.DeviceSerialNumber.ToString().c_str();
        if (s == serial && ImageUtilityStereo::IsStereoCamera(c))
        {
            cam = c;
            return true;
        }

        c->DeInit();
    }

    return false;
}

void BBBDriver::Close()
{
    try
    {
        if (cam)
        {
            StopAcquisition();
            cam->DeInit();
            cam = nullptr;
        }
    }
    catch (...) {}
}

bool BBBDriver::ConfigureStreams_Rectified1_Disparity()
{
    if (!cam) return false;
    INodeMap& nodeMap = cam->GetNodeMap();

    CEnumerationPtr sourceSel = nodeMap.GetNode("SourceSelector");
    CEnumerationPtr compSel = nodeMap.GetNode("ComponentSelector");
    CBooleanPtr compEnable = nodeMap.GetNode("ComponentEnable");

    if (!IsReadable(sourceSel) || !IsWritable(sourceSel)) return false;
    if (!IsReadable(compSel) || !IsWritable(compSel)) return false;
    if (!IsReadable(compEnable) || !IsWritable(compEnable)) return false;

    CEnumEntryPtr sensor1 = sourceSel->GetEntryByName("Sensor1");
    CEnumEntryPtr rectified = compSel->GetEntryByName("Rectified");
    CEnumEntryPtr disparity = compSel->GetEntryByName("Disparity");

    if (!IsReadable(sensor1) || !IsReadable(rectified) || !IsReadable(disparity)) return false;

    sourceSel->SetIntValue(sensor1->GetValue());
    compSel->SetIntValue(rectified->GetValue());
    compEnable->SetValue(true);

    sourceSel->SetIntValue(sensor1->GetValue());
    compSel->SetIntValue(disparity->GetValue());
    compEnable->SetValue(true);

    return true;
}

bool BBBDriver::StartAcquisition()
{
    if (!cam) return false;
    if (acquiring) return true;

    try
    {
        cam->BeginAcquisition();
        acquiring = true;
        return true;
    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "BeginAcquisition fallo: " << e.what() << "\n";
        acquiring = false;
        return false;
    }
}

void BBBDriver::StopAcquisition()
{
    if (!cam || !acquiring) return;
    try { cam->EndAcquisition(); }
    catch (...) {}
    acquiring = false;
}


bool BBBDriver::ConfigureSoftwareTrigger()
{
    if (!cam) return false;
    INodeMap& nodeMap = cam->GetNodeMap();

    // Algunas cámaras requieren AcquisitionMode=Continuous
    SetEnumAsString(nodeMap, "AcquisitionMode", "Continuous");

    if (!SetEnumAsString(nodeMap, "TriggerMode", "Off")) return false;
    if (!SetEnumAsString(nodeMap, "TriggerSelector", "FrameStart")) return false;
    if (!SetEnumAsString(nodeMap, "TriggerSource", "Software")) return false;
    if (!SetEnumAsString(nodeMap, "TriggerMode", "On")) return false;

    return true;
}

bool BBBDriver::ReadScan3DParams(Scan3DParams& out)
{
    if (!cam) return false;
    INodeMap& nodeMap = cam->GetNodeMap();

    if (!GetFloatNode(nodeMap, "Scan3dCoordinateScale", out.scale)) return false;
    if (!GetFloatNode(nodeMap, "Scan3dCoordinateOffset", out.offset)) return false;
    if (!GetFloatNode(nodeMap, "Scan3dFocalLength", out.focal)) return false;
    if (!GetFloatNode(nodeMap, "Scan3dBaseline", out.baseline)) return false;
    if (!GetFloatNode(nodeMap, "Scan3dPrincipalPointU", out.principalU)) return false;
    if (!GetFloatNode(nodeMap, "Scan3dPrincipalPointV", out.principalV)) return false;
    if (!GetBoolNode(nodeMap, "Scan3dInvalidDataFlag", out.invalidFlag)) return false;
    if (!GetFloatNode(nodeMap, "Scan3dInvalidDataValue", out.invalidValue)) return false;

    return true;
}

bool BBBDriver::CaptureOnceSync(ImageList& outSet, uint64_t timeoutMs)
{
    if (!cam) return false;

    if (!StartAcquisition())
        return false;

    try
    {
        // Disparo software
        INodeMap& nodeMap = cam->GetNodeMap();
        CCommandPtr sw = nodeMap.GetNode("TriggerSoftware");
        if (IsWritable(sw))
            sw->Execute();

        outSet = cam->GetNextImageSync(timeoutMs);
        return true;
    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "GetNextImageSync fallo: " << e.what() << "\n";
        return false;
    }
}


// Guardado manual para disparity (evita el crash de disp->Save)
static bool SavePGM8(const ImagePtr& img, const std::string& filePath)
{
    const int w = (int)img->GetWidth();
    const int h = (int)img->GetHeight();
    const uint8_t* data = (const uint8_t*)img->GetData();
    if (!data) return false;

    const int stride = (int)img->GetStride();

    std::ofstream f(filePath, std::ios::binary);
    if (!f.is_open()) return false;

    f << "P5\n" << w << " " << h << "\n255\n";
    for (int y = 0; y < h; ++y)
        f.write((const char*)(data + y * stride), w);

    return true;
}

static bool SavePGM16_BE(const ImagePtr& img, const std::string& filePath)
{
    const int w = (int)img->GetWidth();
    const int h = (int)img->GetHeight();
    const uint16_t* data = (const uint16_t*)img->GetData();
    if (!data) return false;

    const int strideU16 = (int)(img->GetStride() / sizeof(uint16_t));

    std::ofstream f(filePath, std::ios::binary);
    if (!f.is_open()) return false;

    f << "P5\n" << w << " " << h << "\n65535\n";

    for (int y = 0; y < h; ++y)
    {
        const uint16_t* row = data + y * strideU16;
        for (int x = 0; x < w; ++x)
        {
            uint16_t v = row[x];
            unsigned char be[2] = {
                (unsigned char)(v >> 8),
                (unsigned char)(v & 0xFF)
            };
            f.write((char*)be, 2);
        }
    }

    return true;
}

bool BBBDriver::SaveDisparityPGM(const ImageList& set, const std::string& filePath)
{
    ImagePtr disp = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1);
    if (!disp) return false;
    if (disp->IsIncomplete()) return false;
    if (!disp->GetData()) return false;

    try
    {
        std::cout << "Disparity info: "
            << disp->GetWidth() << "x" << disp->GetHeight()
            << " stride=" << disp->GetStride()
            << " bpp=" << disp->GetBitsPerPixel()
            << " fmtEnum=" << (int)disp->GetPixelFormat()

            << "\n";

        const unsigned int bpp = disp->GetBitsPerPixel();

        if (bpp <= 8)
            return SavePGM8(disp, filePath);

        return SavePGM16_BE(disp, filePath);
    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "Excepcion guardando disparity: " << e.what() << "\n";
        return false;
    }
    catch (...)
    {
        std::cout << "Error desconocido guardando disparity\n";
        return false;
    }
}

bool BBBDriver::SaveRectifiedPNG(const ImageList& set, const std::string& filePath)
{
    ImagePtr rect = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1);
    if (!rect || rect->IsIncomplete()) return false;

    try
    {
        rect->Save(filePath.c_str());
        return true;
    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "Excepcion guardando rectified: " << e.what() << "\n";
        return false;
    }
}

bool BBBDriver::SavePointCloudPLY(const ImageList& set, const Scan3DParams& s3d,
    const BBBParams& p, const std::string& filePath)
{
    ImagePtr disp = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1);
    ImagePtr rect = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1);
    if (!disp || !rect) return false;
    if (disp->IsIncomplete() || rect->IsIncomplete()) return false;

    PointCloudParameters pc;
    pc.decimationFactor = (std::max)(1, p.decimationFactor);
    pc.ROIImageLeft = 0;
    pc.ROIImageTop = 0;
    pc.ROIImageRight = disp->GetWidth();
    pc.ROIImageBottom = disp->GetHeight();

    StereoCameraParameters sc;
    sc.coordinateOffset = s3d.offset;
    sc.baseline = s3d.baseline;
    sc.focalLength = s3d.focal;
    sc.principalPointU = s3d.principalU;
    sc.principalPointV = s3d.principalV;
    sc.disparityScaleFactor = s3d.scale;
    sc.invalidDataFlag = s3d.invalidFlag;
    sc.invalidDataValue = s3d.invalidValue;

    try
    {
        PointCloud cloud = ImageUtilityStereo::ComputePointCloud(disp, rect, pc, sc);
        cloud.SavePointCloudAsPly(filePath.c_str());
        return true;
    }
    catch (Spinnaker::Exception& e)
    {
        std::cout << "Excepcion generando PLY: " << e.what() << "\n";
        return false;
    }
}

bool BBBDriver::GetDistanceCentralPointM(const ImageList& set, const Scan3DParams& s3d, float& outMeters)
{
    ImagePtr disp = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1);
    if (!disp || disp->IsIncomplete()) return false;

    const int w = (int)disp->GetWidth();
    const int h = (int)disp->GetHeight();
    const int cx = w / 2;
    const int cy = h / 2;

    const uint16_t* data = (const uint16_t*)disp->GetData();
    if (!data) return false;

    const int strideU16 = (int)(disp->GetStride() / sizeof(uint16_t));
    const uint16_t raw = data[cy * strideU16 + cx];

    const float d = (float)raw * s3d.scale + s3d.offset;
    if (d <= 1e-6f) return false;

    const float z = (s3d.focal * s3d.baseline) / d;
    if (!std::isfinite(z)) return false;

    outMeters = z;
    return true;
}

bool BBBDriver::GetDistanceToBultoM(const ImageList& set, const Scan3DParams& s3d,
    const BBBParams& p, float& outMeters)
{
    ImagePtr disp = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1);
    if (!disp || disp->IsIncomplete()) return false;

    const int w = (int)disp->GetWidth();
    const int h = (int)disp->GetHeight();

    const int x0 = w * p.roiMinPct / 100;
    const int x1 = w * p.roiMaxPct / 100;
    const int y0 = h * p.roiMinPct / 100;
    const int y1 = h * p.roiMaxPct / 100;

    const uint16_t* data = (const uint16_t*)disp->GetData();
    if (!data) return false;

    const int strideU16 = (int)(disp->GetStride() / sizeof(uint16_t));

    std::vector<float> depths;
    depths.reserve((x1 - x0) * (y1 - y0));

    for (int y = y0; y < y1; ++y)
    {
        const uint16_t* row = data + y * strideU16;
        for (int x = x0; x < x1; ++x)
        {
            const uint16_t raw = row[x];
            const float d = (float)raw * s3d.scale + s3d.offset;
            if (d <= 1e-6f) continue;

            const float z = (s3d.focal * s3d.baseline) / d;
            if (!std::isfinite(z)) continue;

            if (z >= p.minRangeM && z <= p.maxRangeM)
                depths.push_back(z);
        }
    }

    if (depths.size() < 100) return false;

    outMeters = Percentile(depths, 0.10f);
    return std::isfinite(outMeters);
}
bool BBBDriver::GetDistanceToBultoM_Debug(const ImageList& set, const Scan3DParams& s3d,
    const BBBParams& p, float& outMeters, int& outUsedPoints)
{
    outUsedPoints = 0;

    ImagePtr disp = set.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1);
    if (!disp || disp->IsIncomplete()) return false;

    const int w = (int)disp->GetWidth();
    const int h = (int)disp->GetHeight();

    const int x0 = w * p.roiMinPct / 100;
    const int x1 = w * p.roiMaxPct / 100;
    const int y0 = h * p.roiMinPct / 100;
    const int y1 = h * p.roiMaxPct / 100;

    const uint16_t* data = (const uint16_t*)disp->GetData();
    if (!data) return false;

    const int strideU16 = (int)(disp->GetStride() / sizeof(uint16_t));

    std::vector<float> depths;
    depths.reserve((x1 - x0) * (y1 - y0));

    for (int y = y0; y < y1; ++y)
    {
        const uint16_t* row = data + y * strideU16;
        for (int x = x0; x < x1; ++x)
        {
            const uint16_t raw = row[x];
            const float d = (float)raw * s3d.scale + s3d.offset;
            if (d <= 1e-6f) continue;

            const float z = (s3d.focal * s3d.baseline) / d;
            if (!std::isfinite(z)) continue;

            if (z >= p.minRangeM && z <= p.maxRangeM)
            {
                depths.push_back(z);
                outUsedPoints++;
            }
        }
    }

    if (depths.size() < 100) return false;

    outMeters = Percentile(depths, 0.10f);
    return std::isfinite(outMeters);
}


bool BBBDriver::SetExposureUs(double exposureUs)
{
    if (!cam) return false;
    try
    {
        cam->ExposureAuto.SetValue(ExposureAuto_Off);
        cam->ExposureTime.SetValue(exposureUs);
        return true;
    }
    catch (...) { return false; }
}

bool BBBDriver::SetGainDb(double gainDb)
{
    if (!cam) return false;
    try
    {
        cam->GainAuto.SetValue(GainAuto_Off);
        cam->Gain.SetValue(gainDb);
        return true;
    }
    catch (...) { return false; }
}
