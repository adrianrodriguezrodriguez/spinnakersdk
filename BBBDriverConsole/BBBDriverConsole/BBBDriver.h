#pragma once

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <string>



struct BBBParams
{
    float minRangeM = 1.0f;
    float maxRangeM = 6.0f;

    int roiMinPct = 35;
    int roiMaxPct = 65;

    int decimationFactor = 2;
};

struct Scan3DParams
{
    float scale = 1.0f;
    float offset = 0.0f;
    float focal = 0.0f;
    float baseline = 0.0f;
    float principalU = 0.0f;
    float principalV = 0.0f;
    bool invalidFlag = false;
    float invalidValue = 0.0f;
};

class BBBDriver
{
public:

    bool StartAcquisition();
    void StopAcquisition();
    bool IsAcquiring() const { return acquiring; }

    BBBDriver();
    ~BBBDriver();

    bool OpenFirstStereo();
    bool OpenBySerial(const std::string& serial);
    void Close();

    bool ConfigureStreams_Rectified1_Disparity();
    bool ConfigureSoftwareTrigger();

    bool ReadScan3DParams(Scan3DParams& out);

    bool CaptureOnceSync(Spinnaker::ImageList& outSet, uint64_t timeoutMs);

    bool SaveDisparityPGM(const Spinnaker::ImageList& set, const std::string& filePath);
    bool SaveRectifiedPNG(const Spinnaker::ImageList& set, const std::string& filePath);

    bool SavePointCloudPLY(const Spinnaker::ImageList& set, const Scan3DParams& s3d,
        const BBBParams& p, const std::string& filePath);

    bool GetDistanceCentralPointM(const Spinnaker::ImageList& set, const Scan3DParams& s3d, float& outMeters);

    bool GetDistanceToBultoM(const Spinnaker::ImageList& set, const Scan3DParams& s3d,
        const BBBParams& p, float& outMeters);

    bool GetDistanceToBultoM_Debug(const Spinnaker::ImageList& set, const Scan3DParams& s3d,
        const BBBParams& p, float& outMeters, int& outUsedPoints);


    bool SetExposureUs(double exposureUs);
    bool SetGainDb(double gainDb);

    Spinnaker::CameraPtr GetCamera() const;

private:
    bool acquiring = false;

    static bool SetEnumAsString(Spinnaker::GenApi::INodeMap& nodeMap, const char* name, const char* value);
    static bool GetFloatNode(Spinnaker::GenApi::INodeMap& nodeMap, const char* name, float& out);
    static bool GetBoolNode(Spinnaker::GenApi::INodeMap& nodeMap, const char* name, bool& out);

private:
    Spinnaker::SystemPtr system;
    Spinnaker::CameraPtr cam;
};
