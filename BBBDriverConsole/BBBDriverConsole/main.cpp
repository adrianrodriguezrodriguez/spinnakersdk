#include "BBBDriver.h"

#include <chrono>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <ctime>

static std::string NowTag()
{
    using namespace std::chrono;
    auto now = system_clock::now();
    auto t = system_clock::to_time_t(now);

    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    tm = *std::localtime(&t);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
    return oss.str();
}

static int AskInt(const std::string& msg, int defVal)
{
    std::cout << msg << " [" << defVal << "]: ";
    std::string s; std::getline(std::cin, s);
    if (s.empty()) return defVal;
    return std::stoi(s);
}

static int AskInt(const std::string& msg)
{
    return AskInt(msg, 0);
}


static float AskFloat(const std::string& msg, float defVal)
{
    std::cout << msg << " [" << defVal << "]: ";
    std::string s; std::getline(std::cin, s);
    if (s.empty()) return defVal;
    return std::stof(s);
}

static double AskDouble(const std::string& msg, double defVal)
{
    std::cout << msg << " [" << defVal << "]: ";
    std::string s; std::getline(std::cin, s);
    if (s.empty()) return defVal;
    return std::stod(s);
}

static void ReleaseImageList(Spinnaker::ImageList& set)
{
    const unsigned int n = (unsigned int)set.GetSize();
    for (unsigned int i = 0; i < n; i++)
    {
        Spinnaker::ImagePtr img = set.GetByIndex(i);
        if (img) img->Release();
    }
}

static const char* ColorModeName(int m)
{
    if (m == 2) return "heatmapZ";
    if (m == 1) return "rectifiedGray";
    return "gray";
}

static void PrintParams(const BBBParams& p)
{
    std::cout << "\nParametros\n";
    std::cout << " - Rango " << p.minRangeM << " a " << p.maxRangeM << " m\n";
    std::cout << " - ROI " << p.roiMinPct << " a " << p.roiMaxPct << " %\n";
    std::cout << " - Decimacion " << p.decimationFactor << "\n";
    std::cout << " - Speckle " << (p.applySpeckleFilter ? "ON" : "OFF") << " size " << p.maxSpeckleSize
        << " thr " << p.speckleThreshold << "\n";
    std::cout << " - Median3x3 " << (p.applyMedian3x3 ? "ON" : "OFF") << "\n";
    std::cout << " - Voxel " << p.voxelLeafM << " m\n";
    std::cout << " - Outlier radius " << p.outlierRadiusM << " m  minN " << p.outlierMinNeighbors << "\n";
    std::cout << " - Cluster " << (p.keepLargestCluster ? "ON" : "OFF") << "\n";
    std::cout << " - ColorMode " << p.colorMode << " " << ColorModeName(p.colorMode) << "\n";
    std::cout << " - PLY " << (p.plyBinary ? "BIN" : "ASCII") << "\n";
}

int main()
{
    BBBDriver driver;
    BBBParams p;

    std::cout << "=== BBBDriverConsole BBB Spinnaker ===\n\n";

    if (!driver.OpenFirstStereo())
    {
        std::cout << "ERROR no detectamos camara estereo\n";
        return 1;
    }

#ifdef _DEBUG
    driver.DisableGVCPHeartbeat(true);
#endif

    std::cout << "OK camara estereo detectada\n";

    if (!driver.ConfigureStreams_Rectified1_Disparity())
    {
        std::cout << "ERROR no pudimos configurar rectified y disparity\n";
        return 2;
    }

    if (!driver.ConfigureSoftwareTrigger())
        std::cout << "AVISO trigger software no activado pero seguimos\n";

    Scan3DParams s3d;
    if (!driver.ReadScan3DParams(s3d))
    {
        std::cout << "ERROR leyendo Scan3D\n";
        return 3;
    }

    std::cout << "Scan3D baseline " << s3d.baseline
        << " focal " << s3d.focal
        << " scale " << s3d.scale
        << " offset " << s3d.offset << "\n";

    if (!driver.StartAcquisition())
    {
        std::cout << "ERROR no pudimos iniciar adquisicion\n";
        return 4;
    }

    PrintParams(p);

    while (true)
    {
        std::cout << "\n---------------------------------\n";
        std::cout << "MENU\n";
        std::cout << " 1 Guardar disparity pgm y rectified png\n";
        std::cout << " 2 Generar nube ply limpia con colorMode\n";
        std::cout << " 3 Medir distancia centro y cara del bulto\n";
        std::cout << " 4 Cambiar parametros\n";
        std::cout << " 5 Releer Scan3D\n";
        std::cout << " 0 Salir\n";
        std::cout << "Opcion: ";

        std::string opt;
        std::getline(std::cin, opt);

        if (opt == "0") break;

        if (opt == "4")
        {
            p.minRangeM = AskFloat("Min rango metros", p.minRangeM);
            p.maxRangeM = AskFloat("Max rango metros", p.maxRangeM);

            p.roiMinPct = AskInt("ROI min porcentaje", p.roiMinPct);
            p.roiMaxPct = AskInt("ROI max porcentaje", p.roiMaxPct);

            p.decimationFactor = AskInt("Decimacion", p.decimationFactor);

            p.applySpeckleFilter = (AskInt("Speckle 1 ON 0 OFF", p.applySpeckleFilter ? 1 : 0) != 0);
            p.maxSpeckleSize = AskInt("Speckle max size", p.maxSpeckleSize);
            p.speckleThreshold = AskInt("Speckle threshold", p.speckleThreshold);

            p.applyMedian3x3 = (AskInt("Median3x3 1 ON 0 OFF", p.applyMedian3x3 ? 1 : 0) != 0);

            p.voxelLeafM = AskFloat("Voxel leaf metros", p.voxelLeafM);
            p.outlierRadiusM = AskFloat("Outlier radius metros", p.outlierRadiusM);
            p.outlierMinNeighbors = AskInt("Outlier min vecinos", p.outlierMinNeighbors);
            p.keepLargestCluster = (AskInt("Cluster mas grande 1 ON 0 OFF", p.keepLargestCluster ? 1 : 0) != 0);

            p.colorMode = AskInt("ColorMode 0 gris 1 rectified 2 heatmapZ", p.colorMode);

            p.plyBinary = (AskInt("PLY bin 1 BIN 0 ASCII", p.plyBinary ? 1 : 0) != 0);

            double expUs = AskDouble("Exposure microsegundos", 5000.0);
            double gainDb = AskDouble("Gain dB", 0.0);

            driver.SetExposureUs(expUs);
            driver.SetGainDb(gainDb);

            PrintParams(p);
            continue;
        }

        if (opt == "5")
        {
            if (driver.ReadScan3DParams(s3d))
                std::cout << "OK Scan3D releido\n";
            else
                std::cout << "FAIL no pudimos releer Scan3D\n";
            continue;
        }

        Spinnaker::ImageList set;
        if (!driver.CaptureOnceSync(set, 5000))
        {
            std::cout << "FAIL no capturamos set\n";
            ReleaseImageList(set);
            continue;
        }

        const std::string tag = NowTag();

        if (opt == "1")
        {
            std::string fDisp = "disparity_" + tag + ".pgm";
            std::string fRect = "rectified_" + tag + ".png";

            bool ok1 = driver.SaveDisparityPGM(set, fDisp);
            bool ok2 = driver.SaveRectifiedPNG(set, fRect);

            std::cout << "Guardado\n";
            std::cout << " - " << fDisp << " " << (ok1 ? "OK" : "FAIL") << "\n";
            std::cout << " - " << fRect << " " << (ok2 ? "OK" : "FAIL") << "\n";
        }
        else if (opt == "2")
        {
            std::string fPly = "cloud_" + tag + ".ply";

            if (driver.SavePointCloudPLY_Filtered(set, s3d, p, fPly))
                std::cout << "OK guardado " << fPly << "\n";
            else
                std::cout << "FAIL no pudimos generar PLY\n";
        }
        else if (opt == "3")
        {
            float zCenter = 0.f;
            float zBulto = 0.f;
            int used = 0;

            bool okC = driver.GetDistanceCentralPointM(set, s3d, zCenter);
            bool okB = driver.GetDistanceToBultoM_Debug(set, s3d, p, zBulto, used);

            std::cout << "Distancias\n";
            if (okC) std::cout << " - Centro " << zCenter << " m\n";
            else std::cout << " - Centro FAIL\n";

            if (okB) std::cout << " - Cara bulto " << zBulto << " m  puntos " << used << "\n";
            else std::cout << " - Cara bulto FAIL  puntos " << used << "\n";
        }
        else
        {
            std::cout << "Opcion no valida\n";
        }

        ReleaseImageList(set);
    }

    driver.StopAcquisition();
    driver.Close();
    std::cout << "Saliendo\n";
    return 0;
}
