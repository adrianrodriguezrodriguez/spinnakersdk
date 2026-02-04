#include "pch.h"
#include "BBBDriver.h"
#include <iostream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

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

static int AskInt(const char* msg, int defVal)
{
    std::cout << msg << " [" << defVal << "]: ";
    std::string s; std::getline(std::cin, s);
    if (s.empty()) return defVal;
    return std::stoi(s);
}

static float AskFloat(const char* msg, float defVal)
{
    std::cout << msg << " [" << defVal << "]: ";
    std::string s; std::getline(std::cin, s);
    if (s.empty()) return defVal;
    return std::stof(s);
}

static double AskDouble(const char* msg, double defVal)
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

static void PrintParams(const BBBParams& p)
{
    std::cout << "\nParametros actuales:\n";
    std::cout << " - Rango valido (m): " << p.minRangeM << " .. " << p.maxRangeM << "\n";
    std::cout << " - ROI (zona central en %): " << p.roiMinPct << "% .. " << p.roiMaxPct << "%\n";
    std::cout << " - Decimation (nube PLY): " << p.decimationFactor << "\n";
}

static void PrintHelp()
{
    std::cout << "\nQue hace cada cosa:\n";
    std::cout << " 1) Captura y guarda:\n";
    std::cout << "    - disparity_XXXX.pgm  (disparidad cruda)\n";
    std::cout << "    - rectified_XXXX.png  (imagen rectificada)\n";
    std::cout << " 2) Genera nube:\n";
    std::cout << "    - cloud_XXXX.ply (para CloudCompare / Meshlab)\n";
    std::cout << " 3) Distancias:\n";
    std::cout << "    - Centro: distancia del pixel central\n";
    std::cout << "    - Cara bulto: estimacion robusta en ROI (percentil 10, filtrada por rango)\n";
    std::cout << " 4) Cambiar parametros:\n";
    std::cout << "    - Rango: filtra puntos mas cerca/lejos\n";
    std::cout << "    - ROI: mira solo la zona central (evita suelo/techo)\n";
    std::cout << "    - Exposure/Gain: calidad/ruido/blur\n";
    std::cout << " 5) Releer Scan3D: por si cambiaste configuracion/calibracion\n";
}

int main()
{
    BBBDriver driver;
    BBBParams p;

    std::cout << "=== BBBDriverConsole (Bumblebee / Spinnaker) ===\n\n";

    if (!driver.OpenFirstStereo())
    {
        std::cout << "ERROR: No se detecta ninguna camara estereo BBB.\n";
        std::cout << "Pulsa Enter para salir.\n";
        std::string tmp; std::getline(std::cin, tmp);
        return 1;
    }

    std::cout << "OK: Camara estereo detectada.\n";

    if (!driver.ConfigureStreams_Rectified1_Disparity())
    {
        std::cout << "ERROR: Fallo configurando streams Rectified Sensor1 + Disparity Sensor1.\n";
        return 2;
    }

    if (!driver.ConfigureSoftwareTrigger())
        std::cout << "AVISO: No pude activar trigger software (seguimos igualmente).\n";

    Scan3DParams s3d;
    if (!driver.ReadScan3DParams(s3d))
    {
        std::cout << "ERROR: Fallo leyendo parametros Scan3D.\n";
        return 3;
    }

    if (!driver.StartAcquisition())
    {
        std::cout << "ERROR: No pude iniciar adquisicion.\n";
        return 4;
    }

    PrintHelp();
    PrintParams(p);

    while (true)
    {
        std::cout << "\n=============================\n";
        std::cout << "MENU\n";
        std::cout << " 1) Capturar (guardar disparity + rectified)\n";
        std::cout << " 2) Capturar y generar nube 3D (PLY)\n";
        std::cout << " 3) Capturar y calcular distancias\n";
        std::cout << " 4) Cambiar parametros (rango/ROI/exposure/gain)\n";
        std::cout << " 5) Releer parametros Scan3D (calibracion)\n";
        std::cout << " 6) Ayuda\n";
        std::cout << " 0) Salir\n";
        std::cout << "Opcion: ";

        std::string opt;
        std::getline(std::cin, opt);

        if (opt == "0")
            break;

        if (opt == "6")
        {
            PrintHelp();
            PrintParams(p);
            continue;
        }

        if (opt == "4")
        {
            std::cout << "\n--- Cambiar parametros ---\n";
            std::cout << "RANGO (m): filtramos puntos fuera de este rango\n";
            p.minRangeM = AskFloat("Min range (m)  (ej: 1.0 o 1.5)", p.minRangeM);
            p.maxRangeM = AskFloat("Max range (m)  (ej: 6.0)", p.maxRangeM);

            std::cout << "\nROI (%): zona central donde estimamos la cara del bulto\n";
            std::cout << "Ejemplo: 35..65 mira el cuadrado central y evita bordes\n";
            p.roiMinPct = AskInt("ROI min pct (0..100)", p.roiMinPct);
            p.roiMaxPct = AskInt("ROI max pct (0..100)", p.roiMaxPct);

            std::cout << "\nPLY decimation: 1=todo, 2=la mitad, 4=1/4 (mas rapido, menos puntos)\n";
            p.decimationFactor = AskInt("Decimation (1/2/4)", p.decimationFactor);

            std::cout << "\nEXPOSURE / GAIN:\n";
            std::cout << " - Exposure (us): mas alto = mas luz pero mas blur\n";
            std::cout << " - Gain (dB): mas alto = mas brillo pero mas ruido\n";
            double expUs = AskDouble("Exposure (us)", 5000.0);
            double gainDb = AskDouble("Gain (dB)", 0.0);

            driver.SetExposureUs(expUs);
            driver.SetGainDb(gainDb);

            PrintParams(p);
            continue;
        }

        if (opt == "5")
        {
            if (driver.ReadScan3DParams(s3d))
                std::cout << "OK: Scan3D releido.\n";
            else
                std::cout << "FAIL: No pude releer Scan3D.\n";
            continue;
        }

        // Capturamos siempre 1 set para opciones 1/2/3
        Spinnaker::ImageList set;
        if (!driver.CaptureOnceSync(set, 5000))
        {
            std::cout << "FAIL: no pude capturar set (timeout o error).\n";
            continue;
        }

        const std::string tag = NowTag();

        if (opt == "1")
        {
            std::string fDisp = "disparity_" + tag + ".pgm";
            std::string fRect = "rectified_" + tag + ".png";

            bool ok1 = driver.SaveDisparityPGM(set, fDisp);
            bool ok2 = driver.SaveRectifiedPNG(set, fRect);

            std::cout << "Guardado:\n";
            std::cout << " - " << fDisp << " : " << (ok1 ? "OK" : "FAIL") << "\n";
            std::cout << " - " << fRect << " : " << (ok2 ? "OK" : "FAIL") << "\n";
        }
        else if (opt == "2")
        {
            std::string fPly = "cloud_" + tag + ".ply";

            if (driver.SavePointCloudPLY(set, s3d, p, fPly))
                std::cout << "OK: Guardado " << fPly << "\n";
            else
                std::cout << "FAIL: no pude generar PLY.\n";
        }
        else if (opt == "3")
        {
            float zCenter = 0.f;
            float zBulto = 0.f;
            int used = 0;

            bool okC = driver.GetDistanceCentralPointM(set, s3d, zCenter);
            bool okB = driver.GetDistanceToBultoM_Debug(set, s3d, p, zBulto, used);

            std::cout << "Resultados:\n";
            std::cout << " - Centro: " << (okC ? std::to_string(zCenter) + " m" : "FAIL") << "\n";
            if (okB)
                std::cout << " - Cara bulto (ROI + P10): " << zBulto << " m  (puntos validos=" << used << ")\n";
            else
                std::cout << " - Cara bulto: FAIL (puntos validos=" << used << ")\n";
        }
        else
        {
            std::cout << "Opcion no valida.\n";
        }

        ReleaseImageList(set);
    }

    driver.StopAcquisition();
    driver.Close();

    std::cout << "Saliendo...\n";
    return 0;
}
