#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <string>
#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace fs = boost::filesystem;
namespace po = boost::program_options;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string rawSfmDataPath;
    std::string outputMeshPath;
    std::string inputMeshPath;

    po::options_description allParams("AliceVision dense point cloud filtering");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&rawSfmDataPath)->required(), 
            "SfMData file.")
        ("inputMesh,i", po::value<std::string>(&inputMeshPath)->required(), 
            "Input Mesh (OBJ file format).")(
        "outputMesh,o", po::value<std::string>(&outputMeshPath)->required(), 
            "Output mesh (OBJ file format).");

    po::options_description logParams("Log parameters");
    logParams.add_options()("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
                            "verbosity level (fatal,  error, warning, info, debug, trace).");




    ALICEVISION_COUT("DENSE POINT CLOUD FILTERING DEBUG...");

    return EXIT_SUCCESS;
}
