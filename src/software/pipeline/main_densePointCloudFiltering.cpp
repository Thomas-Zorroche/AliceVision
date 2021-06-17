#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <aliceVision/fuseCut/DelaunayGraphCut.hpp>
#include <aliceVision/mesh/Texturing.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <OpenMesh/Core/IO/reader/OBJReader.hh>
#include <OpenMesh/Core/IO/writer/OBJWriter.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>


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
    std::string densePointCloudPath;
    std::string outputMeshPath;
    std::string inputMeshPath;

    po::options_description allParams("AliceVision dense point cloud filtering");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputRawSfm,i", po::value<std::string>(&densePointCloudPath)->required(), 
            "SfMData file.")
        ("inputMesh,i", po::value<std::string>(&inputMeshPath)->required(), 
            "Input Mesh (OBJ file format).")(
        "outputMesh,o", po::value<std::string>(&outputMeshPath)->required(), 
            "Output mesh (OBJ file format).");

    po::options_description optionalParams("Optional parameters");

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
            "verbosity level (fatal,  error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

    po::variables_map vm;

    try
    {
        po::store(po::parse_command_line(argc, argv, allParams), vm);

        if(vm.count("help") || (argc == 1))
        {
            ALICEVISION_COUT(allParams);
            return EXIT_SUCCESS;
        }

        po::notify(vm);
    }
    catch(boost::program_options::required_option& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);


    /////////////////////////////////////////////////////////////////
    // Dense Point Cloud Filtering

    ALICEVISION_COUT("DENSE POINT CLOUD FILTERING BEGIN...");

    // Mesh type
    typedef OpenMesh::TriMesh_ArrayKernelT<> Mesh;

    // Retrieve Mesh
    Mesh mesh;
    if(!OpenMesh::IO::read_mesh(mesh, inputMeshPath.c_str()))
    {
        ALICEVISION_LOG_ERROR("Unable to read input mesh from the file: " << inputMeshPath);
        return EXIT_FAILURE;
    }

    // Retrieve Dense Sfm Data Raw
    sfmData::SfMData densePointCloud;
    if(!sfmDataIO::Load(densePointCloud, densePointCloudPath, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << densePointCloudPath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("EXIT SUCCESS");

    return EXIT_SUCCESS;
}
