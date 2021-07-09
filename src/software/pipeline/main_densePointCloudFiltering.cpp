#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <aliceVision/fuseCut/DelaunayGraphCut.hpp>

#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/mesh/Texturing.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/system/Timer.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

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
    system::Timer timer;

    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string densePointCloudPath;
    std::string outputDensePointCloud;
    std::string outputMeshPath;
    std::string inputMeshPath;
    float radiusFactor = 1.0f;
    float filterStrength = 1.0f;
    int nPointsMax = 25;
    int nNeighborsMax = 50;
    int interations = 1;
    float persistence = 1.0f;

    po::options_description allParams("AliceVision dense point cloud filtering");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputRawSfm,i", po::value<std::string>(&densePointCloudPath)->required(), 
            "SfMData file.")
        ("inputMesh,i", po::value<std::string>(&inputMeshPath)->required(), 
            "Input Mesh (OBJ file format).")
        ("outputMesh,o", po::value<std::string>(&outputMeshPath)->required(), 
            "Output mesh (OBJ file format).")(
        "outputDensePointCloud,o", po::value<std::string>(&outputDensePointCloud)->required(), 
            "Output Dense Point Cloud");


    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("filterStrength,i", po::value<float>(&filterStrength)->default_value(filterStrength),
            "filterStrength")
        ("iterations,i", po::value<int>(&interations)->default_value(interations), 
            "interations")
        ("radiusFactor,i", po::value<float>(&radiusFactor)->default_value(radiusFactor),
            "radiusFactor")
        ("persistence,i", po::value<float>(&persistence)->default_value(persistence),
            "persistence")
        ("nNeighborsMax,i", po::value<int>(&nNeighborsMax)->default_value(nNeighborsMax), 
            "nMatchesLimit")
        ("nPointsMax,i", po::value<int>(&nPointsMax)->default_value(nPointsMax), 
            "nMatchesMax");


    po::options_description advancedParams("Advanced parameters");

    ALICEVISION_LOG_INFO("Radius Factor: " << radiusFactor);
    ALICEVISION_LOG_INFO("Filter Strength: " << filterStrength);

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
            "verbosity level (fatal,  error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(advancedParams).add(logParams);

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

    ALICEVISION_COUT("DENSE POINT CLOUD FILTERING NODE BEGIN...");

    // Retrieve Mesh
    mesh::Texturing texturing;
    texturing.loadOBJWithAtlas(inputMeshPath);
    mesh::Mesh* mesh = texturing.mesh;

    if(!mesh)
    {
        ALICEVISION_LOG_ERROR("Unable to read input mesh from the file: " << inputMeshPath);
        return EXIT_FAILURE;
    }
    if(mesh->pts.empty() || mesh->tris.empty())
    {
        ALICEVISION_LOG_ERROR("Error: empty mesh from the file " << inputMeshPath);
        ALICEVISION_LOG_ERROR("Input mesh: " << mesh->pts.size() << " vertices and " << mesh->tris.size()
                                             << " facets.");
        return EXIT_FAILURE;
    }
    ALICEVISION_LOG_INFO("Mesh file: \"" << inputMeshPath << "\" loaded.");
    ALICEVISION_LOG_INFO("Input mesh: " << mesh->pts.size() << " vertices and " << mesh->tris.size() << " facets.");

    // Retrieve Dense Sfm Data Raw
    sfmData::SfMData densePointCloud;
    if(!sfmDataIO::Load(densePointCloud, densePointCloudPath, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << densePointCloudPath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Convert dense point cloud to Point3D vector");

    for(size_t i = 0; i < interations; i++)
    {
        aliceVision::fuseCut::filterDensePointCloud(densePointCloud, mesh, radiusFactor, filterStrength, nPointsMax, nNeighborsMax);
        filterStrength = filterStrength * persistence;
    }

    ALICEVISION_LOG_INFO("Save obj mesh file.");
    mesh->saveToObj(outputMeshPath);

    ALICEVISION_LOG_INFO("Save dense point cloud.");
    sfmDataIO::Save(densePointCloud, outputDensePointCloud, sfmDataIO::ESfMData::ALL_DENSE);

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
