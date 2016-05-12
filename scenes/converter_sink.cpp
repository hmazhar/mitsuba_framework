#include "converter_general.h"
#include "MitsubaGenerator.h"
#include <chrono_parallel/math/ChParallelMath.h>
#include <sph_trianglemesh.h>
using namespace std;
using namespace chrono;
std::vector<std::tuple<int, int, std::string> > labels;
int main(int argc, char* argv[]) {
    if (argc == 1) {
        std::cout << "REQURES FRAME NUMBER AS ARGUMENT" << std::endl;
        // std::cout<<data<<std::endl;
        MitsubaGenerator scene_document("scene.xml");
        scene_document.AddInclude("geometry.xml");
        std::vector<xml_option> sampler_options = {xml_option("integer", "sampleCount", "256")};
        scene_document.AddSensor(ChVector<>(0, -11.5, 0), ChVector<>(0, -9, 0), Vector(0, 0, 1), labels, "sobol", sampler_options);
        std::vector<xml_option> emitter_options = {xml_option("string", "filename", "interior_hdri_2_20150408_1285285587.jpg"),
                                                   xml_option("float", "scale", "4.000000")};
        std::vector<xml_option> integrator_options = {xml_option("boolean", "hideEmitters", "true"), xml_option("integer", "maxDepth", "20"),
                                                      xml_option("integer", "rrDepth", "10")};
        scene_document.AddIntegrator("volpath", integrator_options);
        scene_document.AddEmitter("envmap", emitter_options, ChVector<>(1, 1, 1), ChVector<>(0, 0, 0), Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X));
        scene_document.AddShape("background", ChVector<>(20, 20, 5), ChVector<>(0, 2.1366, 0), Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X));
        scene_document.AddInclude("$frame.xml");
        // scene_document.CreateScene(true, true);
        scene_document.Write();

        return 1;
    }
    stringstream input_file_ss;
    input_file_ss << "rigid_" << argv[1] << ".dat";

    string data;
    ReadCompressed(input_file_ss.str(), data);
    std::replace(data.begin(), data.end(), ',', '\t');

    stringstream output_file_ss, output_mesh_ss;
    if (argc >= 3) {
        output_file_ss << argv[2] << argv[1] << ".xml";
        output_mesh_ss << argv[2] << argv[1] << ".obj";
    } else {
        output_file_ss << argv[1] << ".xml";
        output_mesh_ss << argv[1] << ".obj";
    }
    MitsubaGenerator data_document(output_file_ss.str());

    stringstream data_stream(data);
    SkipLine(data_stream, 4);

    ChVector<> pos, vel, scale;
    ChQuaternion<> rot;

    int type = ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    data_document.AddShape("base", scale, pos, rot);
    SkipLine(data_stream, 1);
    while (data_stream.fail() == false) {
        int type = ProcessPovrayLine(data_stream, pos, vel, scale, rot);
        if (data_stream.fail() == false) {
            switch (type) {
                case chrono::collision::SPHERE:
                    data_document.AddShape("sphere", scale, pos, rot);
                    break;
                case chrono::collision::ELLIPSOID:
                    data_document.AddShape("ellipsoid", scale, pos, rot);
                    break;
                case chrono::collision::BOX:
                    data_document.AddShape("box", scale, pos, rot);
                    break;
                case chrono::collision::CYLINDER:
                    data_document.AddShape("cylinder", scale, pos, rot);
                    break;
                case chrono::collision::CONE:
                    data_document.AddShape("cone", scale, pos, rot);
                    break;
                case (-1):
                    data_document.AddShape("bucky", scale, pos, rot);
                    break;
            }
        }
    }
    std::stringstream input_file_ff;
    input_file_ff << "fluid_" << argv[1] << ".dat";
    std::vector<chrono::real3> position;
    std::vector<chrono::real3> velocity;
    std::ifstream ifile;
    std::cout << "OpenBinary\n";
    OpenBinary(input_file_ff.str(), ifile);
    std::cout << "ReadBinary\n";
    ReadBinary(ifile, position);
    ReadBinary(ifile, velocity);
    std::cout << "CloseBinary\n";
    CloseBinary(ifile);

    MarchingCubesToMesh(position, .02, output_mesh_ss.str(), chrono::real3(-5, -5, -5), chrono::real3(5, 5, 5), 0.000001);
    // data_document.AddShape("fluid", ChVector<>(1), ChVector<>(0), QUNIT);

    data_document.Write();
    return 0;
}
