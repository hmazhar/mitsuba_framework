#include "converter_general.h"
#include "MitsubaGenerator.h"
#include <chrono_parallel/math/ChParallelMath.h>
#include <sph_trianglemesh.h>
using namespace chrono;

std::vector<std::tuple<int, int, std::string> > labels;
int main(int argc, char* argv[]) {
    labels.clear();
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
    std::stringstream input_file_ss, input_file_vehicle;
    input_file_ss << "data_" << argv[1] << ".dat";

    std::cout << "Opening Files: " << input_file_ss.str() << " " << input_file_vehicle.str() << "\n";

    std::ifstream ifile;
    std::cout << "OpenBinary\n";
    OpenBinary(input_file_ss.str(), ifile);

    std::vector<chrono::real3> position;
    std::vector<real3> velocity;
    std::cout << "ReadBinary\n";
    ReadBinary(ifile, position);
    ReadBinary(ifile, velocity);
    std::cout << "CloseBinary\n";
    CloseBinary(ifile);

    std::stringstream output_file_ss;
    std::stringstream output_mesh_ss;

    if (argc == 3) {
        output_file_ss << argv[2] << argv[1] << ".xml";
    } else {
        output_file_ss << argv[1] << ".xml";
    }
    bool mode = true;


    real kernel_radius = 0.016 ;

    MitsubaGenerator data_document(output_file_ss.str());
    for (int i = 0; i < position.size(); i++) {
        ChVector<> pos = ChVector<>(position[i].x, position[i].y, position[i].z);
        data_document.AddShape("sphere", kernel_radius, pos, QUNIT);
    }
    data_document.Write();
    return 0;
}
