#include "converter_general.h"
#include "MitsubaGenerator.h"

using namespace std;
using namespace chrono;

int main(int argc, char* argv[]) {
    if (argc == 1) {
        std::cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << std::endl;
        MitsubaGenerator scene_document("scene.xml");

        std::vector<xml_option> integrator_options = {xml_option("boolean", "hideEmitters", "true"), xml_option("integer", "maxDepth", "20"),
                                                      xml_option("integer", "rrDepth", "10")};

        std::vector<xml_option> emitter_options = {xml_option("string", "filename", "interior_hdri_2_20150408_1285285587.jpg"),
                                                   xml_option("float", "scale", "4.000000")};
        scene_document.AddInclude("geometry.xml");
        scene_document.AddIntegrator("path", integrator_options);
        scene_document.AddEmitter("envmap", emitter_options, ChVector<>(1, 1, 1), ChVector<>(0, 0, 0), Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X));

        scene_document.AddShape("background", ChVector<>(20, 20, 5), ChVector<>(0, 2.1366, 0), Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X));
        scene_document.AddInclude("$frame.xml");
        //
        //        scene_document.camera_origin = ChVector<>(0, -7.4, 3);
        //        scene_document.camera_target = ChVector<>(0, -6.4, 2.84);
        //        scene_document.camera_up = ChVector<>(0, 0, 1);
        //        scene_document.scale = 3;
        //        scene_document.turbidity = 10;
        //        scene_document.CreateScene();
        // scene_document.AddShape("background", ChVector<>(1), ChVector<>(0, -10,
        // 0), ChQuaternion<>(1, 0, 0, 0));
        scene_document.Write();
        return 0;
    }
    stringstream input_file_ss;
    input_file_ss << "data_slsdata_" << argv[1] << ".dat";

    std::cout << "Opening: " << input_file_ss.str() << "\n";

    string data;
    ReadCompressed(input_file_ss.str(), data);
    std::replace(data.begin(), data.end(), ',', '\t');

    stringstream output_file_ss;
    if (argc >= 3) {
        output_file_ss << argv[2] << argv[1] << ".xml";
    } else {
        output_file_ss << argv[1] << ".xml";
    }

    bool color_spheres = false;

    if (argc >= 4) {
        color_spheres = atoi(argv[3]);
    }

    MitsubaGenerator data_document(output_file_ss.str());

    stringstream data_stream(data);
    SkipLine(data_stream, 5);

    ChVector<> pos, vel, scale;
    ChQuaternion<> rot;

    int type = ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    data_document.AddShape("box", scale, pos, rot);

    type = ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    data_document.AddShape("cylinder", scale, pos, rot);

    std::vector<xml_option> sampler_options = {xml_option("integer", "sampleCount", "256"), xml_option("integer", "scramble", argv[1])};
    Vector offset = Vector(0, 0, 0);
    Vector camera_pos = pos + offset;
    camera_pos.z -= 10;
    camera_pos.y = 1;
    camera_pos.x -= 20;

    pos.z -= 5;
    pos.y = 1;
    pos.x -= 4;

    std::vector<std::tuple<int, int, std::string> > labels;
    data_document.AddSensor(camera_pos, pos, Vector(0, 1, 0), labels, "sobol", sampler_options);

    std::vector<ChVector<> > pos_vector;
    std::vector<ChVector<> > vel_vector;
    std::vector<ChVector<> > scale_vector;
    std::vector<ChQuaternion<> > rot_vector;
    std::vector<int> type_vector;
    int object_counts = 0;
    while (data_stream.fail() == false) {
        int type = ProcessPovrayLine(data_stream, pos, vel, scale, rot);
        if (data_stream.fail() == false) {
            pos_vector.push_back(pos);
            vel_vector.push_back(vel);
            scale_vector.push_back(scale);
            rot_vector.push_back(rot);
            type_vector.push_back(type);
            object_counts++;
        }
    }

    std::vector<double> lengths(object_counts);

    for (int i = 0; i < object_counts; i++) {
        vel.x = vel_vector[i].x;
        vel.y = vel_vector[i].y;
        vel.z = vel_vector[i].z;
        lengths[i] = vel.Length();
    }
    std::sort(lengths.begin(), lengths.end());
    double max_vel = lengths[vel_vector.size() - vel_vector.size() * .1];

    for (int i = 0; i < object_counts; i++) {
        int type = type_vector[i];

        pos = pos_vector[i];
        vel = vel_vector[i];
        scale = scale_vector[i];
        rot = rot_vector[i];

        switch (type) {
            case chrono::collision::SPHERE:
                if (color_spheres) {
                    double v = vel.Length() / max_vel;

                    data_document.AddCompleteShape("sphere", "diffuse", VelToColor(v), scale, pos, QUNIT);
                } else {
                    data_document.AddShape("sphere", scale, pos, rot);
                }
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
            default:
                // type is -1 (triangle mesh)
                break;
        }
    }

    data_document.Write();
    return 0;
}
