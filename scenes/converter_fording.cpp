#include "converter_general.h"
#include "MitsubaGenerator.h"
#include <chrono_parallel/math/ChParallelMath.h>
#include <sph_trianglemesh.h>
using namespace chrono;

double vehicle_speed, driveshaft_speed, motor_torque, motor_speed, output_torque;
double wheel_torque_0, wheel_torque_1, wheel_torque_2, wheel_torque_3;
ChVector<> wheel_linvel_0, wheel_linvel_1, wheel_linvel_2, wheel_linvel_3;
ChVector<> wheel_angvel_0, wheel_angvel_1, wheel_angvel_2, wheel_angvel_3;
double spring_def_fl, spring_def_fr, spring_def_rl, spring_def_rr;
double shock_len_fl, shock_len_fr, shock_len_rl, shock_len_rr;
double throttle, braking;

ChVector<> chassis_torque, wheel_torquev_0, wheel_torquev_1, wheel_torquev_2, wheel_torquev_3;
ChVector<> fchassis_torque, fwheel_torquev_0, fwheel_torquev_1, fwheel_torquev_2, fwheel_torquev_3;

ChVector<> chassis_force, wheel_forcev_0, wheel_forcev_1, wheel_forcev_2, wheel_forcev_3;
ChVector<> fchassis_force, fwheel_forcev_0, fwheel_forcev_1, fwheel_forcev_2, fwheel_forcev_3;

std::vector<std::tuple<int, int, std::string> > labels;
std::stringstream input_file_ss, input_file_vehicle, input_stats_vehicle;

Vector vehicle_pos;
MitsubaGenerator data_document;
std::stringstream vehicle_stream;
ChVector<> pos, vel, scale;
ChQuaternion<> rot;

void ReadStats(std::string filename) {
    std::ifstream ifile(filename.c_str());

    std::string line;
    std::getline(ifile, line);
    std::replace(line.begin(), line.end(), ',', '\t');
    std::stringstream ss(line);

    ChVector<> chassis_pos;
    real total_force = 0, ftotal_force = 0;
    real total_torque = 0, ftotal_torque = 0;
    ss >> chassis_pos.x >> chassis_pos.y >> chassis_pos.z;
    ss >> vehicle_speed >> driveshaft_speed >> motor_torque >> motor_speed >> output_torque;

    ss >> throttle >> braking;

    ss >> total_force;
    ss >> total_torque;
    ss >> ftotal_force;
    ss >> ftotal_torque;

    ss >> wheel_torque_0 >> wheel_torque_1 >> wheel_torque_2 >> wheel_torque_3;

    ss >> wheel_linvel_0.x >> wheel_linvel_0.y >> wheel_linvel_0.z   //
        >> wheel_linvel_1.x >> wheel_linvel_1.y >> wheel_linvel_1.z  //
        >> wheel_linvel_2.x >> wheel_linvel_2.y >> wheel_linvel_2.z  //
        >> wheel_linvel_3.x >> wheel_linvel_3.y >> wheel_linvel_3.z;

    ss >> wheel_angvel_0.x >> wheel_angvel_0.y >> wheel_angvel_0.z   //
        >> wheel_angvel_1.x >> wheel_angvel_1.y >> wheel_angvel_1.z  //
        >> wheel_angvel_2.x >> wheel_angvel_2.y >> wheel_angvel_2.z  //
        >> wheel_angvel_3.x >> wheel_angvel_3.y >> wheel_angvel_3.z;

    ss >> spring_def_fl >> spring_def_fr >> spring_def_rl >> spring_def_rr;
    ss >> shock_len_fl >> shock_len_fr >> shock_len_rl >> shock_len_rr;

    ss >> chassis_force.x >> chassis_force.y >> chassis_force.z;
    ss >> wheel_forcev_0.x >> wheel_forcev_0.y >> wheel_forcev_0.z;
    ss >> wheel_forcev_1.x >> wheel_forcev_1.y >> wheel_forcev_1.z;
    ss >> wheel_forcev_2.x >> wheel_forcev_2.y >> wheel_forcev_2.z;
    ss >> wheel_forcev_3.x >> wheel_forcev_3.y >> wheel_forcev_3.z;

    ss >> chassis_torque.x >> chassis_torque.y >> chassis_torque.z;
    ss >> wheel_torquev_0.x >> wheel_torquev_0.y >> wheel_torquev_0.z;
    ss >> wheel_torquev_1.x >> wheel_torquev_1.y >> wheel_torquev_1.z;
    ss >> wheel_torquev_2.x >> wheel_torquev_2.y >> wheel_torquev_2.z;
    ss >> wheel_torquev_3.x >> wheel_torquev_3.y >> wheel_torquev_3.z;

    ss >> fchassis_force.x >> fchassis_force.y >> fchassis_force.z;
    ss >> fwheel_forcev_0.x >> fwheel_forcev_0.y >> fwheel_forcev_0.z;
    ss >> fwheel_forcev_1.x >> fwheel_forcev_1.y >> fwheel_forcev_1.z;
    ss >> fwheel_forcev_2.x >> fwheel_forcev_2.y >> fwheel_forcev_2.z;
    ss >> fwheel_forcev_3.x >> fwheel_forcev_3.y >> fwheel_forcev_3.z;

    ss >> fchassis_torque.x >> fchassis_torque.y >> fchassis_torque.z;
    ss >> fwheel_torquev_0.x >> fwheel_torquev_0.y >> fwheel_torquev_0.z;
    ss >> fwheel_torquev_1.x >> fwheel_torquev_1.y >> fwheel_torquev_1.z;
    ss >> fwheel_torquev_2.x >> fwheel_torquev_2.y >> fwheel_torquev_2.z;
    ss >> fwheel_torquev_3.x >> fwheel_torquev_3.y >> fwheel_torquev_3.z;

    ifile.close();

    std::string line_1 = "vehicle speed: " + std::to_string(vehicle_speed) + " [m/s] driveshaft speed: " + std::to_string(driveshaft_speed) +
                         " [rad/s] motor speed: " + std::to_string(motor_speed);
    //    std::string line_2 = "motor torque: " + std::to_string(motor_torque) + " [Nm] motor speed: " + std::to_string(motor_speed) + " [rad/s] output torque:
    //    " + std::to_string(output_torque) + "[Nm]";
    std::string line_3 = "wheel torques: [" + std::to_string(wheel_torque_0) + ", " + std::to_string(wheel_torque_1) + ", " + std::to_string(wheel_torque_2) +
                         ", " + std::to_string(wheel_torque_3) + "] [Nm]";
    std::string line_4 = "throttle: " + std::to_string(throttle) + " brake: " + std::to_string(braking);

    std::string line_5 = "Force on Vehicle [Total, Fluid]: [" + std::to_string(total_force) + ", " + std::to_string(ftotal_force) + "] [N]";

    labels.push_back(std::make_tuple(25, 25, line_1));
    // labels.push_back(std::make_tuple(25, 50, line_2));
    labels.push_back(std::make_tuple(25, 50, line_3));
    labels.push_back(std::make_tuple(25, 75, line_4));
    labels.push_back(std::make_tuple(25, 100, line_5));
}

void ReadVehicleData() {
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("box", scale, pos, rot);
    SkipLine(vehicle_stream, 5);                              // skip 2 cylinder edges, top, and 3 side plates
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);  // end platform
    data_document.AddShape("box", scale, pos, rot);
    SkipLine(vehicle_stream, 2);  // skip cylinder and end cap
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("box", scale, pos, rot);  // end platform
    SkipLine(vehicle_stream, 2);                     // skip cylinder and end cap
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("box", scale, pos, rot);  // slope
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("box", scale, pos, rot);  // slope

    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("chassis", Vector(1, 1, 1), pos, rot);
    vehicle_pos = pos;
}
void ReadTireData() {
    SkipLine(vehicle_stream, 5);
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("lugged_wheel_R", Vector(1, 1, 1), pos, rot);
    SkipLine(vehicle_stream, 22);
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("lugged_wheel_R", Vector(1, -1, 1), pos, rot);
    SkipLine(vehicle_stream, 22);
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("lugged_wheel_R", Vector(1, 1, 1), pos, rot);
    SkipLine(vehicle_stream, 22);
    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("lugged_wheel_R", Vector(1, -1, 1), pos, rot);
    // SkipLine(vehicle_stream, 22);
}

void ReadFluidData() {}

int main(int argc, char* argv[]) {
    if (argc == 1) {
        std::cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << std::endl;
        MitsubaGenerator scene_document("scene.xml");

        std::vector<xml_option> integrator_options = {xml_option("boolean", "hideEmitters", "true"), xml_option("integer", "maxDepth", "20"),
                                                      xml_option("integer", "rrDepth", "10")};

        std::vector<xml_option> emitter_options = {xml_option("string", "filename", "interior_hdri_2_20150408_1285285587.jpg"),
                                                   xml_option("float", "scale", "4.000000")};
        scene_document.AddInclude("geometry.xml");
        scene_document.AddIntegrator("volpath", integrator_options);
        scene_document.AddEmitter("envmap", emitter_options, ChVector<>(1, 1, 1), ChVector<>(0, 0, 0), Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X));
        scene_document.AddInclude("$frame.xml");
        scene_document.Write();
        return 0;
    }
    // Read all user input

    if (argc != 6) {
        printf("not enough arguments\n");
        return 0;
    }

    std::string sim_frame = std::string(argv[1]);
    std::string output_folder = std::string(argv[2]);
    int color_mode = atoi(argv[3]);
    int follow_camera = atoi(argv[4]);
    bool is_rigid_fluid = atoi(argv[5]);

    //============================================================================
    data_document.Open(output_folder + sim_frame + ".xml");
    //============================================================================
    //============================================================================
    // Vehicle and stats data are always there
    std::string data_v;
    ReadCompressed("vehicle_" + sim_frame + ".dat", data_v);
    std::replace(data_v.begin(), data_v.end(), ',', ' ');
    vehicle_stream << data_v;
    ReadVehicleData();
    ReadStats("stats_" + sim_frame + ".dat");

    std::ifstream fluid_file;
    if (OpenBinary("data_" + sim_frame + ".dat", fluid_file)) {
        std::vector<chrono::real3> position, velocity;
        ReadBinary(fluid_file, position);
        ReadBinary(fluid_file, velocity);
        CloseBinary(fluid_file);

        std::vector<chrono::real3> old_position(0);
        real max_disp = 0;

        if (color_mode == 2) {
            std::vector<double> lengths(position.size());
            std::ifstream ifile;
            std::cout << "Open Color Binary\n";
            OpenBinary("data_120.dat", ifile);
            ReadBinary(ifile, old_position);
            CloseBinary(ifile);

            for (int i = 0; i < position.size(); i++) {
                lengths[i] = Length(position[i] - old_position[i]);
            }
            std::sort(lengths.begin(), lengths.end());
            max_disp = lengths[velocity.size() - velocity.size() * .01];
        }

        std::vector<double> lengths(velocity.size());
        for (int i = 0; i < velocity.size(); i++) {
            lengths[i] = ChVector<>(velocity[i].x, velocity[i].y, velocity[i].z).Length();
        }
        std::sort(lengths.begin(), lengths.end());
        real max_vel = lengths[velocity.size() - velocity.size() * .1];
        printf("max fluid vel : %f\n", max_vel);

        real kernel_radius = .016 * 2;
        if (is_rigid_fluid) {
            kernel_radius = .016 * 2 * 0.9;
        }

        if (color_mode > 0 || is_rigid_fluid) {
            for (int i = 0; i < position.size(); i++) {
                pos = ChVector<>(position[i].x, position[i].y, position[i].z);
                if (color_mode == 1) {
                    double v = ChVector<>(velocity[i].x, velocity[i].y, velocity[i].z).Length() / max_vel;
                    data_document.AddCompleteShape("sphere", "diffuse", VelToColor(v), kernel_radius, pos, QUNIT);
                } else if (color_mode == 2) {
                    double v = Length(position[i] - old_position[i]) / max_disp;
                    data_document.AddCompleteShape("sphere", "diffuse", VelToColor(v), kernel_radius, pos, QUNIT);
                } else {
                    data_document.AddShape("sphere", kernel_radius, pos, QUNIT);
                }
            }
        }
        if (is_rigid_fluid == 0) {
            MarchingCubesToMesh(position, kernel_radius, output_folder + sim_frame + ".obj", real3(-13, -3, -3), real3(13, 3, 9), 0.000001);
            data_document.AddShape("fluid", ChVector<>(1), ChVector<>(0), QUNIT);
        }
    }

    std::vector<xml_option> sampler_options = {xml_option("integer", "sampleCount", "256"), xml_option("integer", "scramble", argv[1])};
    if (follow_camera == 1) {
        Vector camera_pos = vehicle_pos;
        // camera_pos.x += 0;
        camera_pos.y = -8;
        camera_pos.z = 4;
        data_document.AddShape("background", ChVector<>(20, 20, 5), ChVector<>(0, 2.1366, 0), Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X));
        data_document.AddSensor(camera_pos, vehicle_pos, Vector(0, 0, 1), labels, "sobol", sampler_options);

    } else if (follow_camera == 2) {
        Vector camera_pos = vehicle_pos;
        // camera_pos.x += 0;
        camera_pos.y = 8;
        camera_pos.z = 4;
        data_document.AddShape("background", ChVector<>(20, 20, 5), ChVector<>(0, -2.1366, 0), Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X));
        data_document.AddSensor(camera_pos, vehicle_pos, Vector(0, 0, 1), labels, "sobol", sampler_options);

    } else if (follow_camera == 3) {
        Vector camera_pos = ChVector<>(0, 7.4, 3);
        Vector camera_target = ChVector<>(0, 6.4, 2.84);
        data_document.AddSensor(camera_pos, camera_target, Vector(0, 0, 1), labels, "sobol", sampler_options);
        data_document.AddShape("background", ChVector<>(20, 20, 5), ChVector<>(0, -2.1366, 0), Q_from_AngAxis(-90 * CH_C_DEG_TO_RAD, VECT_X));

    } else {
        Vector camera_pos = ChVector<>(0, -7.4, 3);
        Vector camera_target = ChVector<>(0, -6.4, 2.84);
        data_document.AddShape("background", ChVector<>(20, 20, 5), ChVector<>(0, 2.1366, 0), Q_from_AngAxis(90 * CH_C_DEG_TO_RAD, VECT_X));
        data_document.AddSensor(camera_pos, camera_target, Vector(0, 0, 1), labels, "sobol", sampler_options);
    }

    std::ifstream fem_file("tire_" + sim_frame + ".obj");

    if (fem_file.good()) {
        data_document.AddShape("tire", ChVector<>(1), ChVector<>(0), QUNIT);

        while (vehicle_stream.fail() == false) {
            int type = ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
            if (vehicle_stream.fail() == false) {
                switch (type) {
                    case chrono::collision::CYLINDER:
                        if (scale.x == .223) {
                            scale.y = .125;
                        }
                        data_document.AddShape("cylinder", scale, pos, rot);
                        break;
                    default:
                        SkipLine(vehicle_stream, 1);
                        break;
                }
            }
        }

    } else {
        ReadTireData();
    }
    fem_file.close();

    data_document.Write();
    return 0;
}
