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

    std::string line_1 = "vehicle speed: " + std::to_string(vehicle_speed) + " [m/s] driveshaft speed: " + std::to_string(driveshaft_speed) + " [rad/s]";
    std::string line_2 = "motor torque: " + std::to_string(motor_torque) + " [Nm] motor speed: " + std::to_string(motor_speed) + " [rad/s] output torque: " +
                         std::to_string(output_torque) + "[Nm]";
    std::string line_3 = "wheel torques: [" + std::to_string(wheel_torque_0) + ", " + std::to_string(wheel_torque_1) + ", " + std::to_string(wheel_torque_2) +
                         ", " + std::to_string(wheel_torque_3) + "] [Nm]";
    std::string line_4 = "throttle: " + std::to_string(throttle) + " brake: " + std::to_string(braking);

    std::string line_5 = "Force on Vehicle [Total]: " + std::to_string(total_force) + " Force on Vehicle [Fluid]: " + std::to_string(ftotal_force);

    labels.push_back(std::make_tuple(25, 25, line_1));
    labels.push_back(std::make_tuple(25, 50, line_2));
    labels.push_back(std::make_tuple(25, 75, line_3));
    labels.push_back(std::make_tuple(25, 100, line_4));
    labels.push_back(std::make_tuple(25, 125, line_5));
}

int main(int argc, char* argv[]) {
    if (argc == 1) {
        std::cout << "REQURES FRAME NUMBER AS ARGUMENT, ONLY CREATING SCENE" << std::endl;
        MitsubaGenerator scene_document("scene.xml");

        std::vector<xml_option> integrator_options = {xml_option("boolean", "hideEmitters", "true"), xml_option("integer", "maxDepth", "-1"),
                                                      xml_option("integer", "rrDepth", "10")};

        std::vector<xml_option> emitter_options = {xml_option("string", "filename", "interior_hdri_2_20150408_1285285587.jpg"),
                                                   xml_option("float", "scale", "4.000000")};
        scene_document.AddInclude("geometry.xml");
        scene_document.AddIntegrator("volpath", integrator_options);
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
    std::stringstream input_file_ss, input_file_vehicle, input_stats_vehicle;
    input_file_ss << "data_" << argv[1] << ".dat";
    input_file_vehicle << "vehicle_" << argv[1] << ".dat";
    input_stats_vehicle << "stats_" << argv[1] << ".dat";

    std::cout << "Opening Files: " << input_file_ss.str() << " " << input_file_vehicle.str() << " " << input_stats_vehicle.str() << "\n";
    std::cout << "Stats\n";
    ReadStats(input_stats_vehicle.str());

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

    std::string data_v;
    std::cout << "ReadCompressed\n";
    ReadCompressed(input_file_vehicle.str(), data_v);
    std::replace(data_v.begin(), data_v.end(), ',', '\t');
    std::stringstream output_file_ss;
    std::stringstream output_mesh_ss;
    if (argc >= 3) {
        output_file_ss << argv[2] << argv[1] << ".xml";
        output_mesh_ss << argv[2] << argv[1] << ".obj";
        std::cout << "Writing Files: " << output_file_ss.str() << " " << output_mesh_ss.str() << "\n";

    } else {
        output_file_ss << argv[1] << ".xml";
        output_mesh_ss << argv[1] << ".obj";
    }

    bool color_velocity = true;
    bool follow_camera = true;

    if (argc >= 5) {
        color_velocity = atoi(argv[3]);
        follow_camera = atoi(argv[4]);
    }

    MitsubaGenerator data_document(output_file_ss.str());

    std::stringstream vehicle_stream(data_v);

    ChVector<> pos, vel, scale;

    std::vector<double> lengths(velocity.size());

    ChQuaternion<> rot;
    int count = 0;
    real max_vel = 0;
    real avg_vel = 0;
    for (int i = 0; i < velocity.size(); i++) {
        vel.x = velocity[i].x;
        vel.y = velocity[i].y;
        vel.z = velocity[i].z;

        lengths[i] = vel.Length();
        avg_vel += vel.Length();
    }
    avg_vel /= velocity.size();
    // std::cout <<avg_vel<<std::endl;
    std::sort(lengths.begin(), lengths.end());
    max_vel = lengths[velocity.size() - velocity.size() * .1];

    real variance = 0;
    for (int i = 0; i < velocity.size(); i++) {
        variance += (lengths[i] - avg_vel) * (lengths[i] - avg_vel);
        //        std::cout << lengths[i] << "\n";
    }
    variance /= velocity.size();

    real std_dev = sqrt(variance);
    printf("mean: %f, stddev: %f, max: %f\n", avg_vel, std_dev, max_vel);

    real kernel_radius = .016 * 2;
    if (argc >= 6) {
        kernel_radius = .016 * 2 * 0.9;
    }

    MarchingCubesToMesh(position, kernel_radius, output_mesh_ss.str());

    if (color_velocity || argc >= 6) {
        for (int i = 0; i < position.size(); i++) {
            pos.x = position[i].x;
            pos.y = position[i].y;
            pos.z = position[i].z;
            vel.x = velocity[i].x;
            vel.y = velocity[i].y;
            vel.z = velocity[i].z;
            double v = vel.Length() / max_vel;
            if (color_velocity) {
                data_document.AddCompleteShape("sphere", "diffuse", VelToColor(v), kernel_radius, pos, QUNIT);
            } else {
                data_document.AddShape("sphere", kernel_radius, pos, QUNIT);
            }
            count++;
	}
    } else {

        data_document.AddShape("fluid", ChVector<>(1), ChVector<>(0), QUNIT);
    }
    // std::cout << data_v << std::endl;

    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    data_document.AddShape("box", scale, pos, rot);
    SkipLine(vehicle_stream, 5);  // skip 2 cylinder edges, top, and 3 side plates
    //    ProcessPovrayLine(vehicle_stream, pos, vel, scale, rot);
    //    data_document.AddShape("box", scale, pos, rot);
    //    ProcessPovrayLine(data_stream, pos, vel, scale, rot);
    //    data_document.AddShape("box", scale, pos, rot);
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
    // chrono::ChQuaternion<> q;
    // q.Q_from_AngAxis(CH_C_PI, chrono::ChVector<>(1, 0, 0));
    /// rot = rot * q;
    Vector offset = Vector(0, 0, 0);  // rot.Rotate(Vector(-0.055765, 0, -0.52349));
    data_document.AddShape("chassis", Vector(1, 1, 1), pos + offset, rot);
    std::vector<xml_option> sampler_options = {xml_option("integer", "sampleCount", "256")};
    if (follow_camera) {
        Vector camera_pos = pos + offset;
        camera_pos.z = 4;
        camera_pos.y -= 8;
        camera_pos.x += 0;

        data_document.AddSensor(camera_pos, pos + offset, Vector(0, 0, 1), labels, "sobol", sampler_options);
    } else {
        Vector camera_pos = ChVector<>(0, -7.4, 3);
        Vector camera_target = ChVector<>(0, -6.4, 2.84);
        data_document.AddSensor(camera_pos, camera_target, Vector(0, 0, 1), labels, "sobol", sampler_options);
    }

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

    data_document.Write();
    return 0;
}
