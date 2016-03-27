#include "converter_general.h"
#include "MitsubaGenerator.h"
#include <chrono_parallel/math/ChParallelMath.h>
#include <sph_trianglemesh.h>

#include <thrust/transform_reduce.h>
#include <thrust/pair.h>
#include <thrust/functional.h>

#include <thrust/remove.h>
#include <thrust/unique.h>
#include <thrust/binary_search.h>
#include <thrust/sort.h>

using namespace chrono;

int main(int argc, char* argv[]) {
    std::ifstream ifile;
    // std::cout << "OpenBinary\n";
    for (int file_num = 0; file_num < 588; file_num++) {
        OpenBinary("fluid_" + std::to_string(file_num) + ".dat", ifile);
        std::vector<chrono::real> density;
        std::vector<chrono::real> pressure;
        std::vector<chrono::real3> force;
        std::vector<chrono::real3> position;
        std::vector<real3> velocity;
        // std::cout << "ReadBinary\n";
        ReadBinary(ifile, density);
        ReadBinary(ifile, pressure);
        ReadBinary(ifile, force);
        ReadBinary(ifile, position);
        ReadBinary(ifile, velocity);
        //        std::cout << "CloseBinary " << density.size() << " " << pressure.size() << " " << force.size() << " " << position.size() << " " <<
        //        velocity.size()
        //                  << "\n";
        CloseBinary(ifile);

        std::sort(density.begin(), density.end());
        std::sort(pressure.begin(), pressure.end());

        std::vector<chrono::real> force_length(force.size());

        for (int i = 0; i < force.size(); i++) {
            force_length[i] = Length(force[i]);
        }

        std::sort(force_length.begin(), force_length.end());

        real avg_density = 0;
        real avg_pressure = 0;

        for (int i = 0; i < density.size(); i++) {
            avg_density += density[i];
            avg_pressure += pressure[i];
        }
        avg_density = avg_density / density.size();
        avg_pressure = avg_pressure / pressure.size();

        std::vector<chrono::real> marker_height(position.size());
        for (int i = 0; i < position.size(); i++) {
            marker_height[i] = position[i].z;
        }

        thrust::sort_by_key(marker_height.begin(), marker_height.end(),
                            thrust::make_zip_iterator(thrust::make_tuple(force_length.begin(), density.begin(), pressure.begin())));

        if (file_num == 587) {
            std::ofstream ofile("final_frame.txt");
            for (int i = 0; i < position.size(); i++) {
                ofile << marker_height[i] << " " << force_length[i] << " " << density[i] << " " << pressure[i] << "\n";
            }
            ofile.close();
        }

        // Round data to nearest nth value
        // Unique the values
        // write values

        printf("Density, Pressure: [%f %f] \n", avg_density, avg_pressure);
    }

    return 0;
}
