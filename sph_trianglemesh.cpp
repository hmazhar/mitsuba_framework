#define THRUST_HOST_SYSTEM THRUST_HOST_SYSTEM_OMP

#include <sph_trianglemesh.h>

#include <thrust/transform_reduce.h>
#include <thrust/pair.h>
#include <thrust/functional.h>

#include <thrust/remove.h>
#include <thrust/unique.h>
#include <thrust/binary_search.h>
#include <thrust/sort.h>
#include <chrono_parallel/collision/ChBroadphaseUtils.h>
#include <chrono_parallel/physics/MPMUtils.h>
using namespace chrono;
using namespace chrono::collision;

chrono::vec3 bins_per_axis;
real bin_edge;
real inv_bin_edge;
real3 min_bounding_point;
uint grid_size;
real3 diag;
real3 max_bounding_point;

real3 abs_min = real3(-C_LARGE_REAL, -C_LARGE_REAL, -C_LARGE_REAL);
real3 abs_max = real3(C_LARGE_REAL, C_LARGE_REAL, C_LARGE_REAL);

real round_to_nearest = .000001;

std::vector<chrono::vec3> node_num;
std::vector<chrono::real3> node_loc;
std::vector<real> node_mass;

std::vector<real3> _VertexStorage(15);
std::vector<chrono::vec3> _FaceStorage(10);

uint GetDensity(real3 xi) {
    return GridHash(GridCoord(xi.x, inv_bin_edge, min_bounding_point.x),  //
                    GridCoord(xi.y, inv_bin_edge, min_bounding_point.y),  //
                    GridCoord(xi.z, inv_bin_edge, min_bounding_point.z),  //
                    bins_per_axis);
}

real3 GetNormal(real3 p) {
    real3 result(0.0);

    real3 p0 = real3(p.x + bin_edge, p.y, p.z);
    real3 p1 = real3(p.x, p.y + bin_edge, p.z);
    real3 p2 = real3(p.x, p.y, p.z + bin_edge);

    real3 p3 = real3(p.x - bin_edge, p.y, p.z);
    real3 p4 = real3(p.x, p.y - bin_edge, p.z);
    real3 p5 = real3(p.x, p.y, p.z - bin_edge);

    real d0 = node_mass[GetDensity(p0)];
    real d1 = node_mass[GetDensity(p1)];
    real d2 = node_mass[GetDensity(p2)];
    real d3 = node_mass[GetDensity(p3)];
    real d4 = node_mass[GetDensity(p4)];
    real d5 = node_mass[GetDensity(p5)];

    result.x = d0 - d3;
    result.y = d1 - d4;
    result.z = d2 - d5;
    return Normalize(result);
}
// Linear Interpolation function
real3 VertexInterp(real3 p1, real3 p2, real valp1, real valp2) {
    return (p1 + (-valp1 / (valp2 - valp1)) * (p2 - p1));
}
struct GRIDCELL {
    real3 p[8];   // position of each corner of the grid in world space
    real val[8];  // value of the function at this grid corner
};

uint Polygonise(GRIDCELL& Grid, std::vector<chrono::vec3>& Triangles, uint& NewVertexCount, std::vector<real3>& Vertices) {
    uint TriangleCount;
    uint CubeIndex;

    real3 VertexList[12];
    real3 NewVertexList[12];
    char LocalRemap[12];

    // Determine the index into the edge table which
    // tells us which vertices are inside of the surface
    CubeIndex = 0;
    if (Grid.val[0] < 0.0)
        CubeIndex |= 1;
    if (Grid.val[1] < 0.0)
        CubeIndex |= 2;
    if (Grid.val[2] < 0.0)
        CubeIndex |= 4;
    if (Grid.val[3] < 0.0)
        CubeIndex |= 8;
    if (Grid.val[4] < 0.0)
        CubeIndex |= 16;
    if (Grid.val[5] < 0.0)
        CubeIndex |= 32;
    if (Grid.val[6] < 0.0)
        CubeIndex |= 64;
    if (Grid.val[7] < 0.0)
        CubeIndex |= 128;

    // printf("%f %f %f %f %f %f %f %f %d\n", Grid.val[0], Grid.val[1], Grid.val[2], Grid.val[3], Grid.val[4], Grid.val[5], Grid.val[6], Grid.val[7],
    // CubeIndex);

    // Cube is entirely in/out of the surface
    if (edgeTable[CubeIndex] == 0)
        return (0);

    // Find the vertices where the surface intersects the cube
    if (edgeTable[CubeIndex] & 1)
        VertexList[0] = VertexInterp(Grid.p[0], Grid.p[1], Grid.val[0], Grid.val[1]);
    if (edgeTable[CubeIndex] & 2)
        VertexList[1] = VertexInterp(Grid.p[1], Grid.p[2], Grid.val[1], Grid.val[2]);
    if (edgeTable[CubeIndex] & 4)
        VertexList[2] = VertexInterp(Grid.p[2], Grid.p[3], Grid.val[2], Grid.val[3]);
    if (edgeTable[CubeIndex] & 8)
        VertexList[3] = VertexInterp(Grid.p[3], Grid.p[0], Grid.val[3], Grid.val[0]);
    if (edgeTable[CubeIndex] & 16)
        VertexList[4] = VertexInterp(Grid.p[4], Grid.p[5], Grid.val[4], Grid.val[5]);
    if (edgeTable[CubeIndex] & 32)
        VertexList[5] = VertexInterp(Grid.p[5], Grid.p[6], Grid.val[5], Grid.val[6]);
    if (edgeTable[CubeIndex] & 64)
        VertexList[6] = VertexInterp(Grid.p[6], Grid.p[7], Grid.val[6], Grid.val[7]);
    if (edgeTable[CubeIndex] & 128)
        VertexList[7] = VertexInterp(Grid.p[7], Grid.p[4], Grid.val[7], Grid.val[4]);
    if (edgeTable[CubeIndex] & 256)
        VertexList[8] = VertexInterp(Grid.p[0], Grid.p[4], Grid.val[0], Grid.val[4]);
    if (edgeTable[CubeIndex] & 512)
        VertexList[9] = VertexInterp(Grid.p[1], Grid.p[5], Grid.val[1], Grid.val[5]);
    if (edgeTable[CubeIndex] & 1024)
        VertexList[10] = VertexInterp(Grid.p[2], Grid.p[6], Grid.val[2], Grid.val[6]);
    if (edgeTable[CubeIndex] & 2048)
        VertexList[11] = VertexInterp(Grid.p[3], Grid.p[7], Grid.val[3], Grid.val[7]);

    NewVertexCount = 0;
    for (uint i = 0; i < 12; i++)
        LocalRemap[i] = -1;

    for (uint i = 0; triTable[CubeIndex][i] != -1; i++) {
        if (LocalRemap[triTable[CubeIndex][i]] == -1) {
            NewVertexList[NewVertexCount] = VertexList[triTable[CubeIndex][i]];
            LocalRemap[triTable[CubeIndex][i]] = NewVertexCount;
            NewVertexCount++;
        }
    }

    for (uint i = 0; i < NewVertexCount; i++) {
        Vertices[i] = NewVertexList[i];
    }

    TriangleCount = 0;

    for (uint i = 0; triTable[CubeIndex][i] != -1; i += 3) {
        Triangles[TriangleCount] =
            chrono::vec3(LocalRemap[triTable[CubeIndex][i + 0]], LocalRemap[triTable[CubeIndex][i + 1]], LocalRemap[triTable[CubeIndex][i + 2]]);
        TriangleCount++;
    }

    return (TriangleCount);
}

void Weld(std::vector<uint>& meshIndices, std::vector<real3>& meshVertices, std::vector<real3>& meshNormals) {
    printf("Welding\n");
    meshIndices.resize(meshVertices.size());
    meshNormals.resize(meshVertices.size());

#pragma omp parallel for
    for (uint i = 0; i < meshVertices.size(); i++) {
        meshVertices[i].x = Round(meshVertices[i].x / round_to_nearest) * round_to_nearest;
        meshVertices[i].y = Round(meshVertices[i].y / round_to_nearest) * round_to_nearest;
        meshVertices[i].z = Round(meshVertices[i].z / round_to_nearest) * round_to_nearest;
    }
    std::vector<chrono::real3> vertices = meshVertices;
    thrust::sort(vertices.begin(), vertices.end());
    vertices.erase(thrust::unique(vertices.begin(), vertices.end()), vertices.end());
    thrust::lower_bound(vertices.begin(), vertices.end(), meshVertices.begin(), meshVertices.end(), meshIndices.begin());
    meshNormals.resize(vertices.size());

#pragma omp parallel for
    for (uint i = 0; i < vertices.size(); i++) {
        meshNormals[i] = GetNormal(vertices[i]);
    }
    uint start_triangles = meshIndices.size() / 3;
    std::vector<chrono::uvec3> triangles(meshIndices.size() / 3);
#pragma omp parallel for
    for (uint i = 0; i < triangles.size(); i++) {
        triangles[i].x = meshIndices[i * 3 + 0];
        triangles[i].y = meshIndices[i * 3 + 1];
        triangles[i].z = meshIndices[i * 3 + 2];
        // triangles[i]=Sort(triangles[i]);
    }

    thrust::sort(triangles.begin(), triangles.end());

    uint num_triangles = thrust::unique(triangles.begin(), triangles.end()) - triangles.begin();
    meshIndices.resize(num_triangles * 3);

#pragma omp parallel for
    for (uint i = 0; i < num_triangles; i++) {
        meshIndices[i * 3 + 0] = triangles[i].x + 1;
        meshIndices[i * 3 + 1] = triangles[i].y + 1;
        meshIndices[i * 3 + 2] = triangles[i].z + 1;
    }

    printf("Welding: vert: input: %d output: %d, tri: inpue: %d output: %d tolerance: %f\n", meshVertices.size(), vertices.size(), start_triangles,
           num_triangles, round_to_nearest);
    meshVertices = vertices;
}

//    std::ofstream os("Vol.vol");
//
//    os.write("VOL", 3);
//    char version = 3;
//    os.write((char*)&version, sizeof(char));
//    uint value = 1;
//    os.write((char*)&value, sizeof(uint));
//    os.write((char*)&bins_per_axis.x, sizeof(uint));
//    os.write((char*)&bins_per_axis.y, sizeof(uint));
//    os.write((char*)&bins_per_axis.z, sizeof(uint));
//    value = 1;
//    os.write((char*)&value, sizeof(uint));
//
//    real minX = min_bounding_point.x;
//    real minY = min_bounding_point.y;
//    real minZ = min_bounding_point.z;
//
//    real maxX = max_bounding_point.x;
//    real maxY = max_bounding_point.y;
//    real maxZ = max_bounding_point.z;
//
//    os.write((char*)&minX, sizeof(real));
//    os.write((char*)&minY, sizeof(real));
//    os.write((char*)&minZ, sizeof(real));
//    os.write((char*)&maxX, sizeof(real));
//    os.write((char*)&maxY, sizeof(real));
//    os.write((char*)&maxZ, sizeof(real));
//
//    for (uint nod = 0; nod < grid_size; nod++) {
//        real mass = node_mass[nod];
//
//        os.write((char*)&mass, sizeof(real));
//    }
//    os.close();
//}

chrono::vec3 Hash_Decode(uint hash) {
    chrono::vec3 decoded_hash;
    decoded_hash.x = hash % (bins_per_axis.x * bins_per_axis.y) % bins_per_axis.x;
    decoded_hash.y = (hash % (bins_per_axis.x * bins_per_axis.y)) / bins_per_axis.x;
    decoded_hash.z = hash / (bins_per_axis.x * bins_per_axis.y);
    return decoded_hash;
}

void ComputeBoundary(std::vector<real3>& pos_marker,
                     real kernel_radius,
                     std::vector<real3>& meshVertices,
                     std::vector<real3>& meshNormals,
                     std::vector<uint>& meshIndices) {
    bbox res(pos_marker[0], pos_marker[0]);
    bbox_transformation unary_op;
    bbox_reduction binary_op;
    res = thrust::transform_reduce(pos_marker.begin(), pos_marker.end(), unary_op, res, binary_op);

    max_bounding_point = real3((Ceil(res.second.x), (res.second.x + kernel_radius * 12)), (Ceil(res.second.y), (res.second.y + kernel_radius * 12)),
                               (Ceil(res.second.z), (res.second.z + kernel_radius * 12)));

    min_bounding_point = real3((Floor(res.first.x), (res.first.x - kernel_radius * 12)), (Floor(res.first.y), (res.first.y - kernel_radius * 12)),
                               (Floor(res.first.z), (res.first.z - kernel_radius * 12)));

    min_bounding_point.x = Max(abs_min.x, min_bounding_point.x);
    min_bounding_point.y = Max(abs_min.y, min_bounding_point.y);
    min_bounding_point.z = Max(abs_min.z, min_bounding_point.z);

    max_bounding_point.x = Min(abs_max.x, max_bounding_point.x);
    max_bounding_point.y = Min(abs_max.y, max_bounding_point.y);
    max_bounding_point.z = Min(abs_max.z, max_bounding_point.z);

    diag = max_bounding_point - min_bounding_point;
    bin_edge = kernel_radius;
    bins_per_axis = chrono::vec3(diag / bin_edge);
    inv_bin_edge = real(1.) / bin_edge;
    grid_size = bins_per_axis.x * bins_per_axis.y * bins_per_axis.z;
    uint num_spheres = pos_marker.size();

    printf("max_bounding_point [%f %f %f]\n", max_bounding_point.x, max_bounding_point.y, max_bounding_point.z);
    printf("min_bounding_point [%f %f %f]\n", min_bounding_point.x, min_bounding_point.y, min_bounding_point.z);

    //    real3 center = (min_bounding_point + max_bounding_point) * .5;
    //
    //    printf("center [%f %f %f]\n", center.x, center.y, center.z);
    //    printf("diag [%f %f %f]\n", diag.x, diag.y, diag.z);

    printf("Compute DOF [%d] [%d %d %d] [%f] %d\n", grid_size, bins_per_axis.x, bins_per_axis.y, bins_per_axis.z, bin_edge, num_spheres);

    node_num.resize(grid_size);
    node_loc.resize(grid_size);
    node_mass.resize(grid_size);

    std::fill(node_mass.begin(), node_mass.end(), 0);
#pragma omp parallel for
    for (uint p = 0; p < num_spheres; p++) {
        const real3 xi = pos_marker[p];
        if (xi.x < min_bounding_point.x || xi.y < min_bounding_point.y || xi.z < min_bounding_point.z) {
            continue;
        }
        if (xi.x > max_bounding_point.x || xi.y > max_bounding_point.y || xi.z > max_bounding_point.z) {
            continue;
        }

        const int cx = GridCoord(xi.x, inv_bin_edge, min_bounding_point.x);
        const int cy = GridCoord(xi.y, inv_bin_edge, min_bounding_point.y);
        const int cz = GridCoord(xi.z, inv_bin_edge, min_bounding_point.z);

        for (int i = cx - 2; i <= cx + 2; ++i) {
            for (int j = cy - 2; j <= cy + 2; ++j) {
                for (int k = cz - 2; k <= cz + 2; ++k) {
                    const int current_node = GridHash(i, j, k, bins_per_axis);
                    real3 current_node_location = NodeLocation(i, j, k, bin_edge, min_bounding_point);
#pragma omp atomic
                    node_mass[current_node] += N(xi - current_node_location, inv_bin_edge);
                }
            }
        }
    }

    real sum = std::accumulate(node_mass.begin(), node_mass.end(), 0.0);
    real mean = sum / node_mass.size();
    real sq_sum = std::inner_product(node_mass.begin(), node_mass.end(), node_mass.begin(), 0.0);
    real stdev = std::sqrt(sq_sum / node_mass.size() - mean * mean);

    real max_mass = *std::max_element(node_mass.begin(), node_mass.end());

    printf("Max: %f %f %f\n", max_mass, mean, stdev);

#pragma omp parallel for
    for (uint nod = 0; nod < grid_size; nod++) {
        vec3 node_n = GridDecode(nod, bins_per_axis);
        node_num[nod] = node_n;
        node_loc[nod] = NodeLocation(node_n.x, node_n.y, node_n.z, bin_edge, min_bounding_point);
        node_mass[nod] = node_mass[nod] - (mean + stdev * .5);
    }

    uint NewVertexCount;

    for (uint nod = 0; nod < grid_size; nod++) {
        // if (node_mass[nod] != -(mean + stdev * .1))
        {
            chrono::vec3 node_index = node_num[nod];
            real3 node_location = node_loc[nod];

            uint idx = node_index.x;
            uint idy = node_index.y;
            uint idz = node_index.z;

            real px = node_location.x;
            real py = node_location.y;
            real pz = node_location.z;

            uint mxnum = bins_per_axis.x;
            uint mynum = bins_per_axis.y;
            uint mznum = bins_per_axis.z;

            real3 vertlist[12];
            long edgelist[12];
            GRIDCELL g;

            g.p[0] = real3(px, py, pz);
            g.p[1] = real3(px + bin_edge, py, pz);
            g.p[2] = real3(px + bin_edge, py, pz + bin_edge);
            g.p[3] = real3(px, py, pz + bin_edge);
            g.p[4] = real3(px, py + bin_edge, pz);
            g.p[5] = real3(px + bin_edge, py + bin_edge, pz);
            g.p[6] = real3(px + bin_edge, py + bin_edge, pz + bin_edge);
            g.p[7] = real3(px, py + bin_edge, pz + bin_edge);

            g.val[0] = node_mass[GetDensity(g.p[0])];
            g.val[1] = node_mass[GetDensity(g.p[1])];
            g.val[2] = node_mass[GetDensity(g.p[2])];
            g.val[3] = node_mass[GetDensity(g.p[3])];
            g.val[4] = node_mass[GetDensity(g.p[4])];
            g.val[5] = node_mass[GetDensity(g.p[5])];
            g.val[6] = node_mass[GetDensity(g.p[6])];
            g.val[7] = node_mass[GetDensity(g.p[7])];

            uint triangles = Polygonise(g, _FaceStorage, NewVertexCount, _VertexStorage);

            if (triangles) {
                for (uint i = 0; i < triangles; i++) {
                    chrono::vec3 face = _FaceStorage[i] + chrono::vec3(meshVertices.size() + 1);

                    //                    meshIndices.push_back(3 + meshVertices.size());
                    //                    meshIndices.push_back(2 + meshVertices.size());
                    //                    meshIndices.push_back(1 + meshVertices.size());

                    meshVertices.push_back(_VertexStorage[_FaceStorage[i].z]);
                    meshVertices.push_back(_VertexStorage[_FaceStorage[i].y]);
                    meshVertices.push_back(_VertexStorage[_FaceStorage[i].x]);

                    //                    meshNormals.push_back(GetNormal( _VertexStorage[_FaceStorage[i].z], bins_per_axis, bin_edge, inv_bin_edge,
                    //                    min_bounding_point));
                    //                    meshNormals.push_back(GetNormal( _VertexStorage[_FaceStorage[i].y], bins_per_axis, bin_edge, inv_bin_edge,
                    //                    min_bounding_point));
                    //                    meshNormals.push_back(GetNormal( _VertexStorage[_FaceStorage[i].x], bins_per_axis, bin_edge, inv_bin_edge,
                    //                    min_bounding_point));
                }
            }
        };
    }
    Weld(meshIndices, meshVertices, meshNormals);
}

void WriteMeshToFile(std::string filename, std::vector<real3>& meshVertices, std::vector<real3>& meshNormals, std::vector<uint>& meshIndices) {
    printf("Writing: %s\n", filename.c_str());
    std::ofstream ofile(filename);

    for (uint i = 0; i < meshVertices.size(); i++) {
        ofile << "v " << meshVertices[i].x << " " << meshVertices[i].y << " " << meshVertices[i].z << std::endl;
    }
    for (uint i = 0; i < meshNormals.size(); i++) {
        ofile << "vn " << meshNormals[i].x << " " << meshNormals[i].y << " " << meshNormals[i].z << std::endl;
    }
    for (uint i = 0; i < meshIndices.size() / 3; i++) {
        ofile << "f " << meshIndices[i * 3 + 0] << " " << meshIndices[i * 3 + 1] << " " << meshIndices[i * 3 + 2] << std::endl;
    }

    ofile.close();
}

void MarchingCubesToMesh(std::vector<real3>& position, real kernel_radius, std::string filename, real3 minp, real3 maxp, real round_to) {
    std::vector<chrono::real3> meshVertices;
    std::vector<uint> meshIndices;
    std::vector<chrono::real3> meshNormals;
    abs_min = minp;
    abs_max = maxp;
    round_to_nearest = round_to;
    ComputeBoundary(position, kernel_radius, meshVertices, meshNormals, meshIndices);
    WriteMeshToFile(filename, meshVertices, meshNormals, meshIndices);
}
