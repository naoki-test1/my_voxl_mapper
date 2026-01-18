// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// Mesh output taken from open_chisel: github.com/personalrobotics/OpenChisel

#ifndef VOXBLOX_MESH_VIS_H_
#define VOXBLOX_MESH_VIS_H_

#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <modal_pipe.h>
#include "conversions.h"
#include <string>

#define MESH_MAGIC_NUMBER (0x4d455348)
typedef struct mesh_metadata_t
{
    uint32_t magic_number; ///< Unique 32-bit number used to signal the beginning of a VIO packet while parsing a data stream.
    int64_t timestamp_ns;  ///< timestamp at the middle of the frame exposure in monotonic time
    uint64_t size_bytes;   ///< Total size of following data (num_blocks number of blocks, each containing metadata and array of vertices
    uint32_t num_vertices;
    uint32_t num_indices;
} __attribute__((packed)) mesh_metadata_t;

typedef struct mesh_vertex_t
{
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
} __attribute__((packed)) mesh_vertex_t;

typedef struct mesh_index_t
{
    int32_t indices[3];
} __attribute__((packed)) mesh_index_t;

namespace voxblox
{

    enum class ColorMode
    {
        kPointcloud,
        kHeight,
        kNormals
    };
    
    enum class ColorMap
    {
        kViridis,
        kGithub,
    };

    // Follows githubs commit coloring (looks like VOXL portal color scheme)
    constexpr float github[] =
        {
            0.933333, 0.933333, 0.933333,
            0.776470, 0.894117, 0.545098,
            0.482352, 0.788235, 0.435294,
            0.137254, 0.603921, 0.231372,
            0.098039, 0.380392, 0.152941};

    constexpr float viridis[] =
        {
            0.267004, 0.004874, 0.329415,
            0.268510, 0.009605, 0.335427,
            0.269944, 0.014625, 0.341379,
            0.271305, 0.019942, 0.347269,
            0.272594, 0.025563, 0.353093,
            0.273809, 0.031497, 0.358853,
            0.274952, 0.037752, 0.364543,
            0.276022, 0.044167, 0.370164,
            0.277018, 0.050344, 0.375715,
            0.277941, 0.056324, 0.381191,
            0.278791, 0.062145, 0.386592,
            0.279566, 0.067836, 0.391917,
            0.280267, 0.073417, 0.397163,
            0.280894, 0.078907, 0.402329,
            0.281446, 0.084320, 0.407414,
            0.281924, 0.089666, 0.412415,
            0.282327, 0.094955, 0.417331,
            0.282656, 0.100196, 0.422160,
            0.282910, 0.105393, 0.426902,
            0.283091, 0.110553, 0.431554,
            0.283197, 0.115680, 0.436115,
            0.283229, 0.120777, 0.440584,
            0.283187, 0.125848, 0.444960,
            0.283072, 0.130895, 0.449241,
            0.282884, 0.135920, 0.453427,
            0.282623, 0.140926, 0.457517,
            0.282290, 0.145912, 0.461510,
            0.281887, 0.150881, 0.465405,
            0.281412, 0.155834, 0.469201,
            0.280868, 0.160771, 0.472899,
            0.280255, 0.165693, 0.476498,
            0.279574, 0.170599, 0.479997,
            0.278826, 0.175490, 0.483397,
            0.278012, 0.180367, 0.486697,
            0.277134, 0.185228, 0.489898,
            0.276194, 0.190074, 0.493001,
            0.275191, 0.194905, 0.496005,
            0.274128, 0.199721, 0.498911,
            0.273006, 0.204520, 0.501721,
            0.271828, 0.209303, 0.504434,
            0.270595, 0.214069, 0.507052,
            0.269308, 0.218818, 0.509577,
            0.267968, 0.223549, 0.512008,
            0.266580, 0.228262, 0.514349,
            0.265145, 0.232956, 0.516599,
            0.263663, 0.237631, 0.518762,
            0.262138, 0.242286, 0.520837,
            0.260571, 0.246922, 0.522828,
            0.258965, 0.251537, 0.524736,
            0.257322, 0.256130, 0.526563,
            0.255645, 0.260703, 0.528312,
            0.253935, 0.265254, 0.529983,
            0.252194, 0.269783, 0.531579,
            0.250425, 0.274290, 0.533103,
            0.248629, 0.278775, 0.534556,
            0.246811, 0.283237, 0.535941,
            0.244972, 0.287675, 0.537260,
            0.243113, 0.292092, 0.538516,
            0.241237, 0.296485, 0.539709,
            0.239346, 0.300855, 0.540844,
            0.237441, 0.305202, 0.541921,
            0.235526, 0.309527, 0.542944,
            0.233603, 0.313828, 0.543914,
            0.231674, 0.318106, 0.544834,
            0.229739, 0.322361, 0.545706,
            0.227802, 0.326594, 0.546532,
            0.225863, 0.330805, 0.547314,
            0.223925, 0.334994, 0.548053,
            0.221989, 0.339161, 0.548752,
            0.220057, 0.343307, 0.549413,
            0.218130, 0.347432, 0.550038,
            0.216210, 0.351535, 0.550627,
            0.214298, 0.355619, 0.551184,
            0.212395, 0.359683, 0.551710,
            0.210503, 0.363727, 0.552206,
            0.208623, 0.367752, 0.552675,
            0.206756, 0.371758, 0.553117,
            0.204903, 0.375746, 0.553533,
            0.203063, 0.379716, 0.553925,
            0.201239, 0.383670, 0.554294,
            0.199430, 0.387607, 0.554642,
            0.197636, 0.391528, 0.554969,
            0.195860, 0.395433, 0.555276,
            0.194100, 0.399323, 0.555565,
            0.192357, 0.403199, 0.555836,
            0.190631, 0.407061, 0.556089,
            0.188923, 0.410910, 0.556326,
            0.187231, 0.414746, 0.556547,
            0.185556, 0.418570, 0.556753,
            0.183898, 0.422383, 0.556944,
            0.182256, 0.426184, 0.557120,
            0.180629, 0.429975, 0.557282,
            0.179019, 0.433756, 0.557430,
            0.177423, 0.437527, 0.557565,
            0.175841, 0.441290, 0.557685,
            0.174274, 0.445044, 0.557792,
            0.172719, 0.448791, 0.557885,
            0.171176, 0.452530, 0.557965,
            0.169646, 0.456262, 0.558030,
            0.168126, 0.459988, 0.558082,
            0.166617, 0.463708, 0.558119,
            0.165117, 0.467423, 0.558141,
            0.163625, 0.471133, 0.558148,
            0.162142, 0.474838, 0.558140,
            0.160665, 0.478540, 0.558115,
            0.159194, 0.482237, 0.558073,
            0.157729, 0.485932, 0.558013,
            0.156270, 0.489624, 0.557936,
            0.154815, 0.493313, 0.557840,
            0.153364, 0.497000, 0.557724,
            0.151918, 0.500685, 0.557587,
            0.150476, 0.504369, 0.557430,
            0.149039, 0.508051, 0.557250,
            0.147607, 0.511733, 0.557049,
            0.146180, 0.515413, 0.556823,
            0.144759, 0.519093, 0.556572,
            0.143343, 0.522773, 0.556295,
            0.141935, 0.526453, 0.555991,
            0.140536, 0.530132, 0.555659,
            0.139147, 0.533812, 0.555298,
            0.137770, 0.537492, 0.554906,
            0.136408, 0.541173, 0.554483,
            0.135066, 0.544853, 0.554029,
            0.133743, 0.548535, 0.553541,
            0.132444, 0.552216, 0.553018,
            0.131172, 0.555899, 0.552459,
            0.129933, 0.559582, 0.551864,
            0.128729, 0.563265, 0.551229,
            0.127568, 0.566949, 0.550556,
            0.126453, 0.570633, 0.549841,
            0.125394, 0.574318, 0.549086,
            0.124395, 0.578002, 0.548287,
            0.123463, 0.581687, 0.547445,
            0.122606, 0.585371, 0.546557,
            0.121831, 0.589055, 0.545623,
            0.121148, 0.592739, 0.544641,
            0.120565, 0.596422, 0.543611,
            0.120092, 0.600104, 0.542530,
            0.119738, 0.603785, 0.541400,
            0.119512, 0.607464, 0.540218,
            0.119423, 0.611141, 0.538982,
            0.119483, 0.614817, 0.537692,
            0.119699, 0.618490, 0.536347,
            0.120081, 0.622161, 0.534946,
            0.120638, 0.625828, 0.533488,
            0.121380, 0.629492, 0.531973,
            0.122312, 0.633153, 0.530398,
            0.123444, 0.636809, 0.528763,
            0.124780, 0.640461, 0.527068,
            0.126326, 0.644107, 0.525311,
            0.128087, 0.647749, 0.523491,
            0.130067, 0.651384, 0.521608,
            0.132268, 0.655014, 0.519661,
            0.134692, 0.658636, 0.517649,
            0.137339, 0.662252, 0.515571,
            0.140210, 0.665859, 0.513427,
            0.143303, 0.669459, 0.511215,
            0.146616, 0.673050, 0.508936,
            0.150148, 0.676631, 0.506589,
            0.153894, 0.680203, 0.504172,
            0.157851, 0.683765, 0.501686,
            0.162016, 0.687316, 0.499129,
            0.166383, 0.690856, 0.496502,
            0.170948, 0.694384, 0.493803,
            0.175707, 0.697900, 0.491033,
            0.180653, 0.701402, 0.488189,
            0.185783, 0.704891, 0.485273,
            0.191090, 0.708366, 0.482284,
            0.196571, 0.711827, 0.479221,
            0.202219, 0.715272, 0.476084,
            0.208030, 0.718701, 0.472873,
            0.214000, 0.722114, 0.469588,
            0.220124, 0.725509, 0.466226,
            0.226397, 0.728888, 0.462789,
            0.232815, 0.732247, 0.459277,
            0.239374, 0.735588, 0.455688,
            0.246070, 0.738910, 0.452024,
            0.252899, 0.742211, 0.448284,
            0.259857, 0.745492, 0.444467,
            0.266941, 0.748751, 0.440573,
            0.274149, 0.751988, 0.436601,
            0.281477, 0.755203, 0.432552,
            0.288921, 0.758394, 0.428426,
            0.296479, 0.761561, 0.424223,
            0.304148, 0.764704, 0.419943,
            0.311925, 0.767822, 0.415586,
            0.319809, 0.770914, 0.411152,
            0.327796, 0.773980, 0.406640,
            0.335885, 0.777018, 0.402049,
            0.344074, 0.780029, 0.397381,
            0.352360, 0.783011, 0.392636,
            0.360741, 0.785964, 0.387814,
            0.369214, 0.788888, 0.382914,
            0.377779, 0.791781, 0.377939,
            0.386433, 0.794644, 0.372886,
            0.395174, 0.797475, 0.367757,
            0.404001, 0.800275, 0.362552,
            0.412913, 0.803041, 0.357269,
            0.421908, 0.805774, 0.351910,
            0.430983, 0.808473, 0.346476,
            0.440137, 0.811138, 0.340967,
            0.449368, 0.813768, 0.335384,
            0.458674, 0.816363, 0.329727,
            0.468053, 0.818921, 0.323998,
            0.477504, 0.821444, 0.318195,
            0.487026, 0.823929, 0.312321,
            0.496615, 0.826376, 0.306377,
            0.506271, 0.828786, 0.300362,
            0.515992, 0.831158, 0.294279,
            0.525776, 0.833491, 0.288127,
            0.535621, 0.835785, 0.281908,
            0.545524, 0.838039, 0.275626,
            0.555484, 0.840254, 0.269281,
            0.565498, 0.842430, 0.262877,
            0.575563, 0.844566, 0.256415,
            0.585678, 0.846661, 0.249897,
            0.595839, 0.848717, 0.243329,
            0.606045, 0.850733, 0.236712,
            0.616293, 0.852709, 0.230052,
            0.626579, 0.854645, 0.223353,
            0.636902, 0.856542, 0.216620,
            0.647257, 0.858400, 0.209861,
            0.657642, 0.860219, 0.203082,
            0.668054, 0.861999, 0.196293,
            0.678489, 0.863742, 0.189503,
            0.688944, 0.865448, 0.182725,
            0.699415, 0.867117, 0.175971,
            0.709898, 0.868751, 0.169257,
            0.720391, 0.870350, 0.162603,
            0.730889, 0.871916, 0.156029,
            0.741388, 0.873449, 0.149561,
            0.751884, 0.874951, 0.143228,
            0.762373, 0.876424, 0.137064,
            0.772852, 0.877868, 0.131109,
            0.783315, 0.879285, 0.125405,
            0.793760, 0.880678, 0.120005,
            0.804182, 0.882046, 0.114965,
            0.814576, 0.883393, 0.110347,
            0.824940, 0.884720, 0.106217,
            0.835270, 0.886029, 0.102646,
            0.845561, 0.887322, 0.099702,
            0.855810, 0.888601, 0.097452,
            0.866013, 0.889868, 0.095953,
            0.876168, 0.891125, 0.095250,
            0.886271, 0.892374, 0.095374,
            0.896320, 0.893616, 0.096335,
            0.906311, 0.894855, 0.098125,
            0.916242, 0.896091, 0.100717,
            0.926106, 0.897330, 0.104071,
            0.935904, 0.898570, 0.108131,
            0.945636, 0.899815, 0.112838,
            0.955300, 0.901065, 0.118128,
            0.964894, 0.902323, 0.123941,
            0.974417, 0.903590, 0.130215,
            0.983868, 0.904867, 0.136897,
            0.993248, 0.906157, 0.143936};

    static std::string color_mode_string = "normals";
    static std::string color_map_string = "viridis";

    inline void getVertexColor(mesh_vertex_t *vertex, const voxblox::Mesh &mesh, const int index, const ColorMode color_mode, const ColorMap color_map)
    {
        const float rainbow_repeat_dist = 3.0f;

        switch (color_mode)
        {
        case ColorMode::kPointcloud:
        {

            vertex->r = mesh.colors[index].r;
            vertex->g = mesh.colors[index].g;
            vertex->b = mesh.colors[index].b;
            break;
        }

        case ColorMode::kHeight:
        {
            float height = std::fabs(mesh.vertices[index].z() / rainbow_repeat_dist);
            int num_of_repeats = height;
            height -= num_of_repeats;

            if (num_of_repeats % 2 == 0)
            {
                height = 1 - height;
            }

            const float *cmap_data;
            float a = 0;

            switch (color_map)
            {
                case ColorMap::kViridis:
                    cmap_data = viridis;
                    a = height * 254.0f;
                    break;
                case ColorMap::kGithub:
                    cmap_data = github;
                    a = height * 4.0f;
                    break;
            }

            int color0 = std::floor(a);
            int color1 = std::ceil(a);
            float t = a - color0;

            vertex->r = 255 * ((1 - t) * cmap_data[color0 * 3] + t * cmap_data[color1 * 3]);
            vertex->g = 255 * ((1 - t) * cmap_data[color0 * 3 + 1] + t * cmap_data[color1 * 3 + 1]);
            vertex->b = 255 * ((1 - t) * cmap_data[color0 * 3 + 2] + t * cmap_data[color1 * 3 + 2]);
            break;
        }

        case ColorMode::kNormals:
        {

            // Normals should be in scale -1 to 1, so shift to 0 - 255
            vertex->r = 255 - (255 * (mesh.normals[index].x() * 0.5f + 0.5f));
            vertex->g = 255 - (255 * (mesh.normals[index].y() * 0.5f + 0.5f));
            vertex->b = 255 - (255 * (mesh.normals[index].z() * 0.5f + 0.5f));
            break;
        }
        }
    }

    inline ColorMode getColorModeFromString(const std::string &color_mode_string)
    {
        if (color_mode_string == "pointcloud")
        {
            return ColorMode::kPointcloud;
        }
        else if (color_mode_string == "height")
        {
            return ColorMode::kHeight;
        }
        else if (color_mode_string == "normals")
        {
            return ColorMode::kNormals;
        }
        else
        { // Default case is height.
            return ColorMode::kHeight;
        }
    }

    inline ColorMap getColorMapFromString(const std::string &color_map_string)
    {
        if (color_map_string == "viridis")
        {
            return ColorMap::kViridis;
        }
        else if (color_map_string == "github")
        {
            return ColorMap::kGithub;
        }
        else
        { // Default case is viridis.
            return ColorMap::kViridis;
        }
    }

    inline void generateVoxbloxMeshMsg(int ch, MeshLayer *mesh_layer, voxblox_msgs::Mesh *mesh_msg)
    {
        if (mesh_layer == NULL)
        {
            fprintf(stderr, "mesh_layer is null\n");
            return;
        }
        if (mesh_msg == NULL)
        {
            fprintf(stderr, "mesh_msg is null\n");
            return;
        }

        voxblox::Mesh connected_mesh;
        mesh_layer->getConnectedMesh(&connected_mesh);

        static mesh_metadata_t meta;

        // Setup the metadata
        meta.magic_number = MESH_MAGIC_NUMBER;
        meta.timestamp_ns = monotonic_time();
        meta.size_bytes = 0;
        meta.num_vertices = connected_mesh.vertices.size();
        meta.num_indices = connected_mesh.indices.size() / 3;
        meta.size_bytes += meta.num_vertices * sizeof(mesh_vertex_t) + meta.num_indices * sizeof(mesh_index_t);

        // Allocate the memory
        void *data = malloc(meta.size_bytes + sizeof(mesh_metadata_t));
        memcpy(data, &meta, sizeof(mesh_metadata_t));
        char *current = (char *)data + sizeof(mesh_metadata_t);

        ColorMode color_mode = getColorModeFromString(color_mode_string);
        ColorMap color_map = getColorMapFromString(color_map_string);

        for (size_t i = 0; i < connected_mesh.vertices.size(); i++)
        {
            mesh_vertex_t *vertex = (mesh_vertex_t *)current;
            current += sizeof(mesh_vertex_t);

            // push up our vertices
            vertex->x = connected_mesh.vertices[i].x();
            vertex->y = connected_mesh.vertices[i].y();
            vertex->z = connected_mesh.vertices[i].z();

            // push up our colors
            getVertexColor(vertex, connected_mesh, i, color_mode, color_map);
        }

        for (size_t i = 0; i < connected_mesh.indices.size(); i += 3)
        {
            mesh_index_t *index = (mesh_index_t *)current;
            current += sizeof(mesh_index_t);

            // push up our vertices
            index->indices[0] = connected_mesh.indices[i];
            index->indices[1] = connected_mesh.indices[i + 1];
            index->indices[2] = connected_mesh.indices[i + 2];
        }
        pipe_server_write(ch, data, meta.size_bytes + sizeof(mesh_metadata_t));

        free(data);
    }

    // A hash function used to hash a pair of any kind
    struct hash_pair
    {
        template <class T1, class T2>
        size_t operator()(const std::pair<T1, T2> &p) const
        {
            auto hash1 = std::hash<T1>{}(p.first);
            auto hash2 = std::hash<T2>{}(p.second);
            return hash1 ^ hash2;
        }
    };

    inline void createCostmapFromLayer(const Layer<EsdfVoxel> &layer, const unsigned int &free_plane_index, float &free_plane_val, std::unordered_map<std::pair<float, float>, float, hash_pair> &cost_map, bool &only_updates)
    {
        BlockIndexList blocks;
        if (only_updates)
            layer.getAllUpdatedBlocks(Update::kMap, &blocks);
        else
        {
            cost_map.clear();
            layer.getAllAllocatedBlocks(&blocks);
        }

        if (blocks.size() == 0)
            return;

        // Cache layer settings.
        size_t vps = layer.voxels_per_side();
        size_t num_voxels_per_block = vps * vps * vps;

        // Iterate over all blocks.
        for (const BlockIndex &index : blocks)
        {
            // Iterate over all voxels in said blocks.
            const Block<EsdfVoxel> &block = layer.getBlockByIndex(index);

            // Only keep blocks that are at this height
            Point origin = block.origin();
            if (std::abs(origin(free_plane_index) - free_plane_val) > block.block_size())
            {
                continue;
            }

            for (size_t linear_index = 0; linear_index < num_voxels_per_block; ++linear_index)
            {
                Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
                // not if the original gets modified by the cast
                float x = coord.x();
                float y = coord.y();
                std::pair<float, float> curr_coords = std::make_pair(x, y);
                float distance = 10.0;

                const EsdfVoxel voxel = block.getVoxelByLinearIndex(linear_index);
                if (std::abs(coord(free_plane_index) - free_plane_val) <= block.voxel_size())
                {
                    if (voxel.observed || voxel.hallucinated)
                    {
                        distance = (float)std::abs(voxel.distance);
                    }

                    cost_map[curr_coords] = distance;
                }
            }
        }
        return;
    }

    inline void create2DCostmap(const Layer<EsdfVoxel> &layer, float &start_height, std::unordered_map<std::pair<float, float>, float, hash_pair> &cost_map, bool &only_updates)
    {
        static const unsigned int free_plane_index = 2;

        if (std::remainder(start_height, layer.voxel_size()) < 1e-6f)
        {
            start_height += layer.voxel_size() / 2.0f;
        }
        createCostmapFromLayer(layer, free_plane_index, start_height, cost_map, only_updates);

        return;
    }

} // namespace voxblox

#endif // VOXBLOX_MESH_VIS_H_
