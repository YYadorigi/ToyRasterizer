#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include <array>
#include <Eigen/Dense>
#include "tgaimage.hpp"

class Model
{
private:
    std::vector<Eigen::Vector3f> vert;
    std::vector<Eigen::Vector3f> vert_world_coords;
    std::vector<Eigen::Vector3f> text;
    std::vector<Eigen::Vector3f> norm;
    std::vector<std::array<Eigen::Vector3i, 3>> ind; // indexed triangle mesh (vert, text, norm)
    TGAImage texture;

    void load_texture(std::string filename, const char *suffix);

public:
    Model(const char *filename);

    ~Model();

    int num_verts() const;

    int num_faces() const;

    Eigen::Vector3f get_vert(int idx) const;

    Eigen::Vector3f get_vert_world_coords(int idx) const;

    Eigen::Vector3f get_texture(int idx) const;

    Eigen::Vector3f get_normal(int idx) const;

    std::array<Eigen::Vector3i, 3> get_face(int idx) const;

    TGAColor get_texture(Eigen::Vector3f uv) const;

    void transform(const Eigen::Matrix4f &m);
};

#endif //__MODEL_H__