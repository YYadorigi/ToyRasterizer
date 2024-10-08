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
    TGAImage diffuse_texture;
    TGAImage specular_texture;
    TGAImage normal_texture;

    void load_texture(std::string filename);

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

    Eigen::Vector3f get_diffuse(Eigen::Vector3f uv) const;

    float get_specular(Eigen::Vector3f uv) const;

    Eigen::Vector3f get_normal(Eigen::Vector3f uv) const;

    std::pair<Eigen::Vector3f, Eigen::Vector3f> get_world_bounding_box() const;

    void transform(const Eigen::Matrix4f &m, bool rigid = false);
};

#endif //__MODEL_H__