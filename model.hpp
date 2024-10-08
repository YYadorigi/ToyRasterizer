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
    std::vector<Eigen::Vector2f> text;
    std::vector<std::pair<Eigen::Vector3i, Eigen::Vector3i>> ind; // indexed triangle mesh (vert, text, norm)
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

    Eigen::Vector2f get_texture(int idx) const;

    std::pair<Eigen::Vector3i, Eigen::Vector3i> get_face(int idx) const;

    Eigen::Vector3f get_diffuse(Eigen::Vector2f uv) const;

    float get_specular(Eigen::Vector2f uv) const;

    Eigen::Vector3f get_normal(Eigen::Vector2f uv) const;

    void transform(const Eigen::Matrix4f &m, bool rigid = false);
};

#endif //__MODEL_H__