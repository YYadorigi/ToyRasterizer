#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include <array>
#include "geometry.hpp"
#include "tgaimage.hpp"

class Model
{
private:
    std::vector<Vec3f> vert;
    std::vector<Vec3f> text;
    std::vector<Vec3f> norm;
    std::vector<std::array<Vec3i, 3>> ind; // indexed triangle mesh (vert, text, norm)
    TGAImage texture;

    void load_texture(std::string filename, const char *suffix);

public:
    Model(const char *filename);

    ~Model();

    int num_verts() const;

    int num_faces() const;

    Vec3f get_vert(int idx) const;

    Vec3f get_texture(int idx) const;

    Vec3f get_normal(int idx) const;

    std::array<Vec3i, 3> get_face(int idx) const;

    TGAColor get_texture(Vec3f uv) const;
};

#endif //__MODEL_H__