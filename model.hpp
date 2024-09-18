#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include "geometry.hpp"

class Model
{
private:
    std::vector<Vec3f> vert;
    std::vector<Vec3i> ind; // indexed triangle mesh

public:
    Model(const char *filename);
    ~Model();
    int num_verts() const;
    int num_faces() const;
    Vec3f get_vert(int idx) const;
    Vec3i get_face_index(int idx) const;
};

#endif //__MODEL_H__