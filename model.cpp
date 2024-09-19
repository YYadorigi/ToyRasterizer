#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "model.hpp"

Model::Model(const char *filename) : vert(), ind()
{
    std::ifstream in;
    in.open(filename, std::ifstream::in);
    if (in.fail())
        return;

    std::string line;
    while (!in.eof())
    {
        std::getline(in, line);
        std::istringstream iss(line.c_str());
        char trash;
        if (!line.compare(0, 2, "v "))
        {
            iss >> trash;
            Vec3f v;
            for (int i = 0; i < 3; i++)
                iss >> v.raw[i];
            vert.emplace_back(v);
        }
        else if (!line.compare(0, 3, "vt "))
        {
            iss >> trash >> trash;
            Vec3f vt;
            for (int i = 0; i < 3; i++)
                iss >> vt.raw[i];
            text.emplace_back(vt);
        }
        else if (!line.compare(0, 3, "vn "))
        {
            iss >> trash >> trash;
            Vec3f vn;
            for (int i = 0; i < 3; i++)
                iss >> vn.raw[i];
            norm.emplace_back(vn);
        }
        else if (!line.compare(0, 2, "f "))
        {
            Vec3i vert_ind, text_ind, norm_ind;
            int vert_buf, text_buf, norm_buf;
            iss >> trash;
            for (int i = 0; i < 3; i++)
            {
                iss >> vert_buf >> trash >> text_buf >> trash >> norm_buf;
                vert_ind.raw[i] = --vert_buf;
                text_ind.raw[i] = --text_buf;
                norm_ind.raw[i] = --norm_buf;
            }
            ind.emplace_back(std::array<Vec3i, 3>{vert_ind, text_ind, norm_ind});
        }
    }
    load_texture(filename, "_diffuse.tga");
    std::cerr << "# v# " << vert.size() << " f# " << ind.size() << " vt# " << text.size() << " vn# " << norm.size() << std::endl;
}

Model::~Model()
{
    vert.clear();
    ind.clear();
}

void Model::load_texture(std::string filename, const char *suffix)
{
    std::string texfile(filename);
    size_t dot = texfile.find_last_of(".");
    if (dot != std::string::npos)
    {
        texfile = texfile.substr(0, dot) + std::string(suffix);
        std::cerr << "texture file " << texfile << " loading " << (texture.read_tga_file(texfile.c_str()) ? "ok" : "failed") << std::endl;
        texture.flip_vertically();
    }
}

TGAColor Model::get_texture(Vec3f uv) const
{
    int u = uv.x * texture.get_width();
    int v = uv.y * texture.get_height();
    TGAColor color = texture.get(u, v);
    return color;
}

int Model::num_verts() const
{
    return (int)vert.size();
}

int Model::num_faces() const
{
    return (int)ind.size();
}

std::array<Vec3i, 3> Model::get_face(int idx) const
{
    return ind[idx];
}

Vec3f Model::get_vert(int idx) const
{
    return vert[idx];
}

Vec3f Model::get_texture(int idx) const
{
    return text[idx];
}

Vec3f Model::get_normal(int idx) const
{
    return norm[idx];
}
