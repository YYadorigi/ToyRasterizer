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
            Eigen::Vector3f v;
            iss >> v.x() >> v.y() >> v.z();
            vert.emplace_back(v);
            vert_world_coords.emplace_back(v);
        }
        else if (!line.compare(0, 3, "vt "))
        {
            iss >> trash >> trash;
            Eigen::Vector3f vt;
            iss >> vt.x() >> vt.y();
            vt.z() = 0;
            text.emplace_back(vt);
        }
        else if (!line.compare(0, 3, "vn "))
        {
            iss >> trash >> trash;
            Eigen::Vector3f vn;
            iss >> vn.x() >> vn.y() >> vn.z();
            norm.emplace_back(vn);
        }
        else if (!line.compare(0, 2, "f "))
        {
            Eigen::Vector3i vert_ind, text_ind, norm_ind;
            int vert_buf, text_buf, norm_buf;
            iss >> trash;
            for (int i = 0; i < 3; i++)
            {
                iss >> vert_buf >> trash >> text_buf >> trash >> norm_buf;
                vert_ind[i] = vert_buf - 1;
                text_ind[i] = text_buf - 1;
                norm_ind[i] = norm_buf - 1;
            }
            ind.emplace_back(std::array<Eigen::Vector3i, 3>{vert_ind, text_ind, norm_ind});
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

TGAColor Model::get_texture(Eigen::Vector3f uv) const
{
    int u = uv.x() * texture.get_width();
    int v = uv.y() * texture.get_height();
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

std::array<Eigen::Vector3i, 3> Model::get_face(int idx) const
{
    return ind[idx];
}

Eigen::Vector3f Model::get_vert(int idx) const
{
    return vert[idx];
}

Eigen::Vector3f Model::get_vert_world_coords(int idx) const
{
    return vert_world_coords[idx];
}

Eigen::Vector3f Model::get_texture(int idx) const
{
    return text[idx];
}

Eigen::Vector3f Model::get_normal(int idx) const
{
    return norm[idx];
}

void Model::transform(const Eigen::Matrix4f &m)
{
    for (auto &v : vert)
    {
        Eigen::Vector4f v4(v.x(), v.y(), v.z(), 1);
        v4 = m * v4;
        float w_inv = 1.0f / v4.w();
        v.x() = v4.x() * w_inv;
        v.y() = v4.y() * w_inv;
        v.z() = v4.z() * w_inv;
    }

    Eigen::Matrix4f m_inv_t = m.transpose().inverse();

    for (auto &n : norm)
    {
        Eigen::Vector4f n4(n.x(), n.y(), n.z(), 0);
        n4 = m_inv_t * n4;
        n.x() = n4.x();
        n.y() = n4.y();
        n.z() = n4.z();
        n.normalize();
    }
}