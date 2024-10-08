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
            ind.emplace_back(std::pair<Eigen::Vector3i, Eigen::Vector3i>{vert_ind, text_ind});
        }
    }
    load_texture(filename);
    std::cerr << "# v# " << vert.size() << " f# " << ind.size() << " vt# " << text.size() << " vn# " << norm.size() << std::endl;
}

Model::~Model()
{
    vert.clear();
    ind.clear();
}

void Model::load_texture(std::string filename)
{
    std::string texfile(filename);
    size_t dot = texfile.find_last_of(".");
    if (dot != std::string::npos)
    {
        std::string diffuse = texfile.substr(0, dot) + std::string("_diffuse.tga");
        std::string specular = texfile.substr(0, dot) + std::string("_spec.tga");
        std::string tangent_normal = texfile.substr(0, dot) + std::string("_nm.tga");
        std::cerr << "diffuse texture file " << diffuse << " loading " << (diffuse_texture.read_tga_file(diffuse.c_str()) ? "ok" : "failed") << std::endl;
        std::cerr << "specular texture file " << specular << " loading " << (specular_texture.read_tga_file(specular.c_str()) ? "ok" : "failed") << std::endl;
        std::cerr << "normal texture file " << tangent_normal << " loading " << (normal_texture.read_tga_file(tangent_normal.c_str()) ? "ok" : "failed") << std::endl;
        diffuse_texture.flip_vertically();
        specular_texture.flip_vertically();
        normal_texture.flip_vertically();
    }
}

Eigen::Vector3f Model::get_diffuse(Eigen::Vector3f uv) const
{
    int u = uv.x() * diffuse_texture.get_width();
    int v = uv.y() * diffuse_texture.get_height();
    TGAColor color = diffuse_texture.get(u, v);
    Eigen::Vector3f diff_color;
    diff_color << color.raw[2], color.raw[1], color.raw[0];
    return diff_color;
}

float Model::get_specular(Eigen::Vector3f uv) const
{
    int u = uv.x() * specular_texture.get_width();
    int v = uv.y() * specular_texture.get_height();
    float color = specular_texture.get(u, v).raw[0] / 1.f;
    return color;
}

Eigen::Vector3f Model::get_normal(Eigen::Vector3f uv) const
{
    int u = uv.x() * normal_texture.get_width();
    int v = uv.y() * normal_texture.get_height();
    TGAColor color = normal_texture.get(u, v);
    Eigen::Vector3f normal;
    normal << color.raw[2], color.raw[1], color.raw[0];
    return normal.normalized();
}

int Model::num_verts() const
{
    return (int)vert.size();
}

int Model::num_faces() const
{
    return (int)ind.size();
}

std::pair<Eigen::Vector3i, Eigen::Vector3i> Model::get_face(int idx) const
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

void Model::transform(const Eigen::Matrix4f &m, bool rigid)
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

    if (rigid)
    {
        for (auto &v : vert_world_coords)
        {
            Eigen::Vector4f v4(v.x(), v.y(), v.z(), 1);
            v4 = m * v4;
            float w_inv = 1.0f / v4.w();
            v.x() = v4.x() * w_inv;
            v.y() = v4.y() * w_inv;
            v.z() = v4.z() * w_inv;
        }
    }
}

std::pair<Eigen::Vector3f, Eigen::Vector3f> Model::get_world_bounding_box() const
{
    Eigen::Vector3f min(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Eigen::Vector3f max(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    for (auto &v : vert_world_coords)
    {
        min.x() = std::min(min.x(), v.x());
        min.y() = std::min(min.y(), v.y());
        min.z() = std::min(min.z(), v.z());
        max.x() = std::max(max.x(), v.x());
        max.y() = std::max(max.y(), v.y());
        max.z() = std::max(max.z(), v.z());
    }
    return std::make_pair(min, max);
}