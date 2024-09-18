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
        else if (!line.compare(0, 2, "f "))
        {
            Vec3i f;
            int itrash, idx;
            iss >> trash;
            for (int i = 0; i < 3;i++){
                iss >> idx >> trash >> itrash >> trash >> itrash;
                f.raw[i] = --idx;
            }
            ind.emplace_back(f);
        }
    }
    std::cerr << "# v# " << vert.size() << " f# " << ind.size() << std::endl;
}

Model::~Model()
{
    vert.clear();
    ind.clear();
}

int Model::num_verts()
{
    return (int)vert.size();
}

int Model::num_faces()
{
    return (int)ind.size();
}

Vec3i Model::get_face_index(int idx)
{
    return ind[idx];
}

Vec3f Model::get_vert(int idx)
{
    return vert[idx];
}
