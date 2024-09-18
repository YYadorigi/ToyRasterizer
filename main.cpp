#include <iostream>
#include "tgaimage.hpp"
#include "model.hpp"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const int width = 4000;
const int height = 4000;

void line(int x0, int y0, int x1, int y1, TGAImage &image, const TGAColor &color)
{
    if (x1 == x0 && y1 == y0)
        return;

    // transpose
    bool transpose = false;
    if (abs(x1 - x0) < abs(y1 - y0))
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
        transpose = true;
    }

    // direction
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int dy = 2 * abs(y1 - y0);
    int dx = x1 - x0;
    int direction = (y0 < y1 ? 1 : -1);

    int y = y0;
    int error = 0;

    if (transpose)
    {
        for (int x = x0; x <= x1; x++)
        {
            image.set(y, x, color);
            error += dy;
            if (error > dx)
            {
                y += direction;
                error -= dx * 2;
            }
        }
    }
    else
    {
        for (int x = x0; x <= x1; x++)
        {
            image.set(x, y, color);
            error += dy;
            if (error > dx)
            {
                y += direction;
                error -= dx * 2;
            }
        }
    }
}

int main(int argc, char **argv)
{
    Model *model = nullptr;
    if (argc == 2)
    {
        model = new Model(argv[1]);
    }
    else
    {
        model = new Model("../objects/african_head.obj");
    }

    TGAImage image(width, height, TGAImage::RGB);
    for (int i = 0; i < model->num_faces(); i++)
    {
        Vec3i face_ind = model->get_face_index(i);
        for (int j = 0; j < 3; j++)
        {
            Vec3f v0 = model->get_vert(face_ind.raw[j]);
            Vec3f v1 = model->get_vert(face_ind.raw[(j + 1) % 3]);
            int x0 = (v0.x + 1.) / 2. * width;
            int y0 = (v0.y + 1.) / 2. * height;
            int x1 = (v1.x + 1.) / 2. * width;
            int y1 = (v1.y + 1.) / 2. * height;
            line(x0, y0, x1, y1, image, white);
        }
    }

    image.flip_vertically();
    image.write_tga_file("../outputs/output.tga");
    delete model;
    return 0;
    return 0;
}