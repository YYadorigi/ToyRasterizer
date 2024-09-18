#include <iostream>
#include "tgaimage.hpp"
#include "model.hpp"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const int width = 800;
const int height = 800;

void line(const Vec2i &from, const Vec2i &to, TGAImage &image, const TGAColor &color)
{
    if (from == to)
        return;

    Vec2i start = from;
    Vec2i end = to;

    // transpose
    bool transpose = false;
    if (abs(start.u - end.u) < abs(start.v - end.v))
    {
        std::swap(start.u, start.v);
        std::swap(end.u, end.v);
        transpose = true;
    }

    // direction
    if (start.u > end.u)
        std::swap(start, end);

    int dy = 2 * abs(end.v - start.v);
    int dx = end.u - start.u;
    int direction = (start.v < end.v ? 1 : -1);

    int y = start.v;
    int error = 0;

    if (transpose)
    {
        for (int x = start.u; x <= end.u; x++)
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
        for (int x = start.u; x <= end.u; x++)
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

bool inside_triangle(const Vec2i &a, const Vec2i &b, const Vec2i &c, Vec2i p)
{
    auto converter = [](Vec2i a, Vec2i b)
    { return Vec3i((b - a).u, (b - a).v, 0); };

    Vec3i ab = converter(a, b);
    Vec3i bc = converter(b, c);
    Vec3i ca = converter(c, a);
    Vec3i ap = converter(a, p);
    Vec3i bp = converter(b, p);
    Vec3i cp = converter(c, p);

    return (ab.cross_product(ap).z >= 0 && bc.cross_product(bp).z >= 0 && ca.cross_product(cp).z >= 0) ||
           (ab.cross_product(ap).z <= 0 && bc.cross_product(bp).z <= 0 && ca.cross_product(cp).z <= 0);
}

void triangle(const Vec2i a, const Vec2i b, const Vec2i c, TGAImage &image, const TGAColor &color)
{
    int umin, umax, vmin, vmax;

    umin = std::min(std::min(a.u, b.u), c.u);
    umax = std::max(std::max(a.u, b.u), c.u);
    vmin = std::min(std::min(a.v, b.v), c.v);
    vmax = std::max(std::max(a.v, b.v), c.v);

    for (int u = umin; u <= umax; u++)
    {
        for (int v = vmin; v <= vmax; v++)
        {
            if (inside_triangle(a, b, c, Vec2i(u, v)))
                image.set(u, v, color);
        }
    }
}

int main(int argc, char **argv)
{
    TGAImage image(width, height, TGAImage::RGB);
    Model *model = new Model("../objects/african_head.obj");

    Vec3f light_dir(0, 0, -1); // with area infinitely large

    for (int i = 0; i < model->num_faces(); i++)
    {
        Vec3i face = model->get_face_index(i);

        Vec3f world_coords[3];
        Vec2i screen_coords[3];

        for (int j = 0; j < 3; j++)
        {
            Vec3f v = model->get_vert(face.raw[j]);
            world_coords[j] = v;
            screen_coords[j] = Vec2i((v.x + 1) / 2 * width, (v.y + 1) / 2 * height);
        }

        Vec3f normal = (world_coords[1] - world_coords[0]).cross_product(world_coords[2] - world_coords[0]);
        normal.normalized();

        float cos_theta = std::max(0.f, -normal.dot_product(light_dir));

        TGAColor white_reflection(white.r * cos_theta, white.g * cos_theta, white.b * cos_theta, white.a);
        triangle(screen_coords[0], screen_coords[1], screen_coords[2], image, white_reflection);
    }

    image.flip_vertically();
    image.write_tga_file("../outputs/output.tga");
    return 0;
}