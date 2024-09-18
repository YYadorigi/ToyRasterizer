#include <iostream>
#include "tgaimage.hpp"
#include "model.hpp"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const int width = 800;
const int height = 800;
const int virtual_depth = 1000000;

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

Vec3f barycentric_coords(const Vec2i &a, const Vec2i &b, const Vec2i &c, const Vec2i &p)
{
    Vec3i ab = Vec3i(b.u - a.u, c.u - a.u, a.u - p.u);
    Vec3i ac = Vec3i(b.v - a.v, c.v - a.v, a.v - p.v);

    Vec3i u = ab.cross_product(ac);

    if (u.z == 0)
        return Vec3f(-1, 1, 1);

    float uz_inv = 1.f / (float)u.z;
    return Vec3f(1.f - (u.x + u.y) * uz_inv, u.y * uz_inv, u.x * uz_inv);
}

void triangle(const Vec3i a, const Vec3i b, const Vec3i c, TGAImage &image, const TGAColor &color, int *zbuffer)
{
    int umin, umax, vmin, vmax;

    umin = std::min(std::min(a.x, b.x), c.x);
    umax = std::max(std::max(a.x, b.x), c.x);
    vmin = std::min(std::min(a.y, b.y), c.y);
    vmax = std::max(std::max(a.y, b.y), c.y);

    for (int v = vmin; v <= vmax; v++)
    {
        for (int u = umin; u <= umax; u++)
        {
            if (inside_triangle(Vec2i(a.x, a.y), Vec2i(b.x, b.y), Vec2i(c.x, c.y), Vec2i(u, v)))
            {
                Vec3f barycentric = barycentric_coords(Vec2i(a.x, a.y), Vec2i(b.x, b.y), Vec2i(c.x, c.y), Vec2i(u, v));
                int z = a.z * barycentric.x + b.z * barycentric.y + c.z * barycentric.z;
                if (z > zbuffer[v * width + u])
                {
                    image.set(u, v, color);
                    zbuffer[v * width + u] = z;
                }
            }
        }
    }
}

int main(int argc, char **argv)
{
    TGAImage image(width, height, TGAImage::RGB);
    Model *model = new Model("../objects/african_head.obj");

    Vec3f light_dir(0, 0, -1); // with area infinitely large

    int *zbuffer = new int[width * height];
    memset(zbuffer, -virtual_depth, sizeof(int) * width * height);

    for (int i = 0; i < model->num_faces(); i++)
    {
        Vec3i face = model->get_face_index(i);

        Vec3f world_coords[3];
        Vec3i screen_coords[3];

        for (int j = 0; j < 3; j++)
        {
            Vec3f v = model->get_vert(face.raw[j]);
            world_coords[j] = v;
            screen_coords[j] = Vec3i((v.x + 1) / 2 * width, (v.y + 1) / 2 * height, (v.z - 1) / 2 * virtual_depth);
        }

        Vec3f normal = (world_coords[1] - world_coords[0]).cross_product(world_coords[2] - world_coords[0]);
        normal.normalized();

        float cos_theta = -normal.dot_product(light_dir);

        if (cos_theta > 0)
        {
            TGAColor white_reflection(white.r * cos_theta, white.g * cos_theta, white.b * cos_theta, white.a);
            triangle(screen_coords[0], screen_coords[1], screen_coords[2], image, white_reflection, zbuffer);
        }
    }

    image.flip_vertically();
    image.write_tga_file("../outputs/output.tga");
    return 0;
}