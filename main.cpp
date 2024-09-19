#include <iostream>
#include "tgaimage.hpp"
#include "model.hpp"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const int width = 800;
const int height = 800;
const int virtual_depth = 1e6;

bool inside_triangle(const Vec2i &a, const Vec2i &b, const Vec2i &c, const Vec2i &p)
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
    float alpha, beta, gamma;
    try
    {
        alpha = (-(p.u - b.u) * (c.v - b.v) + (p.v - b.v) * (c.u - b.u)) * 1.0 /
                (-(a.u - b.u) * (c.v - b.v) + (a.v - b.v) * (c.u - b.u));
        beta = (-(p.u - c.u) * (a.v - c.v) + (p.v - c.v) * (a.u - c.u)) * 1.0 /
               (-(b.u - c.u) * (a.v - c.v) + (b.v - c.v) * (a.u - c.u));
        gamma = 1.f - alpha - beta;
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    return Vec3f(alpha, beta, gamma);
}

void triangle(const Vec3i &a, const Vec3i &b, const Vec3i &c,
              const Vec3f &a_uv, const Vec3f &b_uv, const Vec3f &c_uv,
              float lambertian_cos, Model &model, TGAImage &image, int *zbuffer)
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
            Vec3f barycentric = barycentric_coords(Vec2i(a.x, a.y), Vec2i(b.x, b.y), Vec2i(c.x, c.y), Vec2i(u, v));

            if (barycentric.x < 0 || barycentric.y < 0 || barycentric.z < 0)
                continue;

            int z = a.z * barycentric.x + b.z * barycentric.y + c.z * barycentric.z;
            if (z > zbuffer[v * width + u])
            {
                Vec3f uv = a_uv * barycentric.x + b_uv * barycentric.y + c_uv * barycentric.z;
                TGAColor color = model.get_texture(uv);
                image.set(u, v, color * lambertian_cos);
                zbuffer[v * width + u] = z;
            }
        }
    }
}

int main(int argc, char **argv)
{
    TGAImage image(width, height, TGAImage::RGB);
    Model model("../objects/african_head.obj");

    Vec3f light_dir(0, 0, -1); // with area infinitely large

    int *zbuffer = new int[width * height];
    memset(zbuffer, -virtual_depth, sizeof(int) * width * height);

    for (int i = 0; i < model.num_faces(); i++)
    {
        Vec3i verts, texts, norms;
        std::array<Vec3i, 3> ind = model.get_face(i);
        verts = ind[0];
        texts = ind[1];
        norms = ind[2];

        Vec3f world_coords[3];
        Vec3i screen_coords[3];
        Vec3f texture_coords[3];
        Vec3f normal_coords[3];

        for (int j = 0; j < 3; j++)
        {
            // convert the vertices from world space to screen space
            Vec3f v = model.get_vert(verts.raw[j]);
            world_coords[j] = v;
            screen_coords[j] = Vec3i((v.x + 1) / 2 * width, (v.y + 1) / 2 * height, (v.z - 1) / 2 * virtual_depth);

            // sample the texture of the vertices
            texture_coords[j] = model.get_texture(texts.raw[j]);

            // sample the normal of the vertices
            normal_coords[j] = model.get_normal(norms.raw[j]);
        }

        Vec3f normal = (world_coords[1] - world_coords[0]).cross_product(world_coords[2] - world_coords[0]);
        normal = normal.normalized();

        float cos_theta = -normal.dot_product(light_dir);

        if (cos_theta > 0) // back face culling
        {
            triangle(screen_coords[0], screen_coords[1], screen_coords[2],
                     texture_coords[0], texture_coords[1], texture_coords[2],
                     cos_theta, model, image, zbuffer);
        }
    }

    image.flip_vertically();
    image.write_tga_file("../outputs/output.tga");

    delete[] zbuffer;
    return 0;
}