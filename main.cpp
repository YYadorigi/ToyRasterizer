#include <iostream>
#include "tgaimage.hpp"
#include "model.hpp"

const TGAColor white = TGAColor(255, 255, 255, 255);
const int width = 800;
const int height = 800;
const Eigen::Vector3f eye(0, 0, 2);
const Eigen::Vector3f look_at(0, 0, -1);
const Eigen::Vector3f up(0, 1, 0);

void line(const Eigen::Vector2i &from, const Eigen::Vector2i &to, TGAImage &image, const TGAColor &color)
{
    if (from == to)
        return;

    Eigen::Vector2i start = from;
    Eigen::Vector2i end = to;

    // transpose
    bool transpose = false;
    if (abs(start.x() - end.x()) < abs(start.y() - end.y()))
    {
        std::swap(start.x(), start.y());
        std::swap(end.x(), end.y());
        transpose = true;
    }

    // direction
    if (start.x() > end.x())
        std::swap(start, end);

    int dy = 2 * abs(end.y() - start.y());
    int dx = end.x() - start.x();
    int direction = (start.y() < end.y() ? 1 : -1);

    int y = start.y();
    int error = 0;

    if (transpose)
    {
        for (int x = start.x(); x <= end.x(); x++)
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
        for (int x = start.x(); x <= end.x(); x++)
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

Eigen::Matrix4f camera_transform(Eigen::Vector3f camera_pos, Eigen::Vector3f look_at, Eigen::Vector3f up)
{
    Eigen::Vector3f w = -look_at.normalized();
    Eigen::Vector3f v = up.normalized();
    Eigen::Vector3f u = look_at.cross(up).normalized();

    Eigen::Matrix4f rotation;
    rotation << u.x(), u.y(), u.z(), 0,
        v.x(), v.y(), v.z(), 0,
        w.x(), w.y(), w.z(), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f translation;
    translation << 1, 0, 0, -camera_pos.x(),
        0, 1, 0, -camera_pos.y(),
        0, 0, 1, -camera_pos.z(),
        0, 0, 0, 1;

    return rotation * translation;
}

Eigen::Matrix4f perspective_transformation(float near, float far)
{
    Eigen::Matrix4f persp;
    persp << near, 0, 0, 0,
        0, near, 0, 0,
        0, 0, near + far, -near * far,
        0, 0, 1, 0;

    return persp;
}

Eigen::Matrix4f orthogonal_transformation(float near, float far, float left, float right, float top, float bottom)
{
    Eigen::Matrix4f ortho;
    ortho << 2 / (right - left), 0, 0, -(right + left) / (right - left),
        0, 2 / (top - bottom), 0, -(top + bottom) / (top - bottom),
        0, 0, 2 / (near - far), -(far + near) / (near - far),
        0, 0, 0, 1;

    return ortho;
}

Eigen::Matrix4f viewport_transformation(int width, int height)
{
    Eigen::Matrix4f viewport;
    viewport << width / 2.f, 0, 0, (width - 1) / 2.f,
        0, height / 2.f, 0, (height - 1) / 2.f,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return viewport;
}

bool inside_triangle(const Eigen::Vector2f &a, const Eigen::Vector2f &b, const Eigen::Vector2f &c, const Eigen::Vector2f &p)
{
    auto converter = [](Eigen::Vector2f a, Eigen::Vector2f b)
    { return Eigen::Vector3f((b - a).x(), (b - a).y(), 0); };

    Eigen::Vector3f ab = converter(a, b);
    Eigen::Vector3f bc = converter(b, c);
    Eigen::Vector3f ca = converter(c, a);
    Eigen::Vector3f ap = converter(a, p);
    Eigen::Vector3f bp = converter(b, p);
    Eigen::Vector3f cp = converter(c, p);

    return (ab.cross(ap).z() >= 0 && bc.cross(bp).z() >= 0 && ca.cross(cp).z() >= 0) ||
           (ab.cross(ap).z() <= 0 && bc.cross(bp).z() <= 0 && ca.cross(cp).z() <= 0);
}

Eigen::Vector3f barycentric_coords(const Eigen::Vector2f &a, const Eigen::Vector2f &b, const Eigen::Vector2f &c, const Eigen::Vector2f &p)
{
    float alpha, beta, gamma;
    try
    {
        alpha = (-(p.x() - b.x()) * (c.y() - b.y()) + (p.y() - b.y()) * (c.x() - b.x())) * 1.0 /
                (-(a.x() - b.x()) * (c.y() - b.y()) + (a.y() - b.y()) * (c.x() - b.x()));
        beta = (-(p.x() - c.x()) * (a.y() - c.y()) + (p.y() - c.y()) * (a.x() - c.x())) * 1.0 /
               (-(b.x() - c.x()) * (a.y() - c.y()) + (b.y() - c.y()) * (a.x() - c.x()));
        gamma = 1.f - alpha - beta;
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    return Eigen::Vector3f(alpha, beta, gamma);
}

void triangle(const Eigen::Vector3f &a, const Eigen::Vector3f &b, const Eigen::Vector3f &c,
              const Eigen::Vector3f &a_uv, const Eigen::Vector3f &b_uv, const Eigen::Vector3f &c_uv,
              const Eigen::Vector3f &a_norm, const Eigen::Vector3f &b_norm, const Eigen::Vector3f &c_norm,
              Model &model, TGAImage &image, float *zbuffer)
{
    int umin, umax, vmin, vmax;

    umin = floorf(std::min(std::min(a.x(), b.x()), c.x()));
    umax = ceilf(std::max(std::max(a.x(), b.x()), c.x()));
    vmin = floorf(std::min(std::min(a.y(), b.y()), c.y()));
    vmax = ceilf(std::max(std::max(a.y(), b.y()), c.y()));

    for (int v = vmin; v <= vmax; v++)
    {
        for (int u = umin; u <= umax; u++)
        {
            Eigen::Vector3f barycentric = barycentric_coords(Eigen::Vector2f(a.x(), a.y()), Eigen::Vector2f(b.x(), b.y()),
                                                             Eigen::Vector2f(c.x(), c.y()), Eigen::Vector2f(u, v));

            if (barycentric.x() < 0 || barycentric.y() < 0 || barycentric.z() < 0)
                continue;

            float a_depth_inv = 1 / a.z();
            float b_depth_inv = 1 / b.z();
            float c_depth_inv = 1 / c.z();

            float z = 1 / (a_depth_inv * barycentric.x() + b_depth_inv * barycentric.y() + c_depth_inv * barycentric.z());
            float alpha = a_depth_inv * barycentric.x() * z;
            float beta = b_depth_inv * barycentric.y() * z;
            float gamma = c_depth_inv * barycentric.z() * z;

            if (z > zbuffer[v * width + u])
            {
                Eigen::Vector3f uv = a_uv * alpha + b_uv * beta + c_uv * gamma;
                // Eigen::Vector3f norm = (a_norm * alpha + b_norm * beta + c_norm * gamma).normalized();
                TGAColor color = model.get_texture(uv);
                image.set(u, v, color);
                zbuffer[v * width + u] = z;
            }
        }
    }
}

int main(int argc, char **argv)
{
    TGAImage image(width, height, TGAImage::RGB);
    Model model("../objects/african_head.obj");

    std::pair<Eigen::Vector3f, Eigen::Vector3f> bbox;
    Eigen::Matrix4f camera = camera_transform(eye, look_at, up);
    model.transform(camera);
    bbox = model.get_bbox();
    Eigen::Matrix4f perspective = perspective_transformation(bbox.second.z(), bbox.first.z());
    model.transform(perspective);
    bbox = model.get_bbox();
    Eigen::Matrix4f orthogonal = orthogonal_transformation(bbox.second.z(), bbox.first.z(), bbox.first.x(), bbox.second.x(), bbox.second.y(), bbox.first.y());
    model.transform(orthogonal);
    Eigen::Matrix4f viewport = viewport_transformation(width, height);
    model.transform(viewport);

    // model.transform(viewport * projection * camera);

    float *zbuffer = new float[width * height];
    std::fill(zbuffer, zbuffer + width * height, -1.1);

    for (int i = 0; i < model.num_faces(); i++)
    {
        Eigen::Vector3i verts = model.get_face(i)[0];
        Eigen::Vector3i texts = model.get_face(i)[1];
        Eigen::Vector3i norms = model.get_face(i)[2];

        Eigen::Vector3f screen_coords[3];
        Eigen::Vector3f texture_coords[3];
        Eigen::Vector3f normal_coords[3];

        for (int j = 0; j < 3; j++)
        {
            // convert the vertices from world space to screen space
            screen_coords[j] = model.get_vert(verts[j]);

            // sample the texture of the vertices
            texture_coords[j] = model.get_texture(texts[j]);

            // sample the normal of the vertices
            normal_coords[j] = model.get_normal(norms[j]);
        }

        triangle(screen_coords[0], screen_coords[1], screen_coords[2],
                 texture_coords[0], texture_coords[1], texture_coords[2],
                 normal_coords[0], normal_coords[1], normal_coords[2],
                 model, image, zbuffer);
    }

    image.flip_vertically();
    image.write_tga_file("../outputs/output.tga");

    delete[] zbuffer;
    return 0;
}