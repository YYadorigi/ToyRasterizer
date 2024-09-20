#include <iostream>
#include <array>
#include "tgaimage.hpp"
#include "model.hpp"

const TGAColor black = TGAColor(0, 0, 0, 255);
const TGAColor white = TGAColor(255, 255, 255, 255);
const float eps = 1e-4;
const int width = 800;
const int height = 800;

const int msaa = 4;

struct Camera
{
    Eigen::Vector3f pos;
    Eigen::Vector3f look_at;
    Eigen::Vector3f up;
};

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

Eigen::Matrix4f model_transform(const Eigen::Vector3f axis, const float rotation_angle, const Eigen::Vector3f scale, const Eigen::Vector3f translation)
{
    // Rodrigues' rotation formula
    Eigen::Matrix4f rotation;
    rotation << cos(rotation_angle) + axis.x() * axis.x() * (1 - cos(rotation_angle)),
        axis.x() * axis.y() * (1 - cos(rotation_angle)) - axis.z() * sin(rotation_angle),
        axis.x() * axis.z() * (1 - cos(rotation_angle)) + axis.y() * sin(rotation_angle),
        0,
        axis.y() * axis.x() * (1 - cos(rotation_angle)) + axis.z() * sin(rotation_angle),
        cos(rotation_angle) + axis.y() * axis.y() * (1 - cos(rotation_angle)),
        axis.y() * axis.z() * (1 - cos(rotation_angle)) - axis.x() * sin(rotation_angle),
        0,
        axis.z() * axis.x() * (1 - cos(rotation_angle)) - axis.y() * sin(rotation_angle),
        axis.z() * axis.y() * (1 - cos(rotation_angle)) + axis.x() * sin(rotation_angle),
        cos(rotation_angle) + axis.z() * axis.z() * (1 - cos(rotation_angle)),
        0,
        0, 0, 0, 1;

    Eigen::Matrix4f scaling;
    scaling << scale.x(), 0, 0, 0,
        0, scale.y(), 0, 0,
        0, 0, scale.z(), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f translation_matrix;
    translation_matrix << 1, 0, 0, translation.x(),
        0, 1, 0, translation.y(),
        0, 0, 1, translation.z(),
        0, 0, 0, 1;

    return translation_matrix * scaling * rotation;
}

Eigen::Matrix4f camera_transform(Camera camera)
{
    Eigen::Vector3f w = -camera.look_at.normalized();
    Eigen::Vector3f v = camera.up.normalized();
    Eigen::Vector3f u = camera.look_at.cross(camera.up).normalized();

    Eigen::Matrix4f rotation;
    rotation << u.x(), u.y(), u.z(), 0,
        v.x(), v.y(), v.z(), 0,
        w.x(), w.y(), w.z(), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f translation;
    translation << 1, 0, 0, -camera.pos.x(),
        0, 1, 0, -camera.pos.y(),
        0, 0, 1, -camera.pos.z(),
        0, 0, 0, 1;

    return rotation * translation;
}

Eigen::Matrix4f perspective_transformation(float near, float far, float fovY, float aspect_ratio)
{
    Eigen::Matrix4f persp;
    persp << near, 0, 0, 0,
        0, near, 0, 0,
        0, 0, near + far, -near * far,
        0, 0, 1, 0;

    float top = fabsf(near) * tan(fovY * M_PI / 360.f);
    float right = top * aspect_ratio;

    Eigen::Matrix4f ortho;
    ortho << 1 / right, 0, 0, 0,
        0, 1 / top, 0, 0,
        0, 0, 2 / (near - far), -(far + near) / (near - far),
        0, 0, 0, 1;

    return ortho * persp;
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
              Model &model, TGAImage &image, TGAColor *framebuffer, float *zbuffer)
{
    int umin, umax, vmin, vmax;

    umin = floorf(std::min(std::min(a.x(), b.x()), c.x()));
    umax = ceilf(std::max(std::max(a.x(), b.x()), c.x()));
    vmin = floorf(std::min(std::min(a.y(), b.y()), c.y()));
    vmax = ceilf(std::max(std::max(a.y(), b.y()), c.y()));

    // clamp
    umin = std::max(0, umin);
    umax = std::min(width - 1, umax);
    vmin = std::max(0, vmin);
    vmax = std::min(height - 1, vmax);

    Eigen::Vector2f multisample_bias[msaa] = {
        Eigen::Vector2f(+0.25, +0.25),
        Eigen::Vector2f(-0.25, +0.25),
        Eigen::Vector2f(-0.25, -0.25),
        Eigen::Vector2f(+0.25, -0.25)};

    if (std::count_if(multisample_bias, multisample_bias + msaa, [](Eigen::Vector2f p)
                      { return p.x() != 0 && p.y() != 0; }) != msaa)
        throw std::runtime_error("Error: the number of multisample points is not corrsponding to msaa setting");

    for (int v = vmin; v <= vmax; v++)
    {
        for (int u = umin; u <= umax; u++)
        {
            TGAColor color = black;
            bool inside = false;
            for (int i = 0; i < msaa; i++)
            {
                Eigen::Vector2f sub_sample = Eigen::Vector2f(u, v) + multisample_bias[i];
                Eigen::Vector3f barycentric = barycentric_coords(Eigen::Vector2f(a.x(), a.y()), Eigen::Vector2f(b.x(), b.y()),
                                                                 Eigen::Vector2f(c.x(), c.y()), sub_sample);

                if (barycentric.x() >= 0 && barycentric.y() >= 0 && barycentric.z() >= 0)
                {
                    inside = true;

                    float z = 1 / (barycentric.x() / a.z() + barycentric.y() / b.z() + barycentric.z() / c.z());
                    float alpha = barycentric.x() / a.z() * z;
                    float beta = barycentric.y() / b.z() * z;
                    float gamma = barycentric.z() / c.z() * z;

                    if (z > zbuffer[(v * width + u) * msaa + i])
                    {
                        Eigen::Vector3f uv = a_uv * alpha + b_uv * beta + c_uv * gamma;
                        // Eigen::Vector3f norm = (a_norm * alpha + b_norm * beta + c_norm * gamma).normalized();
                        TGAColor sub_color = model.get_texture(uv);
                        framebuffer[(v * width + u) * msaa + i] = sub_color;
                        zbuffer[(v * width + u) * msaa + i] = z;
                    }
                }
            }
            if (inside)
            {
                for (int i = 0; i < msaa; i++)
                    color = color + framebuffer[(v * width + u) * msaa + i] * (1.f / msaa); // TODO: integer division error
                image.set(u, v, color);
            }
        }
    }
}

int main(int argc, char **argv)
{
    // initialize image
    TGAImage image(width, height, TGAImage::RGB);

    // load model and its texture
    Model model("../objects/african_head.obj");

    // camera
    Camera camera;
    camera.pos = Eigen::Vector3f(0, 0, 10);
    camera.look_at = Eigen::Vector3f(0, 0, -1);
    camera.up = Eigen::Vector3f(0, 1, 0);

    // frustum
    const float near = -5;
    const float far = -15;
    const float fovY = 15;        // degrees
    const float aspect_ratio = 1; // view width / view height

    Eigen::Matrix4f cam = camera_transform(camera);
    Eigen::Matrix4f persp = perspective_transformation(near, far, fovY, aspect_ratio);
    Eigen::Matrix4f viewport = viewport_transformation(width, height);

    Eigen::Matrix4f mvp = viewport * persp * cam;
    model.transform(mvp);

    float *zbuffer = new float[width * height * msaa];
    std::fill(zbuffer, zbuffer + width * height * msaa, -std::numeric_limits<float>::max());

    TGAColor *framebuffer = new TGAColor[width * height * msaa];
    std::fill(framebuffer, framebuffer + width * height * msaa, black);

    for (int i = 0; i < model.num_faces(); i++)
    {
        std::array<Eigen::Vector3i, 3> face = model.get_face(i);
        Eigen::Vector3i verts = face[0];
        Eigen::Vector3i texts = face[1];
        Eigen::Vector3i norms = face[2];

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
                 model, image, framebuffer, zbuffer);
    }

    image.flip_vertically();
    image.write_tga_file("../outputs/output.tga");

    delete[] zbuffer;
    return 0;
}