#include <Eigen/Dense>
#include "triangle.hpp"
#include "tgaimage.hpp"

const TGAColor black = TGAColor(0, 0, 0, 255);
const TGAColor white = TGAColor(255, 255, 255, 255);

struct Camera
{
    Eigen::Vector3f pos;
    Eigen::Vector3f look_at;
    Eigen::Vector3f up;
};

struct ShadingPoint
{
    Eigen::Vector3f pos;
    Eigen::Vector3f norm;
    Eigen::Vector3f diffuse;
    float specular;
};

struct Light
{
    Eigen::Vector3f pos;
    Eigen::Vector3f intensity;
};

Eigen::Matrix4f model_transform(const Eigen::Vector3f axis, const float rotation_deg, const Eigen::Vector3f scale, const Eigen::Vector3f translation)
{
    // Rodrigues' rotation formula
    Eigen::Matrix4f rotation;
    float rotation_angle = rotation_deg * M_PI / 180.f;
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

TGAColor phong_shading(const ShadingPoint &sp, const Light &light, const Camera &camera)
{
    Eigen::Vector3f light_dir = (light.pos - sp.pos).normalized();
    Eigen::Vector3f normal = sp.norm.normalized();
    Eigen::Vector3f view_dir = (camera.pos - sp.pos).normalized();

    float dist_squared_inv = 1.f / powf((light.pos - sp.pos).norm(), 2);
    float cos_theta = std::max(0.f, normal.dot(light_dir));
    Eigen::Vector3f diffuse;
    for (int i = 0; i < 3; i++)
    {
        diffuse[i] = sp.diffuse[i] * (light.intensity[i] * cos_theta * dist_squared_inv);
    }
    Eigen::Vector3f half = (light_dir + view_dir).normalized();
    Eigen::Vector3f specular = sp.specular * (light.intensity * powf(std::max(0.f, normal.dot(half)), 16) * dist_squared_inv);
    Eigen::Vector3f color = diffuse + specular;
    return TGAColor(color.x(), color.y(), color.z(), 255);
}

void triangle(const Triangle &tri, const std::array<Eigen::Vector3f, 3> &vert_in_world, const Light &light, const Camera &camera,
              const Eigen::Vector2f *sample_bias, const Model &model, TGAImage &image, TGAColor *framebuffer, float *zbuffer)
{
    int width = image.get_width();
    int height = image.get_height();

    Eigen::Vector3f a = tri.v[0];
    Eigen::Vector3f b = tri.v[1];
    Eigen::Vector3f c = tri.v[2];

    Eigen::Vector3f a_world = vert_in_world[0];
    Eigen::Vector3f b_world = vert_in_world[1];
    Eigen::Vector3f c_world = vert_in_world[2];

    Eigen::Vector2f a_uv = tri.t[0];
    Eigen::Vector2f b_uv = tri.t[1];
    Eigen::Vector2f c_uv = tri.t[2];

    PlanarTriangle planar_tri = PlanarTriangle(tri.v[0].head(2), tri.v[1].head(2), tri.v[2].head(2));

    int umin, umax, vmin, vmax;

    umin = (int)floorf(std::min(std::min(a.x(), b.x()), c.x()));
    umax = (int)ceilf(std::max(std::max(a.x(), b.x()), c.x()));
    vmin = (int)floorf(std::min(std::min(a.y(), b.y()), c.y()));
    vmax = (int)ceilf(std::max(std::max(a.y(), b.y()), c.y()));

    // clamp
    umin = std::max(0, umin);
    umax = std::min(width - 1, umax);
    vmin = std::max(0, vmin);
    vmax = std::min(height - 1, vmax);

    int num_samples = sample_bias->size();

    for (int v = vmin; v <= vmax; v++)
    {
        for (int u = umin; u <= umax; u++)
        {
            bool inside = false;
            for (int i = 0; i < num_samples; i++)
            {
                Eigen::Vector2f sub_sample = Eigen::Vector2f(u, v) + sample_bias[i];
                Eigen::Vector3f barycentric = planar_tri.barycentric_coords(sub_sample);

                if (barycentric.x() >= 0 && barycentric.y() >= 0 && barycentric.z() >= 0)
                {
                    inside = true;

                    float z = 1 / (barycentric.x() / a_world.z() + barycentric.y() / b_world.z() + barycentric.z() / c_world.z());
                    float alpha = barycentric.x() / a_world.z() * z;
                    float beta = barycentric.y() / b_world.z() * z;
                    float gamma = barycentric.z() / c_world.z() * z;

                    if (z > zbuffer[(v * width + u) * num_samples + i])
                    {
                        Eigen::Vector3f world_coords = a_world * alpha + b_world * beta + c_world * gamma;
                        Eigen::Vector2f uv = a_uv * alpha + b_uv * beta + c_uv * gamma;
                        Eigen::Vector3f diffuse = model.get_diffuse(uv);
                        float specular = model.get_specular(uv);
                        Eigen::Vector3f normal = model.get_normal(uv);

                        ShadingPoint sp{
                            world_coords,
                            normal,
                            diffuse,
                            specular,
                        };

                        // Phong shading
                        TGAColor color = phong_shading(sp, light, camera);
                        framebuffer[(v * width + u) * num_samples + i] = color;
                        zbuffer[(v * width + u) * num_samples + i] = z;
                    }
                }
            }
            if (inside)
            {
                TGAColor color = black;
                for (int i = 0; i < num_samples; i++)
                    color = color + framebuffer[(v * width + u) * num_samples + i] * (1.f / num_samples);
                image.set(u, v, color);
            }
        }
    }
}
