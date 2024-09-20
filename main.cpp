#include <iostream>
#include <array>
#include "tgaimage.hpp"
#include "model.hpp"
#include "shader.hpp"

const float eps = 1e-4;
const int width = 800;
const int height = 800;

int main(int argc, char **argv)
{
    // initialize image
    TGAImage image(width, height, TGAImage::RGB);

    // load model and its texture
    Model model("../objects/diablo3_pose.obj");

    // camera
    Camera camera;
    camera.pos = Eigen::Vector3f(0, 0, 5);
    camera.look_at = Eigen::Vector3f(0, 0, -1);
    camera.up = Eigen::Vector3f(0, 1, 0);

    // light
    Light light;
    light.pos = Eigen::Vector3f(0, 10, 10);
    light.intensity = Eigen::Vector3f(300, 300, 300);

    // frustum
    const float near = -3;
    const float far = -7;
    const float fovY = 25;        // degrees
    const float aspect_ratio = 1; // view width / view height

    // view transformation
    Eigen::Matrix4f rot = model_transform(Eigen::Vector3f(-1, 1, 0), 0, Eigen::Vector3f(1, 1, 1), Eigen::Vector3f(0, 0, 0));
    Eigen::Matrix4f cam = camera_transform(camera);
    Eigen::Matrix4f rigid = cam * rot;
    Eigen::Matrix4f persp = perspective_transformation(near, far, fovY, aspect_ratio);
    Eigen::Matrix4f viewport = viewport_transformation(width, height);
    Eigen::Matrix4f mvp = viewport * persp;
    model.transform(rigid, true);
    model.transform(mvp);

    // MSAA
    Eigen::Vector2f msaa_bias[4] = {
        Eigen::Vector2f(+0.25, +0.25),
        Eigen::Vector2f(-0.25, +0.25),
        Eigen::Vector2f(-0.25, -0.25),
        Eigen::Vector2f(+0.25, -0.25)};

    // z-buffer
    float *zbuffer = new float[width * height * 4];
    std::fill(zbuffer, zbuffer + width * height * 4, -std::numeric_limits<float>::max());

    // rendered image buffer
    TGAColor *framebuffer = new TGAColor[width * height * 4];
    std::fill(framebuffer, framebuffer + width * height * 4, black);

    for (int i = 0; i < model.num_faces(); i++)
    {
        std::array<Eigen::Vector3i, 3> face = model.get_face(i);
        Eigen::Vector3i verts = face[0];
        Eigen::Vector3i texts = face[1];
        Eigen::Vector3i norms = face[2];

        Triangle tri;
        std::array<Eigen::Vector3f, 3> vert_world_coords;

        for (int j = 0; j < 3; j++)
        {
            tri.v[j] = model.get_vert(verts[j]);
            vert_world_coords[j] = model.get_vert_world_coords(verts[j]);
            tri.t[j] = model.get_texture(texts[j]);
            tri.n[j] = model.get_normal(norms[j]);
        }

        triangle(tri, vert_world_coords, light, camera, msaa_bias, model, image, framebuffer, zbuffer);
    }

    image.flip_vertically();
    image.write_tga_file("../outputs/output.tga");

    delete[] zbuffer;
    return 0;
}