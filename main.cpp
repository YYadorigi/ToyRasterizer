#include <iostream>
#include <array>
#include "tgaimage.hpp"
#include "model.hpp"
#include "shader.hpp"

const int width = 1080;
const int height = 1080;

int main(int argc, char **argv)
{
    // initialize image
    TGAImage image(width, height, TGAImage::RGB);

    // load model and its texture
    Model model("../objects/african_head.obj");

    // camera
    Camera camera;
    camera.pos = Eigen::Vector3f(0, 0, 5);
    camera.look_at = Eigen::Vector3f(0, 0, -1);
    camera.up = Eigen::Vector3f(0, 1, 0);

    // light
    Light light;
    light.pos = Eigen::Vector3f(10, 10, 10);
    light.intensity = Eigen::Vector3f(200, 200, 200); // RGB

    // frustum
    const float near = 3;         // distance to near plane
    const float far = 7;          // distance to far plane
    const float fovY = 30;        // degrees
    const float aspect_ratio = 1; // view width / view height

    // view transformation
    Eigen::Matrix4f rot = model_transform(Eigen::Vector3f(-1, 1, 0), 0, Eigen::Vector3f(1, 1, 1), Eigen::Vector3f(0, 0, 0));
    Eigen::Matrix4f cam = camera_transform(camera);

    Eigen::Matrix4f rigid = cam * rot;
    model.transform(rigid, true);
    light.pos = (rigid * Eigen::Vector4f(light.pos.x(), light.pos.y(), light.pos.z(), 1)).head(3);
    camera.pos = (rigid * Eigen::Vector4f(camera.pos.x(), camera.pos.y(), camera.pos.z(), 1)).head(3);

    Eigen::Matrix4f persp = perspective_transformation(-near, -far, fovY, aspect_ratio);
    Eigen::Matrix4f viewport = viewport_transformation(width, height);
    Eigen::Matrix4f proj = viewport * persp;
    model.transform(proj);

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

    // render image
    std::cerr << "rendering image" << std::endl;
    for (int i = 0; i < model.num_faces(); i++)
    {
        std::pair<Eigen::Vector3i, Eigen::Vector3i> face = model.get_face(i);
        Eigen::Vector3i verts = face.first;
        Eigen::Vector3i texts = face.second;

        Triangle tri{
            model.get_vert(verts.x()),
            model.get_vert(verts.y()),
            model.get_vert(verts.z()),
            model.get_texture(texts.x()),
            model.get_texture(texts.y()),
            model.get_texture(texts.z()),
        };

        std::array<Eigen::Vector3f, 3> vert_world_coords = {
            model.get_vert_world_coords(verts.x()),
            model.get_vert_world_coords(verts.y()),
            model.get_vert_world_coords(verts.z()),
        };

        triangle(tri, vert_world_coords, light, camera, msaa_bias, model, image, framebuffer, zbuffer);
    }

    image.flip_vertically();
    image.write_tga_file("../outputs/output.tga");

    delete[] zbuffer;
    return 0;
}