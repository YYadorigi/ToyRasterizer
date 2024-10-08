#include <Eigen/Dense>

struct PlanarTriangle
{
    Eigen::Vector2f v[3];

    PlanarTriangle() = default;

    PlanarTriangle(Eigen::Vector2f v0, Eigen::Vector2f v1, Eigen::Vector2f v2)
    {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
    }

    PlanarTriangle(Eigen::Vector2f *v)
    {
        if (v->size() != 3)
        {
            throw std::invalid_argument("Vectors must have size 3");
        }

        for (int i = 0; i < 3; i++)
        {
            this->v[i] = v[i];
        }
    }

    ~PlanarTriangle() = default;

    bool inside_triangle(const Eigen::Vector2f &p)
    {
        Eigen::Vector2f a = v[0];
        Eigen::Vector2f b = v[1];
        Eigen::Vector2f c = v[2];

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

    Eigen::Vector3f barycentric_coords(const Eigen::Vector2f &p)
    {
        Eigen::Vector2f a = v[0];
        Eigen::Vector2f b = v[1];
        Eigen::Vector2f c = v[2];

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
};

struct Triangle
{
    Eigen::Vector3f v[3];
    Eigen::Vector2f t[3];

    Triangle() = default;

    Triangle(Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2,
             Eigen::Vector2f t0, Eigen::Vector2f t1, Eigen::Vector2f t2)
    {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
        t[0] = t0;
        t[1] = t1;
        t[2] = t2;
    }

    Triangle(Eigen::Vector3f *v, Eigen::Vector2f *t)
    {
        if (v->size() != 3 || t->size() != 3)
        {
            throw std::invalid_argument("Vectors must have size 3");
        }

        for (int i = 0; i < 3; i++)
        {
            this->v[i] = v[i];
            this->t[i] = t[i];
        }
    }

    ~Triangle() = default;
};