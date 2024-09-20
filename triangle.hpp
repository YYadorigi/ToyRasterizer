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
};

struct Triangle
{
    Eigen::Vector3f v[3];
    Eigen::Vector3f t[3];
    Eigen::Vector3f n[3];

    Triangle() = default;

    Triangle(Eigen::Vector3f v0, Eigen::Vector3f v1, Eigen::Vector3f v2,
             Eigen::Vector3f t0, Eigen::Vector3f t1, Eigen::Vector3f t2,
             Eigen::Vector3f n0, Eigen::Vector3f n1, Eigen::Vector3f n2)
    {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
        t[0] = t0;
        t[1] = t1;
        t[2] = t2;
        n[0] = n0;
        n[1] = n1;
        n[2] = n2;
    }

    Triangle(Eigen::Vector3f *v, Eigen::Vector3f *t, Eigen::Vector3f *n)
    {
        if (v->size() != 3 || t->size() != 3 || n->size() != 3)
        {
            throw std::invalid_argument("Vectors must have size 3");
        }

        for (int i = 0; i < 3; i++)
        {
            this->v[i] = v[i];
            this->t[i] = t[i];
            this->n[i] = n[i];
        }
    }

    ~Triangle() = default;
};