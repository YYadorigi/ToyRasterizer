#ifndef _TGA_IMAGE_H_
#define _TGA_IMAGE_H_

#include <fstream>
#include <iostream>

#pragma pack(push, 1)
struct TGAHeader
{
    char idlength;
    char colormaptype;
    char datatypecode;
    short colormaporigin;
    short colormaplength;
    char colormapdepth;
    short x_origin;
    short y_origin;
    short width;
    short height;
    char bitsperpixel;
    char imagedescriptor;
};
#pragma pack(pop)

struct TGAColor
{
    union
    {
        struct
        {
            unsigned char b, g, r, a;
        };
        unsigned char raw[4];
        unsigned int val;
    };
    int bytespp; // bytes per pixel

    TGAColor() : val(0), bytespp(1)
    {
    }

    TGAColor(unsigned char R, unsigned char G, unsigned char B, unsigned char A) : r(R), g(G), b(B), a(A), bytespp(4)
    {
    }

    TGAColor(int v, int bpp) : val(v), bytespp(bpp)
    {
    }

    TGAColor(const TGAColor &c) : val(c.val), bytespp(c.bytespp)
    {
    }

    TGAColor(const unsigned char *p, int bpp);

    TGAColor &operator=(const TGAColor &c)
    {
        this->val = c.val;
        this->bytespp = c.bytespp;
        return *this;
    }
};

class TGAImage
{
protected:
    unsigned char *data;
    int width;
    int height;
    int bytespp;

    bool load_rle_data(std::ifstream &in);
    bool unload_rle_data(std::ofstream &out);

public:
    enum Format
    {
        GRAYSCALE = 1,
        RGB = 3,
        RGBA = 4
    };

    TGAImage() : data(nullptr), width(0), height(0), bytespp(0)
    {
    }

    TGAImage(int w, int h, int bpp) : data(nullptr), width(w), height(h), bytespp(bpp)
    {
        unsigned long long bytes = w * h * bpp;
        data = new unsigned char[bytes];
        memset(data, 0, bytes);
    }

    TGAImage(const TGAImage &img);

    ~TGAImage();

    TGAImage &operator=(const TGAImage &img);

    bool read_tga_file(const char *filename);
    bool write_tga_file(const char *filename, bool rle = true);
    bool flip_horizontally();
    bool flip_vertically();
    bool scale(int w, int h);
    TGAColor get(int x, int y);
    bool set(int x, int y, TGAColor c);

    int get_width();
    int get_height();
    int get_bytespp();
    unsigned char *buffer();
    void clear();
};

#endif