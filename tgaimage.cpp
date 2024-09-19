#include <iostream>
#include <fstream>
#include <cstring>
#include <ctime>
#include <cmath>
#include "tgaimage.hpp"

TGAColor::TGAColor(const unsigned char *p, int bpp) : val(0), bytespp(bpp)
{
    try
    {
        for (int i = 0; i < bpp; i++)
        {
            raw[i] = p[i];
        }
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << "\n";
    }
}

bool TGAImage::load_rle_data(std::ifstream &in)
{
    unsigned long long pixelcount = width * height;
    unsigned long long currentpixel = 0;
    unsigned long long currentbyte = 0;
    TGAColor colorbuffer;

    while (currentpixel < pixelcount)
    {
        unsigned char chunk_header = 0;
        chunk_header = in.get();

        if (!in.good())
        {
            std::cerr << "an error occured while reading the data\n";
            return false;
        }

        if (chunk_header < 128)
        {
            chunk_header += 1;
            for (int i = 0; i < chunk_header; i++)
            {
                in.read((char *)colorbuffer.raw, bytespp);

                if (!in.good())
                {
                    std::cerr << "an error occured while reading the header\n";
                    return false;
                }

                for (int t = 0; t < bytespp; t++)
                    data[currentbyte++] = colorbuffer.raw[t];
                currentpixel++;

                if (currentpixel > pixelcount)
                {
                    std::cerr << "Too many pixels read\n";
                    return false;
                }
            }
        }
        else
        {
            chunk_header -= 127;
            in.read((char *)colorbuffer.raw, bytespp);

            if (!in.good())
            {
                std::cerr << "an error occured while reading the header\n";
                return false;
            }

            for (int i = 0; i < chunk_header; i++)
            {
                for (int t = 0; t < bytespp; t++)
                    data[currentbyte++] = colorbuffer.raw[t];
                currentpixel++;

                if (currentpixel > pixelcount)
                {
                    std::cerr << "Too many pixels read\n";
                    return false;
                }
            }
        }
    }
    return true;
}

bool TGAImage::unload_rle_data(std::ofstream &out)
{
    const unsigned char max_chunk_length = 128;
    unsigned long long pixelcount = width * height;
    unsigned long long currentpixel = 0;
    while (currentpixel < pixelcount)
    {
        unsigned long long chunk_start = currentpixel * bytespp;
        unsigned long long currentbyte = currentpixel * bytespp;
        unsigned char run_length = 1;
        bool raw = true;

        while (currentpixel + run_length < pixelcount && run_length < max_chunk_length)
        {
            bool succ_eq = true;
            for (int t = 0; succ_eq && t < bytespp; t++)
            {
                succ_eq = (data[currentbyte + t] == data[currentbyte + t + bytespp]);
            }
            currentbyte += bytespp;
            if (1 == run_length)
            {
                raw = !succ_eq;
            }
            if (raw && succ_eq)
            {
                run_length--;
                break;
            }
            if (!raw && !succ_eq)
            {
                break;
            }
            run_length++;
        }

        currentpixel += run_length;

        out.put(raw ? run_length - 1 : run_length + 127);
        if (!out.good())
        {
            std::cerr << "can't dump the tga file\n";
            return false;
        }

        out.write((char *)(data + chunk_start), (raw ? run_length * bytespp : bytespp));
        if (!out.good())
        {
            std::cerr << "can't dump the tga file\n";
            return false;
        }
    }
    return true;
}

TGAImage::TGAImage(const TGAImage &img) : data(nullptr), width(img.width), height(img.height), bytespp(img.bytespp)
{
    unsigned long long bytes = img.width * img.height * img.bytespp;
    data = new unsigned char[bytes];
    unsigned char *img_data = img.data;
    memcpy(data, img_data, bytes);
}

TGAImage::~TGAImage()
{
    delete[] data;
}

TGAImage &TGAImage::operator=(const TGAImage &img)
{
    this->width = img.width;
    this->height = img.height;
    this->bytespp = img.bytespp;
    unsigned long long bytes = img.width * img.height * img.bytespp;
    data = new unsigned char[bytes];
    unsigned char *img_data = img.data;
    memcpy(data, img_data, bytes);
    return *this;
}

bool TGAImage::read_tga_file(const char *filename)
{
    if (data)
        delete[] data;

    data = nullptr;
    std::ifstream in;
    in.open(filename, std::ios::binary);

    if (!in.is_open())
    {
        in.close();
        std::cerr << "can't open file " << filename << "\n";
        return false;
    }

    TGAHeader header;
    in.read((char *)&header, sizeof(header));

    if (!in.good())
    {
        in.close();
        std::cerr << "an error occured while reading the header" << "\n";
        return false;
    }

    width = header.width;
    height = header.height;

    if (header.bitsperpixel % 8 != 0)
    {
        in.close();
        std::cerr << "the number of bits per pixel is not a multiple of eight" << "\n";
        return false;
    }

    bytespp = header.bitsperpixel >> 3;

    if (width <= 0 || height <= 0 || (bytespp != GRAYSCALE && bytespp != RGB && bytespp != RGBA))
    {
        in.close();
        std::cerr << "bad bpp (or width/height) value\n";
        return false;
    }

    unsigned long long bytes = bytespp * width * height;
    data = new unsigned char[bytes];

    if (3 == header.datatypecode || 2 == header.datatypecode)
    {
        in.read((char *)data, bytes);
        if (!in.good())
        {
            in.close();
            std::cerr << "an error occured while reading the data\n";
            return false;
        }
    }
    else if (10 == header.datatypecode || 11 == header.datatypecode)
    {
        if (!load_rle_data(in))
        {
            in.close();
            std::cerr << "an error occured while reading the data\n";
            return false;
        }
    }
    else
    {
        in.close();
        std::cerr << "unknown file format " << (int)header.datatypecode << "\n";
        return false;
    }

    if (!(header.imagedescriptor & 0x20))
    {
        flip_vertically();
    }
    if (header.imagedescriptor & 0x10)
    {
        flip_horizontally();
    }

    std::cerr << width << "*" << height << "/" << bytespp * 8 << "\n";
    in.close();

    return true;
}

bool TGAImage::write_tga_file(const char *filename, bool rle)
{
    unsigned char developer_area_ref[4] = {0, 0, 0, 0};
    unsigned char extension_area_ref[4] = {0, 0, 0, 0};
    unsigned char footer[18] = {'T', 'R', 'U', 'E', 'V', 'I', 'S', 'I', 'O', 'N', '-', 'X', 'F', 'I', 'L', 'E', '.', '\0'};
    std::ofstream out;

    out.open(filename, std::ios::binary);
    if (!out.is_open())
    {
        std::cerr << "can't open file " << filename << "\n";
        out.close();
        return false;
    }

    TGAHeader header;
    memset((void *)&header, 0, sizeof(header));
    header.bitsperpixel = bytespp << 3;
    header.width = width;
    header.height = height;
    header.datatypecode = (bytespp == GRAYSCALE ? (rle ? 11 : 3) : (rle ? 10 : 2));
    header.imagedescriptor = 0x20; // top-left origin

    out.write((char *)&header, sizeof(header));
    if (!out.good())
    {
        out.close();
        std::cerr << "can't dump the tga file\n";
        return false;
    }

    if (!rle)
    {
        out.write((char *)data, width * height * bytespp);
        if (!out.good())
        {
            std::cerr << "can't unload raw data\n";
            out.close();
            return false;
        }
    }
    else
    {
        if (!unload_rle_data(out))
        {
            out.close();
            std::cerr << "can't unload rle data\n";
            return false;
        }
    }

    out.write((char *)developer_area_ref, sizeof(developer_area_ref));
    if (!out.good())
    {
        std::cerr << "can't dump the tga file\n";
        out.close();
        return false;
    }

    out.write((char *)extension_area_ref, sizeof(extension_area_ref));
    if (!out.good())
    {
        std::cerr << "can't dump the tga file\n";
        out.close();
        return false;
    }

    out.write((char *)footer, sizeof(footer));
    if (!out.good())
    {
        std::cerr << "can't dump the tga file\n";
        out.close();
        return false;
    }

    out.close();
    return true;
}

bool TGAImage::flip_horizontally()
{
    if (!data)
        return false;
    int half = width >> 1;
    for (int h = 0; h < height; h++)
    {
        for (int i = 0; i < half; i++)
        {
            unsigned char left = data[h * width + i];
            data[h * width + i] = data[(h + 1) * width - 1 - i];
            data[(h + 1) * width - 1 - i] = left;
        }
    }
    return true;
}

bool TGAImage::flip_vertically()
{
    if (!data)
        return false;
    int half = height >> 1;
    unsigned long long bytes_per_line = width * bytespp;
    unsigned char *horizontal_line_buffer = new unsigned char[bytes_per_line];
    for (int h = 0; h < half; h++)
    {
        unsigned long long l1 = h * bytes_per_line;
        unsigned long long l2 = (height - 1 - h) * bytes_per_line;
        memmove((void *)horizontal_line_buffer, (void *)(data + l1), bytes_per_line);
        memmove((void *)(data + l1), (void *)(data + l2), bytes_per_line);
        memmove((void *)(data + l2), (void *)horizontal_line_buffer, bytes_per_line);
    }
    delete[] horizontal_line_buffer;
    return true;
}

bool TGAImage::scale(int w, int h)
{
    if (w <= 0 || h <= 0)
        return false;

    unsigned char *tdata = new unsigned char[w * h * bytespp];
    int nscanline = 0;
    int oscanline = 0;
    int erry = 0;
    unsigned long nlinebytes = w * bytespp;
    unsigned long olinebytes = width * bytespp;

    for (int j = 0; j < height; j++)
    {
        int errx = width - w;
        int nx = -bytespp;
        int ox = -bytespp;

        for (int i = 0; i < width; i++)
        {
            ox += bytespp;
            errx += w;
            while (errx >= (int)width)
            {
                errx -= width;
                nx += bytespp;
                try
                {
                    memcpy(tdata + nscanline + nx, data + oscanline + ox, bytespp);
                }
                catch (std::exception &e)
                {
                    std::cerr << e.what() << "\n";
                }
            }
        }

        erry += h;
        oscanline += olinebytes;

        while (erry >= (int)height)
        {
            if (erry >= (int)height << 1) // it means we jump over a scanline
                memcpy(tdata + nscanline + nlinebytes, tdata + nscanline, nlinebytes);
            erry -= height;
            nscanline += nlinebytes;
        }
    }

    delete[] data;
    data = tdata;
    width = w;
    height = h;
    return true;
}

TGAColor TGAImage::get(int x, int y) const
{
    if (x < 0 || y < 0 || x >= width || y >= height)
    {
        return TGAColor();
    }
    TGAColor res;
    try
    {
        res = TGAColor(data + (y * width + x) * bytespp, bytespp);
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << "\n";
    }
    return res;
}

bool TGAImage::set(int x, int y, const TGAColor &c)
{
    if (x < 0 || y < 0 || x >= width || y >= height)
    {
        return false;
    }
    try
    {
        memcpy(&data[(y * width + x) * bytespp], (void *)c.raw, (size_t)bytespp);
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << "\n";
    }
    return false;
}

int TGAImage::get_width() const
{
    return width;
}

int TGAImage::get_height() const
{
    return height;
}

int TGAImage::get_bytespp() const
{
    return bytespp;
}

unsigned char *TGAImage::buffer() const
{
    return data;
}

void TGAImage::clear()
{
    memset(data, 0, width * height * bytespp);
}
