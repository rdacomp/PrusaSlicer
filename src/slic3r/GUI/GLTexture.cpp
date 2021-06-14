#include "libslic3r/libslic3r.h"

#include "GLTexture.hpp"
#include "3DScene.hpp"
#include "OpenGLManager.hpp"

#include "libslic3r/Utils.hpp"

#include <GL/glew.h>

#include <wx/image.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>
#if ENABLE_TEXTURED_VOLUMES
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/beast/core/detail/base64.hpp>
#include <boost/nowide/fstream.hpp>

#include <png.h>
#endif // ENABLE_TEXTURED_VOLUMES

#include <vector>
#include <algorithm>
#include <thread>

#define STB_DXT_IMPLEMENTATION
#include "stb_dxt/stb_dxt.h"

#include "nanosvg/nanosvg.h"
#include "nanosvg/nanosvgrast.h"

namespace Slic3r {
namespace GUI {

void GLTexture::Compressor::reset()
{
	if (m_thread.joinable()) {
        m_abort_compressing = true;
		m_thread.join();
	    m_levels.clear();
	    m_num_levels_compressed = 0;
	    m_abort_compressing = false;
	}
	assert(m_levels.empty());
	assert(m_abort_compressing == false);
	assert(m_num_levels_compressed == 0);
}

void GLTexture::Compressor::start_compressing()
{
	// The worker thread should be stopped already.
	assert(! m_thread.joinable());
	assert(! m_levels.empty());
	assert(m_abort_compressing == false);
	assert(m_num_levels_compressed == 0);
	if (! m_levels.empty()) {
		std::thread thrd(&GLTexture::Compressor::compress, this);
    	m_thread = std::move(thrd);
    }
}

bool GLTexture::Compressor::unsent_compressed_data_available() const
{
	if (m_levels.empty())
		return false;
	// Querying the atomic m_num_levels_compressed value synchronizes processor caches, so that the data of m_levels modified by the worker thread are accessible to the calling thread.
	unsigned int num_compressed = m_num_levels_compressed;
	for (unsigned int i = 0; i < num_compressed; ++ i)
        if (! m_levels[i].sent_to_gpu && ! m_levels[i].compressed_data.empty())
            return true;
    return false;
}

void GLTexture::Compressor::send_compressed_data_to_gpu()
{
    // this method should be called inside the main thread of Slicer or a new OpenGL context (sharing resources) would be needed
	if (m_levels.empty())
		return;

    glsafe(::glPixelStorei(GL_UNPACK_ALIGNMENT, 1));
    glsafe(::glBindTexture(GL_TEXTURE_2D, m_texture.m_id));
	// Querying the atomic m_num_levels_compressed value synchronizes processor caches, so that the dat of m_levels modified by the worker thread are accessible to the calling thread.
	int num_compressed = (int)m_num_levels_compressed;
    for (int i = 0; i < num_compressed; ++ i) {
        Level& level = m_levels[i];
        if (! level.sent_to_gpu && ! level.compressed_data.empty()) {
            glsafe(::glCompressedTexSubImage2D(GL_TEXTURE_2D, (GLint)i, 0, 0, (GLsizei)level.w, (GLsizei)level.h, GL_COMPRESSED_RGBA_S3TC_DXT5_EXT, (GLsizei)level.compressed_data.size(), (const GLvoid*)level.compressed_data.data()));
            glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, i));
            glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, (i > 0) ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR));
            level.sent_to_gpu = true;
            // we are done with the compressed data, we can discard it
            level.compressed_data.clear();
        }
    }
    glsafe(::glBindTexture(GL_TEXTURE_2D, 0));

    if (num_compressed == (int)m_levels.size())
        // Finalize the worker thread, close it.
    	this->reset();
}

void GLTexture::Compressor::compress()
{
    // reference: https://github.com/Cyan4973/RygsDXTc

    assert(m_num_levels_compressed == 0);
    assert(m_abort_compressing == false);

    for (Level& level : m_levels) {
        if (m_abort_compressing)
            break;

        // stb_dxt library, despite claiming that the needed size of the destination buffer is equal to (source buffer size)/4,
        // crashes if doing so, requiring a minimum of 64 bytes and up to a third of the source buffer size, so we set the destination buffer initial size to be half the source buffer size
        level.compressed_data = std::vector<unsigned char>(std::max((unsigned int)64, (unsigned int)level.src_data.size() / 2), 0);
        int compressed_size = 0;
        rygCompress(level.compressed_data.data(), level.src_data.data(), level.w, level.h, 1, compressed_size);
        level.compressed_data.resize(compressed_size);

        // we are done with the source data, we can discard it
        level.src_data.clear();
        ++ m_num_levels_compressed;
    }
}

GLTexture::Quad_UVs GLTexture::FullTextureUVs = { { 0.0f, 1.0f }, { 1.0f, 1.0f }, { 1.0f, 0.0f }, { 0.0f, 0.0f } };

bool GLTexture::load_from_file(const std::string& filename, bool use_mipmaps, ECompressionType compression_type, bool apply_anisotropy)
{
    reset();

    if (!boost::filesystem::exists(filename))
        return false;

    if (boost::algorithm::iends_with(filename, ".png"))
        return load_from_png(filename, use_mipmaps, compression_type, apply_anisotropy);
    else
        return false;
}

bool GLTexture::load_from_svg_file(const std::string& filename, bool use_mipmaps, bool compress, bool apply_anisotropy, unsigned int max_size_px)
{
    reset();

    if (!boost::filesystem::exists(filename))
        return false;

    if (boost::algorithm::iends_with(filename, ".svg"))
        return load_from_svg(filename, use_mipmaps, compress, apply_anisotropy, max_size_px);
    else
        return false;
}

bool GLTexture::load_from_svg_files_as_sprites_array(const std::vector<std::string>& filenames, const std::vector<std::pair<int, bool>>& states, unsigned int sprite_size_px, bool compress)
{
    reset();

    if (filenames.empty() || states.empty() || sprite_size_px == 0)
        return false;

    // every tile needs to have a 1px border around it to avoid artifacts when linear sampling on its edges
    unsigned int sprite_size_px_ex = sprite_size_px + 1;

    m_width = 1 + (int)(sprite_size_px_ex * states.size());
    m_height = 1 + (int)(sprite_size_px_ex * filenames.size());

    int n_pixels = m_width * m_height;
    int sprite_n_pixels = sprite_size_px_ex * sprite_size_px_ex;
    int sprite_stride = sprite_size_px_ex * 4;
    int sprite_bytes = sprite_n_pixels * 4;

    if (n_pixels <= 0) {
        reset();
        return false;
    }

    std::vector<unsigned char> data(n_pixels * 4, 0);
    std::vector<unsigned char> sprite_data(sprite_bytes, 0);
    std::vector<unsigned char> sprite_white_only_data(sprite_bytes, 0);
    std::vector<unsigned char> sprite_gray_only_data(sprite_bytes, 0);
    std::vector<unsigned char> output_data(sprite_bytes, 0);

    NSVGrasterizer* rast = nsvgCreateRasterizer();
    if (rast == nullptr) {
        reset();
        return false;
    }

    int sprite_id = -1;
    for (const std::string& filename : filenames) {
        ++sprite_id;

        if (!boost::filesystem::exists(filename))
            continue;

        if (!boost::algorithm::iends_with(filename, ".svg"))
            continue;

        NSVGimage* image = nsvgParseFromFile(filename.c_str(), "px", 96.0f);
        if (image == nullptr)
            continue;

        float scale = (float)sprite_size_px / std::max(image->width, image->height);

        // offset by 1 to leave the first pixel empty (both in x and y)
        nsvgRasterize(rast, image, 1, 1, scale, sprite_data.data(), sprite_size_px, sprite_size_px, sprite_stride);

        // makes white only copy of the sprite
        ::memcpy((void*)sprite_white_only_data.data(), (const void*)sprite_data.data(), sprite_bytes);
        for (int i = 0; i < sprite_n_pixels; ++i) {
            int offset = i * 4;
            if (sprite_white_only_data.data()[offset] != 0)
                ::memset((void*)&sprite_white_only_data.data()[offset], 255, 3);
        }

        // makes gray only copy of the sprite
        ::memcpy((void*)sprite_gray_only_data.data(), (const void*)sprite_data.data(), sprite_bytes);
        for (int i = 0; i < sprite_n_pixels; ++i) {
            int offset = i * 4;
            if (sprite_gray_only_data.data()[offset] != 0)
                ::memset((void*)&sprite_gray_only_data.data()[offset], 128, 3);
        }

        int sprite_offset_px = sprite_id * (int)sprite_size_px_ex * m_width;
        int state_id = -1;
        for (const std::pair<int, bool>& state : states) {
            ++state_id;

            // select the sprite variant
            std::vector<unsigned char>* src = nullptr;
            switch (state.first)
            {
            case 1:  { src = &sprite_white_only_data; break; }
            case 2:  { src = &sprite_gray_only_data; break; }
            default: { src = &sprite_data; break; }
            }

            ::memcpy((void*)output_data.data(), (const void*)src->data(), sprite_bytes);
            // applies background, if needed
            if (state.second) {
                float inv_255 = 1.0f / 255.0f;
                // offset by 1 to leave the first pixel empty (both in x and y)
                for (unsigned int r = 1; r <= sprite_size_px; ++r) {
                    unsigned int offset_r = r * sprite_size_px_ex;
                    for (unsigned int c = 1; c <= sprite_size_px; ++c) {
                        unsigned int offset = (offset_r + c) * 4;
                        float alpha = (float)output_data.data()[offset + 3] * inv_255;
                        output_data.data()[offset + 0] = (unsigned char)(output_data.data()[offset + 0] * alpha);
                        output_data.data()[offset + 1] = (unsigned char)(output_data.data()[offset + 1] * alpha);
                        output_data.data()[offset + 2] = (unsigned char)(output_data.data()[offset + 2] * alpha);
                        output_data.data()[offset + 3] = (unsigned char)(128 * (1.0f - alpha) + output_data.data()[offset + 3] * alpha);
                    }
                }
            }

            int state_offset_px = sprite_offset_px + state_id * sprite_size_px_ex;
            for (int j = 0; j < (int)sprite_size_px_ex; ++j) {
                ::memcpy((void*)&data.data()[(state_offset_px + j * m_width) * 4], (const void*)&output_data.data()[j * sprite_stride], sprite_stride);
            }
        }

        nsvgDelete(image);
    }

    nsvgDeleteRasterizer(rast);

    // sends data to gpu
    glsafe(::glPixelStorei(GL_UNPACK_ALIGNMENT, 1));
    glsafe(::glGenTextures(1, &m_id));
    glsafe(::glBindTexture(GL_TEXTURE_2D, m_id));
    if (compress && GLEW_EXT_texture_compression_s3tc)
        glsafe(::glTexImage2D(GL_TEXTURE_2D, 0, GL_COMPRESSED_RGBA_S3TC_DXT5_EXT, (GLsizei)m_width, (GLsizei)m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const void*)data.data()));
    else
        glsafe(::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, (GLsizei)m_width, (GLsizei)m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const void*)data.data()));
    glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0));
    glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

    glsafe(::glBindTexture(GL_TEXTURE_2D, 0));

    m_source = filenames.front();
    
#if 0
    // debug output
    static int pass = 0;
    ++pass;

    wxImage output(m_width, m_height);
    output.InitAlpha();

    for (int h = 0; h < m_height; ++h) {
        int px_h = h * m_width;
        for (int w = 0; w < m_width; ++w) {
            int offset = (px_h + w) * 4;
            output.SetRGB(w, h, data.data()[offset + 0], data.data()[offset + 1], data.data()[offset + 2]);
            output.SetAlpha(w, h, data.data()[offset + 3]);
        }
    }

    std::string out_filename = resources_dir() + "/icons/test_" + std::to_string(pass) + ".png";
    output.SaveFile(out_filename, wxBITMAP_TYPE_PNG);
#endif // 0

    return true;
}

void GLTexture::reset()
{
    if (m_id != 0)
        glsafe(::glDeleteTextures(1, &m_id));

    m_id = 0;
    m_width = 0;
    m_height = 0;
    m_source = "";
    m_compressor.reset();

#if ENABLE_TEXTURED_VOLUMES
    on_reset();
#endif // ENABLE_TEXTURED_VOLUMES
}

void GLTexture::render_texture(unsigned int tex_id, float left, float right, float bottom, float top)
{
    render_sub_texture(tex_id, left, right, bottom, top, FullTextureUVs);
}

void GLTexture::render_sub_texture(unsigned int tex_id, float left, float right, float bottom, float top, const GLTexture::Quad_UVs& uvs)
{
    glsafe(::glEnable(GL_BLEND));
    glsafe(::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

    glsafe(::glEnable(GL_TEXTURE_2D));
    glsafe(::glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE));

    glsafe(::glBindTexture(GL_TEXTURE_2D, (GLuint)tex_id));

    ::glBegin(GL_QUADS);
    ::glTexCoord2f(uvs.left_bottom.u, uvs.left_bottom.v); ::glVertex2f(left, bottom);
    ::glTexCoord2f(uvs.right_bottom.u, uvs.right_bottom.v); ::glVertex2f(right, bottom);
    ::glTexCoord2f(uvs.right_top.u, uvs.right_top.v); ::glVertex2f(right, top);
    ::glTexCoord2f(uvs.left_top.u, uvs.left_top.v); ::glVertex2f(left, top);
    glsafe(::glEnd());

    glsafe(::glBindTexture(GL_TEXTURE_2D, 0));

    glsafe(::glDisable(GL_TEXTURE_2D));
    glsafe(::glDisable(GL_BLEND));
}

bool GLTexture::load_from_png(const std::string& filename, bool use_mipmaps, ECompressionType compression_type, bool apply_anisotropy)
{
    const bool compression_enabled = (compression_type != ECompressionType::None) && GLEW_EXT_texture_compression_s3tc;

    // Load a PNG with an alpha channel.
    wxImage image;
    if (!image.LoadFile(wxString::FromUTF8(filename.c_str()), wxBITMAP_TYPE_PNG))
        return false;

    m_width = image.GetWidth();
    m_height = image.GetHeight();

    bool requires_rescale = false;

    if (compression_enabled && compression_type == ECompressionType::MultiThreaded)
        requires_rescale = adjust_size_for_compression();

    if (requires_rescale)
        image = image.ResampleBicubic(m_width, m_height);

    const int n_pixels = m_width * m_height;
    if (n_pixels <= 0) {
        reset();
        return false;
    }

    std::vector<unsigned char> data;

    // Get RGB & alpha raw data from wxImage, pack them into an array.
    auto copy_data = [](wxImage& image, std::vector<unsigned char>& data, int n_pixels) {
        unsigned char* img_rgb = image.GetData();
        unsigned char* img_alpha = image.GetAlpha();
        data.resize(n_pixels * 4);
        for (int i = 0; i < n_pixels; ++i) {
            const int data_id = i * 4;
            const int img_id = i * 3;
            data[data_id + 0] = img_rgb[img_id + 0];
            data[data_id + 1] = img_rgb[img_id + 1];
            data[data_id + 2] = img_rgb[img_id + 2];
            data[data_id + 3] = (img_alpha != nullptr) ? img_alpha[i] : 255;
        }
    };

    copy_data(image, data, n_pixels);

    send_to_gpu(data, use_mipmaps, compression_type, apply_anisotropy, [&image, copy_data](int lod_w, int lod_h, std::vector<unsigned char>& data) {
        wxImage im = image.ResampleBicubic(lod_w, lod_h);
        copy_data(im, data, lod_w * lod_h);
        });

    m_source = filename;

    if (compression_enabled && compression_type == ECompressionType::MultiThreaded)
        // start asynchronous compression
        m_compressor.start_compressing();

    return true;
}

#if ENABLE_TEXTURED_VOLUMES
bool GLTexture::load_from_png_memory(const std::vector<unsigned char>& png_data, bool use_mipmaps, ECompressionType compression_type, bool apply_anisotropy)
{
    bool compression_enabled = (compression_type != ECompressionType::None) && GLEW_EXT_texture_compression_s3tc;

    // for reference, see: http://pulsarengine.com/2009/01/reading-png-images-from-memory/
    static const size_t PngSignatureLength = 8;

    // check PNG signature
    if (png_data.size() < PngSignatureLength) {
        BOOST_LOG_TRIVIAL(error) << "Found invalid data while loading png from memory";
        return false;
    }

    std::array<unsigned char, PngSignatureLength> png_signature;
    ::memcpy((void*)png_signature.data(), (const void*)png_data.data(), PngSignatureLength);

    if (!png_check_sig(png_signature.data(), PngSignatureLength)) {
        BOOST_LOG_TRIVIAL(error) << "Found invalid signature while loading png from memory";
        return false;
    }

    // create PNG file read struct (memory is allocated by libpng)
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (png_ptr == nullptr) {
        BOOST_LOG_TRIVIAL(error) << "Unable to create read struct while loading png from memory";
        return false;
    }

    // create PNG image data info struct (memory is allocated by libpng)
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == nullptr) {
        // libpng must free file info struct memory before we bail
        png_destroy_read_struct(&png_ptr, nullptr, nullptr);
        BOOST_LOG_TRIVIAL(error) << "Unable to create info struct while loading png from memory";
        return false;
    }

    // helper class to read png data from memory
    class PngRead
    {
        const std::vector<unsigned char>& m_data;
        size_t m_pos{ PngSignatureLength }; // skip signature

    public:
        explicit PngRead(const std::vector<unsigned char>& data) : m_data(data) {}
        void read(void* dest, size_t num_bytes) {
            ::memcpy(dest, (const void*)&m_data[m_pos], num_bytes);
            m_pos += num_bytes;
        }
    };

    PngRead png_read(png_data);

    // set custom read function
    png_set_read_fn(png_ptr, (png_voidp)(&png_read), [](png_structp png_ptr, png_bytep out_bytes, png_size_t byte_count_to_read) {
        png_voidp io_ptr = png_get_io_ptr(png_ptr);
        assert(io_ptr != nullptr);
        PngRead* reader = (PngRead*)io_ptr;
        reader->read(out_bytes, static_cast<size_t>(byte_count_to_read));
        });

    // tell libpng to skip signature
    png_set_sig_bytes(png_ptr, PngSignatureLength);

    // read info struct
    png_read_info(png_ptr, info_ptr);

    // read header
    png_uint_32 width = 0;
    png_uint_32 height = 0;
    int bitDepth = 0;
    int colorType = -1;
    png_uint_32 res = png_get_IHDR(png_ptr, info_ptr, &width, &height, &bitDepth, &colorType, nullptr, nullptr, nullptr);
    if (res != 1) {
        png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
        BOOST_LOG_TRIVIAL(error) << "Unable to read header while loading png from memory";
        return false;
    }

    // check dimensions
    if (width == 0 || height == 0) {
        png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
        BOOST_LOG_TRIVIAL(error) << "Found invalid size while loading png from memory";
        return false;
    }

    // temporary constrain, see http://www.libpng.org/pub/png/libpng-1.2.5-manual.html to remove
    if (colorType != PNG_COLOR_TYPE_RGB && colorType != PNG_COLOR_TYPE_RGB_ALPHA) {
        png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
        BOOST_LOG_TRIVIAL(error) << "Found invalid color type while loading png from memory";
        return false;
    }

    // temporary constrain, see http://www.libpng.org/pub/png/libpng-1.2.5-manual.html to remove
    if (bitDepth != 8) {
        png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
        BOOST_LOG_TRIVIAL(error) << "Found bit depth while loading png from memory";
        return false;
    }

    // read image data
    std::vector<unsigned char> rgb_data(width * height * 3, 0);
    std::vector<unsigned char> alpha_data(width * height, 0);
    const png_uint_32 bytes_per_row = png_get_rowbytes(png_ptr, info_ptr);
    std::vector<unsigned char> row_data(bytes_per_row, 0);

    for (png_uint_32 row = 0; row < height; ++row) {
        png_read_row(png_ptr, (png_bytep)row_data.data(), nullptr);
        for (png_uint_32 col = 0; col < width; ++col) {
            png_uint_32 alpha_id = (height - 1 - row) * width + col;
            png_uint_32 rgb_id = 3 * alpha_id;
            switch (colorType)
            {
            case PNG_COLOR_TYPE_RGB:
            {
                png_uint_32 col_id = 3 * col;
                rgb_data[rgb_id + 0] = row_data[col_id + 0];
                rgb_data[rgb_id + 1] = row_data[col_id + 1];
                rgb_data[rgb_id + 2] = row_data[col_id + 2];
                alpha_data[alpha_id] = 255;
                break;
            }
            case PNG_COLOR_TYPE_RGB_ALPHA:
            {
                png_uint_32 col_id = 4 * col;
                rgb_data[rgb_id + 0] = row_data[col_id + 0];
                rgb_data[rgb_id + 1] = row_data[col_id + 1];
                rgb_data[rgb_id + 2] = row_data[col_id + 2];
                alpha_data[alpha_id] = row_data[col_id + 3];
                break;
            }
            }
        }
    }

    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);

    m_width = static_cast<int>(width);
    m_height = static_cast<int>(height);

    const int n_pixels = m_width * m_height;
    if (n_pixels <= 0) {
        reset();
        return false;
    }

    wxImage image(m_width, m_height);
    image.InitAlpha();
    image.SetData(rgb_data.data(), true);
    image.SetAlpha(alpha_data.data(), true);

    bool requires_rescale = false;

    if (compression_enabled && compression_type == ECompressionType::MultiThreaded)
        requires_rescale = adjust_size_for_compression();

    if (requires_rescale)
        image = image.ResampleBicubic(m_width, m_height);

#define DEBUG_OUTPUT 0
#if DEBUG_OUTPUT
    wxString out_file = m_source + ".png";
    wxImage out_image = image.Mirror(false);
    out_image.SaveFile(out_file, wxBITMAP_TYPE_PNG);
#endif // DEBUG_OUTPUT

    std::vector<unsigned char> data;

    // Get RGB & alpha raw data from wxImage, pack them into an array.
    auto copy_data = [](wxImage& image, std::vector<unsigned char>& data, int n_pixels) {
        unsigned char* img_rgb = image.GetData();
        unsigned char* img_alpha = image.GetAlpha();
        data.resize(n_pixels * 4);
        for (int i = 0; i < n_pixels; ++i) {
            const int data_id = i * 4;
            const int img_id = i * 3;
            data[data_id + 0] = img_rgb[img_id + 0];
            data[data_id + 1] = img_rgb[img_id + 1];
            data[data_id + 2] = img_rgb[img_id + 2];
            data[data_id + 3] = (img_alpha != nullptr) ? img_alpha[i] : 255;
        }
    };

    copy_data(image, data, n_pixels);

    send_to_gpu(data, use_mipmaps, compression_type, apply_anisotropy, [&image, copy_data](int lod_w, int lod_h, std::vector<unsigned char>& data) {
        wxImage im = image.ResampleBicubic(lod_w, lod_h);
        copy_data(im, data, lod_w * lod_h);
        });

    if (compression_enabled && compression_type == ECompressionType::MultiThreaded)
        // start asynchronous compression
        m_compressor.start_compressing();

    return true;
}
#endif // ENABLE_TEXTURED_VOLUMES

bool GLTexture::load_from_svg(const std::string& filename, bool use_mipmaps, bool compress, bool apply_anisotropy, unsigned int max_size_px)
{
    const bool compression_enabled = compress && GLEW_EXT_texture_compression_s3tc;

    NSVGimage* image = nsvgParseFromFile(filename.c_str(), "px", 96.0f);
    if (image == nullptr) {
        return false;
    }

    float scale = (float)max_size_px / std::max(image->width, image->height);

    m_width = (int)(scale * image->width);
    m_height = (int)(scale * image->height);

    if (compression_enabled)
        adjust_size_for_compression();

    const int n_pixels = m_width * m_height;

    if (n_pixels <= 0) {
        reset();
        nsvgDelete(image);
        return false;
    }

    NSVGrasterizer* rast = nsvgCreateRasterizer();
    if (rast == nullptr) {
        nsvgDelete(image);
        reset();
        return false;
    }

    // creates the temporary buffer only once, with max size, and reuse it for all the levels, if generating mipmaps
    std::vector<unsigned char> data(n_pixels * 4, 0);
    nsvgRasterize(rast, image, 0, 0, scale, data.data(), m_width, m_height, m_width * 4);

    send_to_gpu(data, use_mipmaps, compress ? ECompressionType::MultiThreaded : ECompressionType::None, apply_anisotropy, [&scale, rast, image](int lod_w, int lod_h, std::vector<unsigned char>& data) {
        scale *= 0.5f;
        data.resize(lod_w * lod_h * 4);
        nsvgRasterize(rast, image, 0, 0, scale, data.data(), lod_w, lod_h, lod_w * 4);
        });

    m_source = filename;

    if (compression_enabled)
        // start asynchronous compression
        m_compressor.start_compressing();

    nsvgDeleteRasterizer(rast);
    nsvgDelete(image);

    return true;
}

bool GLTexture::adjust_size_for_compression()
{
    bool ret = false;

    // the stb_dxt compression library seems to like only texture sizes which are a multiple of 4
    int width_rem = m_width % 4;
    int height_rem = m_height % 4;

    if (width_rem != 0) {
        m_width += (4 - width_rem);
        ret = true;
    }

    if (height_rem != 0) {
        m_height += (4 - height_rem);
        ret = true;
    }

    return ret;
}

void GLTexture::send_to_gpu(std::vector<unsigned char>& data, bool use_mipmaps, ECompressionType compression_type, bool apply_anisotropy,
    std::function<void(int, int, std::vector<unsigned char>&)> resampler)
{
    glsafe(::glPixelStorei(GL_UNPACK_ALIGNMENT, 1));
    glsafe(::glGenTextures(1, &m_id));
    glsafe(::glBindTexture(GL_TEXTURE_2D, m_id));

    if (apply_anisotropy) {
        GLfloat max_anisotropy = OpenGLManager::get_gl_info().get_max_anisotropy();
        if (max_anisotropy > 1.0f)
            glsafe(::glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, max_anisotropy));
    }

    bool compression_enabled = (compression_type != ECompressionType::None) && GLEW_EXT_texture_compression_s3tc;
    if (compression_enabled) {
        if (compression_type == ECompressionType::SingleThreaded)
            glsafe(::glTexImage2D(GL_TEXTURE_2D, 0, GL_COMPRESSED_RGBA_S3TC_DXT5_EXT, (GLsizei)m_width, (GLsizei)m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const void*)data.data()));
        else {
            // initializes the texture on GPU 
            glsafe(::glTexImage2D(GL_TEXTURE_2D, 0, GL_COMPRESSED_RGBA_S3TC_DXT5_EXT, (GLsizei)m_width, (GLsizei)m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0));
            // and send the uncompressed data to the compressor
            m_compressor.add_level((unsigned int)m_width, (unsigned int)m_height, data);
        }
    }
    else
        glsafe(::glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, (GLsizei)m_width, (GLsizei)m_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const void*)data.data()));

    if (use_mipmaps) {
        // we manually generate mipmaps because glGenerateMipmap() function is not reliable on all graphics cards
        int lod_w = m_width;
        int lod_h = m_height;
        GLint level = 0;
        while (lod_w > 1 || lod_h > 1) {
            ++level;

            lod_w = std::max(lod_w / 2, 1);
            lod_h = std::max(lod_h / 2, 1);

            resampler(lod_w, lod_h, data);

            if (compression_enabled) {
                if (compression_type == ECompressionType::SingleThreaded)
                    glsafe(::glTexImage2D(GL_TEXTURE_2D, level, GL_COMPRESSED_RGBA_S3TC_DXT5_EXT, (GLsizei)lod_w, (GLsizei)lod_h, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const void*)data.data()));
                else {
                    // initializes the texture on GPU 
                    glsafe(::glTexImage2D(GL_TEXTURE_2D, level, GL_COMPRESSED_RGBA_S3TC_DXT5_EXT, (GLsizei)lod_w, (GLsizei)lod_h, 0, GL_RGBA, GL_UNSIGNED_BYTE, 0));
                    // and send the uncompressed data to the compressor
                    m_compressor.add_level((unsigned int)lod_w, (unsigned int)lod_h, data);
                }
            }
            else
                glsafe(::glTexImage2D(GL_TEXTURE_2D, level, GL_RGBA, (GLsizei)lod_w, (GLsizei)lod_h, 0, GL_RGBA, GL_UNSIGNED_BYTE, (const void*)data.data()));
        }

        if (!compression_enabled) {
            glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, level));
            glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR));
        }
    }
    else {
        glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
        glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 0));
    }

    glsafe(::glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));

    glsafe(::glBindTexture(GL_TEXTURE_2D, 0));
}

#if ENABLE_TEXTURED_VOLUMES
bool GLIdeaMakerTexture::load_from_ideamaker_texture_file(const std::string& filename, bool use_mipmaps, ECompressionType compression_type, bool apply_anisotropy)
{
    reset();

    if (!boost::filesystem::exists(filename))
        return false;

    if (boost::algorithm::iends_with(filename, ".texture")) {

        boost::nowide::ifstream file(filename, boost::nowide::ifstream::binary);
        if (!file.good())
            return false;

        boost::property_tree::ptree root;
        boost::property_tree::read_json(file, root);

        file.close();

        // http://www.cochoy.fr/boost-property-tree/
        boost::optional<std::string> id = root.get_optional<std::string>("header.texture_id");
        boost::optional<std::string> name = root.get_optional<std::string>("header.texture_name");
        boost::optional<std::string> image_data = root.get_optional<std::string>("image_data");
        boost::optional<std::string> border_color = root.get_optional<std::string>("settings.texture_border_color");
        boost::optional<float> repeat_x = root.get_optional<float>("settings.texture_repeat_x");
        boost::optional<float> repeat_y = root.get_optional<float>("settings.texture_repeat_y");
        boost::optional<float> rotation_z = root.get_optional<float>("settings.texture_rotation_z");
        boost::optional<float> translation_x = root.get_optional<float>("settings.texture_translation_x");
        boost::optional<float> translation_y = root.get_optional<float>("settings.texture_translation_y");
        boost::optional<std::string> wrapping = root.get_optional<std::string>("settings.texture_wrapping");
        boost::optional<std::string> version = root.get_optional<std::string>("version");

        m_source = filename;

        if (id.has_value())
            m_imaker_id = id.value();
        if (border_color.has_value())
            m_border_color = border_color.value();
        if (repeat_x.has_value())
            m_repeat_x = repeat_x.value();
        if (repeat_y.has_value())
            m_repeat_y = repeat_y.value();
        if (rotation_z.has_value())
            m_rotation_z = rotation_z.value();
        if (translation_x.has_value())
            m_translation_x = translation_x.value();
        if (translation_y.has_value())
            m_translation_y = translation_y.value();
        if (wrapping.has_value()) {
            std::string value = wrapping.value();
            if (value == "repeat")
                m_wrapping = EWrapping::Repeat;
            else if (value == "mirror")
                m_wrapping = EWrapping::Repeat;
            else if (value == "clamptoedge")
                m_wrapping = EWrapping::ClampToEdge;
            else if (value == "clamptoborder")
                m_wrapping = EWrapping::ClampToBorder;
            else
                m_wrapping = EWrapping::Unknown;
        }
        if (version.has_value())
            m_version = version.value();

        if (image_data.has_value()) {
            const std::string src = image_data.value();
            std::string decoded;
            decoded.resize(boost::beast::detail::base64::decoded_size(src.length()));
            decoded.resize(boost::beast::detail::base64::decode((void*)&decoded[0], src.data(), src.length()).first);
            std::vector<unsigned char> src_data(decoded.length());
            ::memcpy((void*)src_data.data(), (const void*)decoded.data(), decoded.length());
            bool ret = load_from_png_memory(src_data, true, GLTexture::ECompressionType::SingleThreaded, true);
            if (!ret)
                reset();
            return ret;
        }
    }
    return false;
}
#endif // ENABLE_TEXTURED_VOLUMES

} // namespace GUI
} // namespace Slic3r
