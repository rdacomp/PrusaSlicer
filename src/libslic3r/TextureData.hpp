#ifndef slic3r_TextureData_hpp_
#define slic3r_TextureData_hpp_

#if ENABLE_TEXTURED_VOLUMES

namespace Slic3r {

struct TextureData
{
    // type of data layout
    enum class EFormat : unsigned char
    {
        raw_rgba, // [r][g][b][a]...
        png       // png buffer
    };

    EFormat format{ EFormat::raw_rgba };
    unsigned int width{ 0 };
    unsigned int height{ 0 };
    std::vector<unsigned char> data;

    TextureData() { reset(); }
    // initialize data to a white texture of given size (for raw_rgba only)
    void set(unsigned int w, unsigned int h);
    void reset();

    bool is_valid() const {
        return width != 0 && height != 0 && 
        ((format == EFormat::png && !data.empty()) ||
         (format == EFormat::raw_rgba && (unsigned int)data.size() == 4 * width * height));
    }

    static const TextureData DUMMY;
};

} // namespace Slic3r

#endif // ENABLE_TEXTURED_VOLUMES

#endif // slic3r_TextureData_hpp_
