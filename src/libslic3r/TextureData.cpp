#include "libslic3r.h"
#include "TextureData.hpp"

#if ENABLE_TEXTURED_VOLUMES

namespace Slic3r {

const TextureData TextureData::DUMMY = TextureData();

void TextureData::set(unsigned int w, unsigned int h)
{
    if (w == 0 || h == 0)
        return;

    if (width != w || height != h) {
        width = w;
        height = h;
        // defaults to white texture
        data = std::vector<unsigned char>(4 * width * height, 255);
        format = EFormat::raw_rgba;
    }
}

void TextureData::reset()
{
    format = EFormat::raw_rgba;
    width = 0;
    height = 0;
    data.clear();
}

} // namespace Slic3r

#endif // ENABLE_TEXTURED_VOLUMES
