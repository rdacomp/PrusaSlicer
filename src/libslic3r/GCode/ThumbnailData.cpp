#include "libslic3r/libslic3r.h"
#include "ThumbnailData.hpp"

#if !ENABLE_TEXTURED_VOLUMES

namespace Slic3r {

void ThumbnailData::set(unsigned int w, unsigned int h)
{
    if (w == 0 || h == 0)
        return;

    if (width != w || height != h) {
        width = w;
        height = h;
        // defaults to white texture
        data = std::vector<unsigned char>(width * height * 4, 255);
    }
}

void ThumbnailData::reset()
{
    width = 0;
    height = 0;
    data.clear();
}

bool ThumbnailData::is_valid() const
{
    return width != 0 && height != 0 && (unsigned int)data.size() == 4 * width * height;
}

} // namespace Slic3r

#endif // !ENABLE_TEXTURED_VOLUMES
