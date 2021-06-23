#ifndef slic3r_ThumbnailData_hpp_
#define slic3r_ThumbnailData_hpp_

#include <vector>
#include "libslic3r/Point.hpp"
#if ENABLE_TEXTURED_VOLUMES
#include "libslic3r/TextureData.hpp"
#endif // ENABLE_TEXTURED_VOLUMES

namespace Slic3r {

#if ENABLE_TEXTURED_VOLUMES
    using ThumbnailsList = std::vector<TextureData>;
#else
struct ThumbnailData
{
    unsigned int width;
    unsigned int height;
    std::vector<unsigned char> data;

    ThumbnailData() { reset(); }
    void set(unsigned int w, unsigned int h);
    void reset();

    bool is_valid() const;
};

using ThumbnailsList = std::vector<ThumbnailData>;
#endif // ENABLE_TEXTURED_VOLUMES

struct ThumbnailsParams
{
	const Vec2ds 	sizes;
	bool 			printable_only;
	bool 			parts_only;
	bool 			show_bed;
	bool 			transparent_background;
};

typedef std::function<ThumbnailsList(const ThumbnailsParams&)> ThumbnailsGeneratorCallback;

} // namespace Slic3r

#endif // slic3r_ThumbnailData_hpp_
