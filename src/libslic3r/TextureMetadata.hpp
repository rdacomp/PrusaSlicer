#ifndef slic3r_TextureMetadata_hpp_
#define slic3r_TextureMetadata_hpp_

#if ENABLE_TEXTURED_VOLUMES

namespace Slic3r {

struct TextureMetadata
{
    enum class EMapping
    {
        Cubic,
        Cylindrical,
        Spherical
    };

    enum class EWrapping
    {
        Repeat,
        Mirror,
        ClampToEdge,
        ClampToBorder
    };

    // Type of mapping
    EMapping mapping{ EMapping::Cubic };
    // Type of wrapping
    EWrapping wrapping{ EWrapping::Repeat };

    // Offset in u direction, [0..1] in percent
    float offset_u{ 0.0f };
    // Offset in v direction, [0..1] in percent
    float offset_v{ 0.0f };

    // Repeat factor in u direction
    float repeat_u{ 1.0f };
    // Repeat factor in v direction
    float repeat_v{ 1.0f };

    // rotation, [0..360] in degrees
    float rotation{ 0.0f };

    // Texture name
    std::string name;

    bool operator == (const TextureMetadata& rhs) const {
        if (mapping != rhs.mapping)
            return false;
        if (wrapping != rhs.wrapping)
            return false;
        if (offset_u != rhs.offset_u)
            return false;
        if (offset_v != rhs.offset_v)
            return false;
        if (repeat_u != rhs.repeat_u)
            return false;
        if (repeat_v != rhs.repeat_v)
            return false;
        if (rotation != rhs.rotation)
            return false;
        if (name != rhs.name)
            return false;

        return true;
    }

    bool operator != (const TextureMetadata& rhs) const {
        return !operator==(rhs);
    }

    void reset() {
        mapping = EMapping::Cubic;
        wrapping = EWrapping::Repeat;
        offset_u = 0.0f;
        offset_v = 0.0f;
        repeat_u = 1.0f;
        repeat_v = 1.0f;
        rotation = 0.0f;
        name.clear();
    }

    template<class Archive> void serialize(Archive& ar) {
        ar(mapping, wrapping, offset_u, offset_v, repeat_u, repeat_v, rotation, name);
    }

    static const TextureMetadata DUMMY;
};

} // namespace Slic3r

#endif // ENABLE_TEXTURED_VOLUMES

#endif // slic3r_TextureMetadata_hpp_
