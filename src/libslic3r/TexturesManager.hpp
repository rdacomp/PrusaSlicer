#ifndef slic3r_TexturesManager_hpp_
#define slic3r_TexturesManager_hpp_

#if ENABLE_TEXTURED_VOLUMES
#define ENABLE_TEXTURES_MANAGER_DEBUG 1

#include "TextureData.hpp"

namespace Slic3r {

class TexturesManager
{
    struct TextureItem
    {
        // texture data
        std::shared_ptr<TextureData> texture;
        // reference count
        unsigned int count{ 0 };
        // id of the texture on gpu
        unsigned int id{ 0 };
        // texture name
        std::string name;
        // texture source
        std::string source;
    };

    std::vector<TextureItem> m_textures;

public:
    std::string add_texture(const std::string& filename);
    void remove_texture(const std::string& name);
    void remove_all_textures();

    // return the gpu id of the texture
    unsigned int get_texture_id(const std::string& name) const;
//    const TextureMetadata& get_texture_metadata(const std::string& name) const;
    std::vector<std::string> get_texture_names() const;
    const TextureData& get_texture_data(const std::string& name) const;

#if ENABLE_TEXTURES_MANAGER_DEBUG
    void output_content() const;
#endif // ENABLE_TEXTURES_MANAGER_DEBUG

    // remove the trailing ":id" from the given name string, if present
    static std::string decode_name(const std::string& name);

private:
    // add a trailing ":id" to the given name string, if already in use
    std::string encode_name(const std::string& name);

    bool load_from_ideamaker_texture_file(const std::string& filename, TextureItem& out);
};

} // namespace Slic3r

#endif // ENABLE_TEXTURED_VOLUMES

#endif // slic3r_TexturesManager_hpp_