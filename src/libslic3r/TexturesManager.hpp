#ifndef slic3r_TexturesManager_hpp_
#define slic3r_TexturesManager_hpp_

#if ENABLE_TEXTURED_VOLUMES
#define ENABLE_TEXTURES_MANAGER_DEBUG 0

#include "TextureData.hpp"

#include <memory>

namespace Slic3r {

class TexturesManager
{
    struct TextureItem
    {
        // texture data
        std::shared_ptr<TextureData> texture;
        // reference count
        unsigned int count{ 0 };
        // texture name
        std::string name;
        // texture source
        std::string source;
    };

    std::vector<TextureItem> m_textures;

public:
    TexturesManager& operator = (const TexturesManager& other) {
        m_textures = other.m_textures;
        return *this;
    }

    TexturesManager& operator = (TexturesManager&& other) {
        m_textures = std::move(other.m_textures);
        return *this;
    }

    std::string add_texture_from_file(const std::string& filename);
    std::string add_texture_from_png_buffer(const std::string& filename, const std::vector<unsigned char>& png_data);
    void merge(const TexturesManager& other);
    void remove_texture(const std::string& name);
    void remove_all_textures();

    std::vector<std::string> get_texture_names() const;
    const TextureData& get_texture_data(const std::string& name) const;
    std::string get_texture_source(const std::string& name) const;

#if ENABLE_TEXTURES_MANAGER_DEBUG
    void output_content() const;
#endif // ENABLE_TEXTURES_MANAGER_DEBUG

    // remove the trailing ":id" from the given name string, if present
    static std::string decode_name(const std::string& name);

private:
    // add a trailing ":id" to the given name string, if already in use
    std::string encode_name(const std::string& name);

    bool load_texture_from_ideamaker_file(const std::string& filename, TextureItem& out);
    bool load_texture_from_png_file(const std::string& filename, TextureItem& out);
    bool load_texture_from_png_buffer(const std::string& filename, const std::vector<unsigned char>& png_data, TextureItem& out);
};

} // namespace Slic3r

#endif // ENABLE_TEXTURED_VOLUMES

#endif // slic3r_TexturesManager_hpp_