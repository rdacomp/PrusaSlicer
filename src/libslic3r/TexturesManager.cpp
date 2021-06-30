#include "libslic3r.h"
#include "TexturesManager.hpp"

#if ENABLE_TEXTURED_VOLUMES
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/beast/core/detail/base64.hpp>

#include <png.h>

namespace Slic3r {

std::string TexturesManager::add_texture_from_file(const std::string& filename)
{
    // check if the texture from the given filename is already in cache
    for (TextureItem& item : m_textures) {
        if (filename == item.source) {
            // increment reference count
            ++item.count;

#if ENABLE_TEXTURES_MANAGER_DEBUG
            output_content();
#endif // ENABLE_TEXTURES_MANAGER_DEBUG

            // return texture name
            return item.name;
        }
    }

    // load the texture and add it to the cache
    std::shared_ptr<TextureData> texture = std::make_shared<TextureData>();
    TextureItem item;
    item.texture = std::make_shared<TextureData>();
    item.texture->format = TextureData::EFormat::png;
    bool res = false;

    if (boost::algorithm::iends_with(filename, ".png"))
        res = load_texture_from_png_file(filename, item);
    else if (boost::algorithm::iends_with(filename, ".texture"))
        res = load_texture_from_ideamaker_file(filename, item);

    if (res) {
        item.count = 1;
        m_textures.emplace_back(item);

#if ENABLE_TEXTURES_MANAGER_DEBUG
        output_content();
#endif // ENABLE_TEXTURES_MANAGER_DEBUG

        // return texture name
        return m_textures.back().name;
    }

    return "";
}

static std::pair<unsigned int, unsigned int> extract_size_from_png_buffer(const std::vector<unsigned char>& png_data)
{
    std::pair<unsigned int, unsigned int> ret = { 0,0 };

    // for reference, see: http://pulsarengine.com/2009/01/reading-png-images-from-memory/
    static const size_t PngSignatureLength = 8;

    // check PNG signature
    if (png_data.size() < PngSignatureLength) {
        BOOST_LOG_TRIVIAL(error) << "Found invalid data while loading png from memory";
        return ret;
    }

    std::array<unsigned char, PngSignatureLength> png_signature;
    ::memcpy((void*)png_signature.data(), (const void*)png_data.data(), PngSignatureLength);

    if (!png_check_sig(png_signature.data(), PngSignatureLength)) {
        BOOST_LOG_TRIVIAL(error) << "Found invalid signature while loading png from memory";
        return ret;
    }

    // create PNG file read struct (memory is allocated by libpng)
    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (png_ptr == nullptr) {
        BOOST_LOG_TRIVIAL(error) << "Unable to create read struct while loading png from memory";
        return ret;
    }

    // create PNG image data info struct (memory is allocated by libpng)
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == nullptr) {
        // libpng must free file info struct memory before we bail
        png_destroy_read_struct(&png_ptr, nullptr, nullptr);
        BOOST_LOG_TRIVIAL(error) << "Unable to create info struct while loading png from memory";
        return ret;
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
        return ret;
    }

    ret.first = width;
    ret.second = height;

    return ret;
}

std::string TexturesManager::add_texture_from_png_buffer(const std::string& filename, const std::vector<unsigned char>& png_data)
{
    // check if the texture from the given filename is already in cache
    for (TextureItem& item : m_textures) {
        if (filename == item.source) {
            // increment reference count
            ++item.count;

#if ENABLE_TEXTURES_MANAGER_DEBUG
            output_content();
#endif // ENABLE_TEXTURES_MANAGER_DEBUG

            // return texture name
            return item.name;
        }
    }

    // extract the texture from the buffer and add it to the cache
    std::shared_ptr<TextureData> texture = std::make_shared<TextureData>();
    TextureItem item;
    item.texture = std::make_shared<TextureData>();
    item.texture->format = TextureData::EFormat::png;
    if (load_texture_from_png_buffer(filename, png_data, item)) {
        item.count = 1;
        m_textures.emplace_back(item);

#if ENABLE_TEXTURES_MANAGER_DEBUG
        output_content();
#endif // ENABLE_TEXTURES_MANAGER_DEBUG

        // return texture name
        return m_textures.back().name;
    }

    return "";
}

void TexturesManager::merge(const TexturesManager& other)
{
    // FIXME : check for duplicates
    m_textures.insert(m_textures.end(), other.m_textures.begin(), other.m_textures.end());

#if ENABLE_TEXTURES_MANAGER_DEBUG
    output_content();
#endif // ENABLE_TEXTURES_MANAGER_DEBUG
}

void TexturesManager::remove_texture(const std::string& name)
{
    for (size_t i = 0; i < m_textures.size(); ++i) {
        TextureItem& item = m_textures[i];
        if (name == item.name) {
            // decrement reference count
            --item.count;
            if (item.count == 0) {
                // remove texture from the cache if count == 0
                item.texture.reset();
                m_textures.erase(m_textures.begin() + i);
            }

#if ENABLE_TEXTURES_MANAGER_DEBUG
            output_content();
#endif // ENABLE_TEXTURES_MANAGER_DEBUG

            return;
        }
    }
}

void TexturesManager::remove_all_textures()
{
    m_textures.clear();
#if ENABLE_TEXTURES_MANAGER_DEBUG
    output_content();
#endif // ENABLE_TEXTURES_MANAGER_DEBUG
}

#if ENABLE_TEXTURES_MANAGER_DEBUG
void TexturesManager::output_content() const
{
    std::cout << "\nTexturesManager content\n";
    std::cout << "=======================\n";

    if (m_textures.empty())
        std::cout << "empty\n";
    else {
        for (const TextureItem& item : m_textures) {
            std::cout << item.name << " (" << item.count << ")\n";
        }
    }

    std::cout << "=======================\n\n";
}
#endif // ENABLE_TEXTURES_MANAGER_DEBUG

std::vector<std::string> TexturesManager::get_texture_names() const
{
    std::vector<std::string> ret;
    for (const TextureItem& item : m_textures) {
        ret.push_back(item.name);
    }
    return ret;
}

const TextureData& TexturesManager::get_texture_data(const std::string& name) const
{
    for (const TextureItem& item : m_textures) {
        if (name == item.name)
            return *item.texture;
    }
    return TextureData::DUMMY;
}

std::string TexturesManager::get_texture_source(const std::string& name) const
{
    for (const TextureItem& item : m_textures) {
        if (name == item.name)
            return item.source;
    }
    return "";
}

std::string TexturesManager::decode_name(const std::string& name)
{
    std::string::size_type pos = name.find(':');
    return (pos == name.npos) ? name : name.substr(0, pos);
}

std::string TexturesManager::encode_name(const std::string& name)
{
    unsigned int count = 0;
    for (TextureItem& item : m_textures) {
        if (item.name == name || boost::istarts_with(item.name, name + ":"))
            ++count;
    }

    return (count == 0) ? name : TexturesManager::decode_name(name) + ":" + std::to_string(count);
}

bool TexturesManager::load_texture_from_ideamaker_file(const std::string& filename, TextureItem& out)
{
    if (!boost::filesystem::exists(filename))
        return false;

    if (!boost::algorithm::iends_with(filename, ".texture"))
        return false;

    boost::nowide::ifstream file(filename, boost::nowide::ifstream::binary);
    if (!file.good())
        return false;

    boost::property_tree::ptree root;
    boost::property_tree::read_json(file, root);

    file.close();

    // http://www.cochoy.fr/boost-property-tree/
//    boost::optional<std::string> id = root.get_optional<std::string>("header.texture_id");
    boost::optional<std::string> name = root.get_optional<std::string>("header.texture_name");
    boost::optional<std::string> image_data = root.get_optional<std::string>("image_data");
//    boost::optional<std::string> border_color = root.get_optional<std::string>("settings.texture_border_color");
//    boost::optional<float> repeat_x = root.get_optional<float>("settings.texture_repeat_x");
//    boost::optional<float> repeat_y = root.get_optional<float>("settings.texture_repeat_y");
//    boost::optional<float> rotation_z = root.get_optional<float>("settings.texture_rotation_z");
//    boost::optional<float> translation_x = root.get_optional<float>("settings.texture_translation_x");
//    boost::optional<float> translation_y = root.get_optional<float>("settings.texture_translation_y");
//    boost::optional<std::string> wrapping = root.get_optional<std::string>("settings.texture_wrapping");
//    boost::optional<std::string> version = root.get_optional<std::string>("version");

    out.source = filename;
    out.name = encode_name(boost::filesystem::path(filename).stem().string());

    if (image_data.has_value()) {
        const std::string src = image_data.value();
        std::string decoded;
        decoded.resize(boost::beast::detail::base64::decoded_size(src.length()));
        decoded.resize(boost::beast::detail::base64::decode((void*)&decoded[0], src.data(), src.length()).first);
        out.texture->data = std::vector<unsigned char>(decoded.length());
        ::memcpy((void*)out.texture->data.data(), (const void*)decoded.data(), decoded.length());
        const auto& [width, height] = extract_size_from_png_buffer(out.texture->data);
        if (width > 0 && height > 0) {
            out.texture->width = width;
            out.texture->height = height;
            return out.texture->is_valid();
        }
    }

    return false;
}

bool TexturesManager::load_texture_from_png_file(const std::string& filename, TextureItem& out)
{
    if (!boost::filesystem::exists(filename))
        return false;

    if (!boost::algorithm::iends_with(filename, ".png"))
        return false;

    boost::nowide::ifstream file(filename, boost::nowide::ifstream::binary);
    if (!file.good())
        return false;

    file.seekg(0, std::ios::end);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    std::vector<unsigned char> buffer(size);
    file.read((char*)buffer.data(), size);
    if (!file.good())
        return false;

    file.close();

    return load_texture_from_png_buffer(filename, buffer, out);
}

bool TexturesManager::load_texture_from_png_buffer(const std::string& filename, const std::vector<unsigned char>& png_data, TextureItem& out)
{
    out.source = filename;
    out.name = encode_name(boost::filesystem::path(filename).stem().string());
    out.texture->data = png_data;
    const auto& [width, height] = extract_size_from_png_buffer(out.texture->data);
    if (width > 0 && height > 0) {
        out.texture->width = width;
        out.texture->height = height;
        return out.texture->is_valid();
    }

    return false;
}

} // namespace Slic3r

#endif // ENABLE_TEXTURED_VOLUMES