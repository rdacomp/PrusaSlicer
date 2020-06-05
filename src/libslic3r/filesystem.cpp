#include "filesystem.hpp"

#include <random>
#include <tbb/mutex.h>

namespace Slic3r {

class TempHashGenerator
{
public:
	std::pair<uint32_t, uint32_t> operator()() {
		tbb::mutex::scoped_lock lock(m_mutex);
		if (! m_initialized) {
			m_mt19937.seed(std::time(nullptr));
			m_initialized = true;
		}
		// Generate 64 bits of randomness.
		return std::make_pair(uint32_t(m_mt19937()), uint32_t(m_mt19937()));
	}

private:
	tbb::mutex	 m_mutex;
	bool 		 m_initialized = false;
    std::mt19937 m_mt19937;
};

static TempHashGenerator g_temp_hash_generator;

filesystem::path gen_temp_file_path(const std::string &prefix, const std::string &suffix)
{
	std::pair<uint32_t, uint32_t> hash = g_temp_hash_generator();
	char hash_str[20];
	sprintf(hash_str, "%04X-%04X-%04X-%04X",
        int(hash.first  >> 16), int(hash.first  & 0x0ffff),
        int(hash.second >> 16), int(hash.second & 0x0ffff));

	return filesystem::temp_directory_path() / filesystem::u8path(prefix + hash_str + suffix);
}

} // namespace Slic3r
