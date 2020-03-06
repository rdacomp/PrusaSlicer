#ifndef slic3r_GUI_RemovableDriveManager_hpp_
#define slic3r_GUI_RemovableDriveManager_hpp_

#include <vector>
#include <string>
#include <functional>
#include <optional>

#include <boost/thread.hpp>
#include <tbb/mutex.h>
#include <condition_variable>

// Custom wxWidget events
#include "Event.hpp"

namespace Slic3r {
namespace GUI {

struct DriveData
{
	std::string name;
	std::string path;

	void clear() {
		name.clear();
		path.clear();
	}
	bool empty() const {
		return path.empty();
	}
};

inline bool operator< (const DriveData &lhs, const DriveData &rhs) { return lhs.path < rhs.path; }
inline bool operator> (const DriveData &lhs, const DriveData &rhs) { return lhs.path > rhs.path; }
inline bool operator==(const DriveData &lhs, const DriveData &rhs) { return lhs.path == rhs.path; }

using RemovableDriveEjectEvent = Event<DriveData>;
wxDECLARE_EVENT(EVT_REMOVABLE_DRIVE_EJECTED, RemovableDriveEjectEvent);

using RemovableDrivesChangedEvent = SimpleEvent;
wxDECLARE_EVENT(EVT_REMOVABLE_DRIVES_CHANGED, RemovableDrivesChangedEvent);

class RemovableDriveManager
{
public:
	RemovableDriveManager() = default;
	RemovableDriveManager(RemovableDriveManager const&) = delete;
	void operator=(RemovableDriveManager const&) = delete;
	~RemovableDriveManager() { assert(! m_initialized); }

	//call only once. on apple register for unmnount callbacks. Enumerates devices for first time so init shoud be called on linux too.
	void 		init(wxEvtHandler *callback_evt_handler);
	void 		shutdown();

	// Eject drive of a file set by set_and_verify_last_save_path().
	void 		eject_drive(bool update_removable_drives_before);

	// returns path to last drive which was used, if none was used, returns empty string
	std::string get_drive_path(bool update_removable_drives_before);
	bool 		is_path_on_removable_drive(const std::string &path, bool update_removable_drives_before);

	// marks one of the eveices in vector as last used
	void 		set_and_verify_last_save_path(const std::string &path, bool update_removable_drives_before);
	bool 		is_last_drive_removed(bool update_removable_drives_before);
	size_t 		get_drives_count() { tbb::mutex::scoped_lock lock(m_drives_mutex); return m_current_drives.size(); }

private:
	// Worker thread, worker thread synchronization and callbacks to the UI thread.
	void 					thread_proc();
	boost::thread 			m_thread;
	std::condition_variable m_thread_stop_condition;
	mutable std::mutex 		m_thread_stop_mutex;
	wxEvtHandler*			m_callback_evt_handler { nullptr };
	bool 			 		m_initialized { false };
	bool 					m_stop { false };

	// Enumerates current drives and sends out wxWidget events on change or eject.
	void 					update();
	// Called from update() to enumerate removable drives.
	std::vector<DriveData> 	search_for_removable_drives() const;

	// m_current_drives is guarded by m_drives_mutex
	// sorted ascending by path
	std::vector<DriveData> 	m_current_drives;
	// When user requested an eject, the drive to be forcefuly ejected is stored here, so the next update will
	// recognize that the eject was finished with success and an eject event is sent out.
	// guarded with m_drives_mutex
	DriveData 				m_drive_data_last_eject;
	mutable tbb::mutex 		m_drives_mutex;

	// State machine for last save name / path.	
	// Returns drive path (same as path in DriveData) if exists otherwise empty string.
	// Called by set_last_save_path(), verify_last_save_path()
	std::optional<DriveData> get_drive_from_path(const std::string& path);
	void 					reset_last_save_path() {
		m_last_save_path_verified = false;
		m_last_save_path.clear();
		m_last_save_name.clear();
	}
	std::string 			m_last_save_path;
	std::string 			m_last_save_name;
	bool 					m_last_save_path_verified { false };

#if _WIN32
	//registers for notifications by creating invisible window
	//void register_window();
#else
    void register_window_osx();
    void unregister_window_osx();
    void list_devices(RemovableDriveManager& parent, std::vector<DriveData> &out) const;
    // not used as of now
    void eject_device(const std::string &path);
    // Opaque pointer to RemovableDriveManagerMM
    void *m_impl_osx;
#endif
};

}}

#endif // slic3r_GUI_RemovableDriveManager_hpp_
