#include "RemovableDriveManager.hpp"

#include <boost/nowide/convert.hpp>
#include <boost/log/trivial.hpp>

#if _WIN32
#include <windows.h>
#include <tchar.h>
#include <winioctl.h>
#include <shlwapi.h>

#include <Dbt.h>

#else
// unix, linux & OSX includes
#include <errno.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <glob.h>
#include <pwd.h>
#include <boost/filesystem.hpp>
#include <boost/filesystem/convenience.hpp>
#endif

namespace Slic3r {
namespace GUI { 

wxDEFINE_EVENT(EVT_REMOVABLE_DRIVE_EJECTED, RemovableDriveEjectEvent);
wxDEFINE_EVENT(EVT_REMOVABLE_DRIVES_CHANGED, RemovableDrivesChangedEvent);

#if _WIN32
std::vector<DriveData> RemovableDriveManager::search_for_removable_drives() const
{
	//get logical drives flags by letter in alphabetical order
	DWORD drives_mask = ::GetLogicalDrives();

	// Allocate the buffers before the loop.
	std::wstring volume_name;
	volume_name.resize(MAX_PATH + 1);
	std::wstring file_system_name;
	file_system_name.resize(MAX_PATH + 1);

	// Iterate the Windows drives from 'A' to 'Z'
	std::vector<DriveData> current_drives;
	for (size_t i = 0; i < 26; ++ i)
		if (drives_mask & (1 << i)) {
			std::string path { char('A' + i), ':' };
			UINT drive_type = ::GetDriveTypeA(path.c_str());
			// DRIVE_REMOVABLE on W are sd cards and usb thumbnails (not usb harddrives)
			if (drive_type ==  DRIVE_REMOVABLE) {
				// get name of drive
				std::wstring wpath = boost::nowide::widen(path);
				BOOL error = ::GetVolumeInformationW(wpath.c_str(), volume_name.data(), sizeof(volume_name), nullptr, nullptr, nullptr, file_system_name.data(), sizeof(file_system_name));
				if (error != 0) {
					volume_name.erase(volume_name.begin() + wcslen(volume_name.c_str()), volume_name.end());
					if (! file_system_name.empty()) {
						ULARGE_INTEGER free_space;
						::GetDiskFreeSpaceExA(path.c_str(), &free_space, nullptr, nullptr);
						if (free_space.QuadPart > 0) {
							path += "\\";
							current_drives.push_back(DriveData{ boost::nowide::narrow(volume_name), path });
						}
					}
				}
			}
		}
	return current_drives;
}

// Called from UI therefore it blocks the UI thread.
// It also blocks updates at the worker thread.
// Win32 implementation.
void RemovableDriveManager::eject_drive(bool update_removable_drives_before)
{
	if (! m_last_save_path_verified || m_last_save_path.empty())
		return;

	if (update_removable_drives_before)
		this->update();

	tbb::mutex::scoped_lock lock(m_drives_mutex);
	for (const DriveData &drive_data : m_current_drives)
		if (drive_data.path == m_last_save_path) {
			// get handle to device
			std::string mpath = "\\\\.\\" + m_last_save_path;
			mpath = mpath.substr(0, mpath.size() - 1);
			HANDLE handle = CreateFileA(mpath.c_str(), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
			if (handle == INVALID_HANDLE_VALUE) {
				std::cerr << "Ejecting " << mpath << " failed " << GetLastError() << " \n";
				return;
			}
			DWORD deviceControlRetVal(0);
			//these 3 commands should eject device safely but they dont, the device does disappear from file explorer but the "device was safely remove" notification doesnt trigger.
			//sd cards does  trigger WM_DEVICECHANGE messege, usb drives dont
			DeviceIoControl(handle, FSCTL_LOCK_VOLUME, nullptr, 0, nullptr, 0, &deviceControlRetVal, nullptr);
			DeviceIoControl(handle, FSCTL_DISMOUNT_VOLUME, nullptr, 0, nullptr, 0, &deviceControlRetVal, nullptr);
			// some implemenatations also calls IOCTL_STORAGE_MEDIA_REMOVAL here but it returns error to me
			BOOL error = DeviceIoControl(handle, IOCTL_STORAGE_EJECT_MEDIA, nullptr, 0, nullptr, 0, &deviceControlRetVal, nullptr);
			if (error == 0) {
				CloseHandle(handle);
				BOOST_LOG_TRIVIAL(error) << "Ejecting " << mpath << " failed " << deviceControlRetVal << " " << GetLastError() << " \n";
				return;
			}
			CloseHandle(handle);
			m_drive_data_last_eject = drive_data;
			break;
		}
}

bool RemovableDriveManager::is_path_on_removable_drive(const std::string &path, bool update_removable_drives_before)
{
	if (update_removable_drives_before)
		this->update();

	tbb::mutex::scoped_lock lock(m_drives_mutex);
	if (m_current_drives.empty())
		return false;
	std::size_t found = path.find_last_of("\\");
	std::string new_path = path.substr(0, found);
	int letter = PathGetDriveNumberA(new_path.c_str());
	for (const DriveData &drive_data : m_current_drives) {
		char drive = drive_data.path[0];
		if (drive == 'A' + letter)
			return true;	
	}
	return false;
}

std::optional<DriveData> RemovableDriveManager::get_drive_from_path(const std::string& path)
{
	tbb::mutex::scoped_lock lock(m_drives_mutex);
	std::size_t found = path.find_last_of("\\");
	std::string new_path = path.substr(0, found);
	int letter = PathGetDriveNumberA(new_path.c_str());	
	for (const DriveData &drive_data : m_current_drives) {
		assert(! drive_data.path.empty());
		if (drive_data.path.front() == 'A' + letter)
			return std::optional<DriveData>(drive_data);
	}
	return std::optional<DriveData>(std::nullopt);
}

#if 0
// currently not used, left for possible future use
INT_PTR WINAPI WinProcCallback(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	// here we need to catch messeges about device removal
	// problem is that when ejecting usb (how is it implemented above) there is no messege dispached. Only after physical removal of the device.
	//uncomment register_window() in init() to register and comment update() in GUI_App.cpp (only for windows!) to stop recieving periodical updates 
	
	LRESULT lRet = 1;
	static HDEVNOTIFY hDeviceNotify;
	static constexpr GUID WceusbshGUID = { 0x25dbce51, 0x6c8f, 0x4a72, 0x8a,0x6d,0xb5,0x4c,0x2b,0x4f,0xc8,0x35 };

	switch (message)
	{
	case WM_CREATE:
		DEV_BROADCAST_DEVICEINTERFACE NotificationFilter;

		ZeroMemory(&NotificationFilter, sizeof(NotificationFilter));
		NotificationFilter.dbcc_size = sizeof(DEV_BROADCAST_DEVICEINTERFACE);
		NotificationFilter.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
		NotificationFilter.dbcc_classguid = WceusbshGUID;

		hDeviceNotify = RegisterDeviceNotification(hWnd, &NotificationFilter, DEVICE_NOTIFY_WINDOW_HANDLE);
		break;
	
	case WM_DEVICECHANGE:
	{
		// here is the important
		if(wParam == DBT_DEVICEREMOVECOMPLETE)
		{
			RemovableDriveManager::get_instance().update(0, true);
		}
	}
	break;
	
	default:
		// Send all other messages on to the default windows handler.
		lRet = DefWindowProc(hWnd, message, wParam, lParam);
		break;
	}
	return lRet;
	
}

void RemovableDriveManager::register_window()
{
	//creates new unvisible window that is recieving callbacks from system
	// structure to register 
	// currently not used, left for possible future use
	WNDCLASSEX wndClass;
	wndClass.cbSize = sizeof(WNDCLASSEX);
	wndClass.style = CS_OWNDC | CS_HREDRAW | CS_VREDRAW;
	wndClass.hInstance = reinterpret_cast<HINSTANCE>(GetModuleHandle(0));
	wndClass.lpfnWndProc = reinterpret_cast<WNDPROC>(WinProcCallback);//this is callback
	wndClass.cbClsExtra = 0;
	wndClass.cbWndExtra = 0;
	wndClass.hIcon = LoadIcon(0, IDI_APPLICATION);
	wndClass.hbrBackground = CreateSolidBrush(RGB(192, 192, 192));
	wndClass.hCursor = LoadCursor(0, IDC_ARROW);
	wndClass.lpszClassName = L"PrusaSlicer_aux_class";
	wndClass.lpszMenuName = NULL;
	wndClass.hIconSm = wndClass.hIcon;
	if(!RegisterClassEx(&wndClass))
	{
		DWORD err = GetLastError();
		return;
	}

	HWND hWnd = CreateWindowEx(
		WS_EX_NOACTIVATE,
		L"PrusaSlicer_aux_class",
		L"PrusaSlicer_aux_wnd",
		WS_DISABLED, // style
		CW_USEDEFAULT, 0,
		640, 480,
		NULL, NULL,
		GetModuleHandle(NULL),
		NULL);
	if(hWnd == NULL)
	{
		DWORD err = GetLastError();
	}
	//ShowWindow(hWnd, SW_SHOWNORMAL);
	UpdateWindow(hWnd);
}
#endif

#else

namespace search_for_drives_internal 
{
	static bool compare_filesystem_id(const std::string &path_a, const std::string &path_b)
	{
		struct stat buf;
		stat(path_a.c_str() ,&buf);
		dev_t id_a = buf.st_dev;
		stat(path_b.c_str() ,&buf);
		dev_t id_b = buf.st_dev;
		return id_a == id_b;
	}

	void inspect_file(const std::string &path, const std::string &parent_path, std::vector<DriveData> &out)
	{
		//confirms if the file is removable drive and adds it to vector

		//if not same file system - could be removable drive
		if (! compare_filesystem_id(path, parent_path)) {
			//free space
			boost::filesystem::space_info si = boost::filesystem::space(path);
			if (si.available != 0) {
				//user id
				struct stat buf;
				stat(path.c_str(), &buf);
				uid_t uid = buf.st_uid;
				std::string username(std::getenv("USER"));
				struct passwd *pw = getpwuid(uid);
				if (pw != 0 && pw->pw_name == username)
					out.emplace_back(DriveData{ boost::filesystem::basename(boost::filesystem::path(path)), path });
				}
			}
		}
	}

	static void search_path(const std::string &path, const std::string &parent_path, std::vector<DriveData> &out)
	{
	    glob_t globbuf;
		globbuf.gl_offs = 2;
		int error = glob(path.c_str(), GLOB_TILDE, NULL, &globbuf);
		if (error == 0) 
		{
			for (size_t i = 0; i < globbuf.gl_pathc; ++ i)
				inspect_file(globbuf.gl_pathv[i], parent_path, out);
		} else
		{
			//if error - path probably doesnt exists so function just exits
			//std::cout<<"glob error "<< error<< "\n";
		}
		
		globfree(&globbuf);
	}
}

std::vector<DriveData> RemovableDriveManager::search_for_removable_drives() const
{
	std::vector<DriveData> current_drives;

#if __APPLE__

	// if on macos obj-c class will enumerate
	if (m_rdmmm)
		m_rdmmm->list_devices(*this, current_drives);

#else

    //search /media/* folder
	search_for_drives_internal::search_path("/media/*", "/media", current_drives);

	//search_path("/Volumes/*", "/Volumes");
    std::string path(std::getenv("USER"));
	std::string pp(path);

	//search /media/USERNAME/* folder
	pp = "/media/"+pp;
	path = "/media/" + path + "/*";
	search_for_drives_internal::search_path(path, pp, current_drives);

	//search /run/media/USERNAME/* folder
	path = "/run" + path;
	pp = "/run"+pp;
	search_for_drives_internal::search_path(path, pp, current_drives);
#endif

	return current_drives;
}

// Called from UI therefore it blocks the UI thread.
// It also blocks updates at the worker thread.
// Unix & OSX implementation.
void RemovableDriveManager::eject_drive(bool update_removable_drives_before)
{
	if (! m_last_save_path_verified || m_last_save_path.empty())
		return;

	if (update_removable_drives_before)
		this->update();

	tbb::mutex::scoped_lock lock(m_drives_mutex);
	for (const DriveData &drive_data : m_current_drives)
		if (drive_data.path == m_last_save_path) {            
			std::string correct_path(m_last_save_path);
			for (size_t i = 0; i < correct_path.size(); ++i)
            	if (correct_path[i]==' ') {
					correct_path = correct_path.insert(i,1,'\\');
            		++ i;
            	}
			//std::cout<<"Ejecting "<<(*it).name<<" from "<< correct_path<<"\n";
			// there is no usable command in c++ so terminal command is used instead
			// but neither triggers "succesful safe removal messege"
			std::string command = 
#if __APPLE__
			//m_rdmmm->eject_device(m_last_save_path);
				"diskutil unmount ";
#else
				"umount ";
#endif
			command += correct_path;
			int err = system(command.c_str());
			if (err) {
				BOOST_LOG_TRIVIAL(error) << "Ejecting " << m_last_save_path << " failed";
				return;
			}

			m_drives_at_eject = m_current_drives;
			break;
		}
}

bool RemovableDriveManager::is_path_on_removable_drive(const std::string &path, bool update_removable_drives_before)
{
	if (update_removable_drives_before)
		this->update();

	std::size_t found = path.find_last_of("/");
	std::string new_path = found == path.size() - 1 ? path.substr(0, found) : path;

	tbb::mutex::scoped_lock lock(m_drives_mutex);
	for (const DriveData &data : m_current_drives)
		if (search_for_drives_internal::compare_filesystem_id(new_path, data.path))
			return true;
	return false;
}

std::optional<DriveData> RemovableDriveManager::get_drive_from_path(const std::string& path, bool update_removable_drives_before)
{
	if (update_removable_drives_before)
		this->update();

	std::size_t found = path.find_last_of("/");
	std::string new_path = found == path.size() - 1 ? path.substr(0, found) : path;
    // trim the filename
    found = new_path.find_last_of("/");
    new_path = new_path.substr(0, found);
    
	// check if same filesystem
	tbb::mutex::scoped_lock lock(m_drives_mutex);
	for (const DriveData &data : m_current_drives)
		if (search_for_drives_internal::compare_filesystem_id(new_path, data.path))
			return std::optional<DriveData>(drive_data);
	return std::optional<DriveData>(std::nullopt);
}
#endif

void RemovableDriveManager::init(wxEvtHandler *callback_evt_handler)
{
	assert(! m_initialized);
	assert(m_callback_evt_handler == nullptr);

	if (m_initialized)
		return;

	m_initialized = true;
	m_callback_evt_handler = callback_evt_handler;
#if _WIN32
	//register_window();
#elif __APPLE__
    m_rdmmm->register_window();
#endif
	m_thread = boost::thread((boost::bind(&RemovableDriveManager::thread_proc, this)));
	//FIXME do we want to send out events before the Plater is constructed?
//	this->update();
}

void RemovableDriveManager::shutdown()
{
    if (m_thread.joinable()) {
    	// Stop the worker thread, if running.
		{
			// Notify the worker thread to cancel wait on detection polling.
			std::lock_guard<std::mutex> lck(m_thread_stop_mutex);
			m_stop = true;
		}
		m_thread_stop_condition.notify_all();
		// Wait for the worker thread to stop.
		m_thread.join();
		m_stop = false;
	}

	m_initialized = false;
}

// Update is called from thread_proc() and from most of the public methods on demand.
void RemovableDriveManager::update()
{
	std::vector<DriveData> current_drives = this->search_for_removable_drives();

	// Post update events.
	tbb::mutex::scoped_lock lock(m_drives_mutex);
	std::sort(current_drives.begin(), current_drives.end());
	if (current_drives != m_current_drives) {
		if (! m_drive_data_last_eject.empty() && std::find(current_drives.begin(), current_drives.end(), m_drive_data_last_eject) == current_drives.end()) {
			assert(m_callback_evt_handler);
			if (m_callback_evt_handler) 
				wxPostEvent(m_callback_evt_handler, RemovableDriveEjectEvent(EVT_REMOVABLE_DRIVE_EJECTED, std::move(m_drive_data_last_eject)));
			m_drive_data_last_eject.clear();
		} else {
			assert(m_callback_evt_handler);
			if (m_callback_evt_handler)
				wxPostEvent(m_callback_evt_handler, RemovableDrivesChangedEvent(EVT_REMOVABLE_DRIVES_CHANGED));
		}
	}
	m_current_drives = std::move(current_drives);
}

void RemovableDriveManager::thread_proc()
{
	for (;;) {
		// Wait for 2 seconds before running the disk enumeration.
		// Cancellable.
		{
			std::unique_lock<std::mutex> lck(m_thread_stop_mutex);
			m_thread_stop_condition.wait_for(lck, std::chrono::seconds(2), [this]{ return m_stop; });
		}
		if (m_stop)
			// Stop the worker thread.
			break;
		// Update m_current drives and send out update events.
		this->update();
	}
}

std::string RemovableDriveManager::get_drive_path(bool update_removable_drives_before) 
{
	if (update_removable_drives_before)
		this->update();
	tbb::mutex::scoped_lock lock(m_drives_mutex);
	if (m_current_drives.empty()) {
		this->reset_last_save_path();
		return std::string();
	}
	return m_last_save_path_verified ? m_last_save_path : m_current_drives.front().path;
}

void RemovableDriveManager::set_and_verify_last_save_path(const std::string &path, bool update_removable_drives_before)
{
	if (update_removable_drives_before)
		this->update();

	std::optional<DriveData> last_drive;
	if (m_last_save_path_verified) { 
		// If old path is on drive
		last_drive = get_drive_from_path(path);
		if (last_drive) {
			// and new is too, rewrite the path
			m_last_save_path_verified = false;
			m_last_save_path = path;
		} else
			// else do nothing
			return;
	} else {
		last_drive = get_drive_from_path(path);
		m_last_save_path = path;
	}

	if (last_drive) {
		m_last_save_path_verified = true;
		m_last_save_path = last_drive->path;
		m_last_save_name = last_drive->name;
	} else
		this->reset_last_save_path();
}

bool RemovableDriveManager::is_last_drive_removed(bool update_removable_drives_before) 
{
	if (! m_last_save_path_verified)
		return true;

	if (update_removable_drives_before)
		this->update();

	bool is_drive_mounted = false;
	{
		tbb::mutex::scoped_lock lock(m_drives_mutex);
		is_drive_mounted = std::find_if(m_current_drives.begin(), m_current_drives.end(), [this](const DriveData &data){ return data.path == m_last_save_path; }) != m_current_drives.end();
	}

	if (! is_drive_mounted) 
		this->reset_last_save_path();

	return ! is_drive_mounted;
}

}}//namespace Slicer::Gui
