#include "RemovableDriveManager.hpp"
#include <iostream>
#include "boost/nowide/convert.hpp"

#if _WIN32
#include <windows.h>
#include <tchar.h>
#include <winioctl.h>
#include <shlwapi.h>

#include <Dbt.h>
GUID WceusbshGUID = { 0x25dbce51, 0x6c8f, 0x4a72,
					  0x8a,0x6d,0xb5,0x4c,0x2b,0x4f,0xc8,0x35 };

#else
//linux includes
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

#if _WIN32
/* currently not used, left for possible future use
INT_PTR WINAPI WinProcCallback(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
*/
void RemovableDriveManager::search_for_drives()
{
	m_drives_mutex.lock();
	m_current_drives.clear();
	m_drives_mutex.unlock();
	//get logical drives flags by letter in alphabetical order
	DWORD drives_mask = GetLogicalDrives();
	for (size_t i = 0; i < 26; i++)
	{
		if(drives_mask & (1 << i))
		{
			std::string path (1,(char)('A' + i));
			path+=":";
			UINT drive_type = GetDriveTypeA(path.c_str());
			// DRIVE_REMOVABLE on W are sd cards and usb thumbnails (not usb harddrives)
			if (drive_type ==  DRIVE_REMOVABLE)
			{
				// get name of drive
				std::wstring wpath = boost::nowide::widen(path);
				std::wstring volume_name;
				volume_name.resize(1024);
				std::wstring file_system_name;
				file_system_name.resize(1024);
				LPWSTR  lp_volume_name_buffer = new wchar_t;
				BOOL error = GetVolumeInformationW(wpath.c_str(), &volume_name[0], sizeof(volume_name), NULL, NULL, NULL, &file_system_name[0], sizeof(file_system_name));
				if(error != 0)
				{
					volume_name.erase(std::find(volume_name.begin(), volume_name.end(), '\0'), volume_name.end());
					if (file_system_name != L"")
					{
						ULARGE_INTEGER free_space;
						GetDiskFreeSpaceExA(path.c_str(), &free_space, NULL, NULL);
						if (free_space.QuadPart > 0)
						{
							path += "\\";
							m_drives_mutex.lock();
							m_current_drives.push_back(DriveData(boost::nowide::narrow(volume_name), path));
							m_drives_mutex.unlock();
						}
					}
				}
			}
		}
	}
	
}
void RemovableDriveManager::eject_drive(const std::string &path)
{
	m_drives_mutex.lock();
	bool drives_empty = m_current_drives.empty();
	m_drives_mutex.unlock();
	if(drives_empty)
		return;
	m_drives_mutex.lock();
	for (auto it = m_current_drives.begin(); it != m_current_drives.end(); ++it)
	{
		if ((*it).path == path)
		{
			// get handle to device
			std::string mpath = "\\\\.\\" + path;
			mpath = mpath.substr(0, mpath.size() - 1);
			HANDLE handle = CreateFileA(mpath.c_str(), GENERIC_READ | GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
			if (handle == INVALID_HANDLE_VALUE)
			{
				std::cerr << "Ejecting " << mpath << " failed " << GetLastError() << " \n";
				m_drives_mutex.unlock();
				return;
			}
			DWORD deviceControlRetVal(0);
			//these 3 commands should eject device safely but they dont, the device does disappear from file explorer but the "device was safely remove" notification doesnt trigger.
			//sd cards does  trigger WM_DEVICECHANGE messege, usb drives dont
			
			DeviceIoControl(handle, FSCTL_LOCK_VOLUME, nullptr, 0, nullptr, 0, &deviceControlRetVal, nullptr);
			DeviceIoControl(handle, FSCTL_DISMOUNT_VOLUME, nullptr, 0, nullptr, 0, &deviceControlRetVal, nullptr);
			// some implemenatations also calls IOCTL_STORAGE_MEDIA_REMOVAL here but it returns error to me
			BOOL error = DeviceIoControl(handle, IOCTL_STORAGE_EJECT_MEDIA, nullptr, 0, nullptr, 0, &deviceControlRetVal, nullptr);
			if (error == 0)
			{
				CloseHandle(handle);
				std::cerr << "Ejecting " << mpath << " failed " << deviceControlRetVal << " " << GetLastError() << " \n";
				return;
			}
			CloseHandle(handle);
			m_did_eject = true;
			m_current_drives.erase(it);
			m_ejected_path = m_last_save_path;
			m_ejected_name = m_last_save_name;
			break;
		}
	}
	m_drives_mutex.unlock();
}
bool RemovableDriveManager::is_path_on_removable_drive(const std::string &path)
{
	m_drives_mutex.lock();
	bool drives_empty = m_current_drives.empty();
	m_drives_mutex.unlock();
	if (drives_empty)
		return false;
	std::size_t found = path.find_last_of("\\");
	std::string new_path = path.substr(0, found);
	int letter = PathGetDriveNumberA(new_path.c_str());
	m_drives_mutex.lock();
	for (auto it = m_current_drives.begin(); it != m_current_drives.end(); ++it)
	{
		char drive = (*it).path[0];
		if (drive == ('A' + letter)){
			m_drives_mutex.unlock();
			return true;
		}
	}
	m_drives_mutex.unlock();
	return false;
}
std::string RemovableDriveManager::get_drive_from_path(const std::string& path)
{
	std::size_t found = path.find_last_of("\\");
	std::string new_path = path.substr(0, found);
	int letter = PathGetDriveNumberA(new_path.c_str());
	m_drives_mutex.lock();
	for (auto it = m_current_drives.begin(); it != m_current_drives.end(); ++it)
	{
		char drive = (*it).path[0];
		if (drive == ('A' + letter))
		{
			m_drives_mutex.unlock();
			return (*it).path;
		}
	}
	m_drives_mutex.unlock();
	return "";
}
void RemovableDriveManager::register_window()
{
	//creates new unvisible window that is recieving callbacks from system
	// structure to register 
	/* currently not used, left for possible future use
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
	*/
}
/* currently not used, left for possible future use
INT_PTR WINAPI WinProcCallback(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	// here we need to catch messeges about device removal
	// problem is that when ejecting usb (how is it implemented above) there is no messege dispached. Only after physical removal of the device.
	//uncomment register_window() in init() to register and comment update() in GUI_App.cpp (only for windows!) to stop recieving periodical updates 
	
	LRESULT lRet = 1;
	static HDEVNOTIFY hDeviceNotify;

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
-			RemovableDriveManager::get_instance().update(0, true);
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
*/
#else
void RemovableDriveManager::search_for_drives()
{
	m_drives_mutex.lock();
    m_current_drives.clear();
	m_drives_mutex.unlock();
#if __APPLE__
	// if on macos obj-c class will enumerate

	if(m_rdmmm)
	{
		m_rdmmm->list_devices(*this);
	}
#else


    //search /media/* folder
	search_path("/media/*", "/media");

	//search_path("/Volumes/*", "/Volumes");
    std::string path(std::getenv("USER"));
	std::string pp(path);

	{
		//search /media/USERNAME/* folder
		pp = "/media/"+pp;
		path = "/media/" + path + "/*";
		search_path(path, pp);

		//search /run/media/USERNAME/* folder
		path = "/run" + path;
		pp = "/run"+pp;
		search_path(path, pp);

	}
#endif
	
}
void RemovableDriveManager::search_path(const std::string &path,const std::string &parent_path)
{
    glob_t globbuf;
	globbuf.gl_offs = 2;
	int error = glob(path.c_str(), GLOB_TILDE, NULL, &globbuf);
	if(error == 0) 
	{
		for(size_t i = 0; i < globbuf.gl_pathc; i++)
		{
			inspect_file(globbuf.gl_pathv[i], parent_path);
		}
	}else
	{
		//if error - path probably doesnt exists so function just exits
		//std::cout<<"glob error "<< error<< "\n";
	}
	
	globfree(&globbuf);
}
void RemovableDriveManager::inspect_file(const std::string &path, const std::string &parent_path)
{
	//confirms if the file is removable drive and adds it to vector

	//if not same file system - could be removable drive
	if(!compare_filesystem_id(path, parent_path))
	{
		//free space
		boost::filesystem::space_info si = boost::filesystem::space(path);
		if(si.available != 0)
		{
			//user id
			struct stat buf;
			stat(path.c_str(), &buf);
			uid_t uid = buf.st_uid;
			std::string username(std::getenv("USER"));
			struct passwd *pw = getpwuid(uid);
			if (pw != 0 && pw->pw_name == username)
			{				
				m_drives_mutex.lock();				
	       		m_current_drives.push_back(DriveData(boost::filesystem::basename(boost::filesystem::path(path)), path));
				m_drives_mutex.unlock();
			}
		}
	}
}
bool RemovableDriveManager::compare_filesystem_id(const std::string &path_a, const std::string &path_b)
{
	struct stat buf;
	stat(path_a.c_str() ,&buf);
	dev_t id_a = buf.st_dev;
	stat(path_b.c_str() ,&buf);
	dev_t id_b = buf.st_dev;
	return id_a == id_b;
}
void RemovableDriveManager::eject_drive(const std::string &path)
{
	m_drives_mutex.lock();
	bool drives_empty = m_current_drives.empty();
	m_drives_mutex.unlock();
	if (drives_empty)
		return;
	m_drives_mutex.lock();
	for (auto it = m_current_drives.begin(); it != m_current_drives.end(); ++it)
	{
		if((*it).path == path)
		{
            
            std::string correct_path(path);
            for (size_t i = 0; i < correct_path.size(); ++i)
            {
            	if(correct_path[i]==' ')
            	{
            		correct_path = correct_path.insert(i,1,'\\');
            		i++;
            	}
            }
            //std::cout<<"Ejecting "<<(*it).name<<" from "<< correct_path<<"\n";
// there is no usable command in c++ so terminal command is used instead
// but neither triggers "succesful safe removal messege"
            std::string command = "";
#if __APPLE__
            //m_rdmmm->eject_device(path);
            command = "diskutil unmount ";
#else
            command = "umount ";
#endif
            command += correct_path;
            int err = system(command.c_str());
            if(err)
            {
                std::cerr<<"Ejecting failed\n";
				m_drives_mutex.unlock();
                return;
            }

			m_did_eject = true;
            m_current_drives.erase(it);
			m_ejected_path = m_last_save_path;
			m_ejected_name = m_last_save_name;
            break;
		}

	}
	m_drives_mutex.unlock();
}
bool RemovableDriveManager::is_path_on_removable_drive(const std::string &path)
{
	m_drives_mutex.lock();
	bool drives_empty = m_current_drives.empty();
	m_drives_mutex.unlock();
	if (drives_empty)
		return false;
	std::size_t found = path.find_last_of("/");
	std::string new_path = found == path.size() - 1 ? path.substr(0, found) : path;
	m_drives_mutex.lock();
	for (auto it = m_current_drives.begin(); it != m_current_drives.end(); ++it)
	{
		if(compare_filesystem_id(new_path, (*it).path))
		{
			m_drives_mutex.unlock();
			return true;
		}
	}
	m_drives_mutex.unlock();
	return false;
}
std::string RemovableDriveManager::get_drive_from_path(const std::string& path) 
{
	std::size_t found = path.find_last_of("/");
	std::string new_path = found == path.size() - 1 ? path.substr(0, found) : path;
    
    // trim the filename
    found = new_path.find_last_of("/");
    new_path = new_path.substr(0, found);
    
	//check if same filesystem
	m_drives_mutex.lock();
	for (auto it = m_current_drives.begin(); it != m_current_drives.end(); ++it)
	{
		if (compare_filesystem_id(new_path, (*it).path))
		{
			m_drives_mutex.unlock();
			return (*it).path;
		}	
	}
	m_drives_mutex.unlock();
	return "";
}
#endif

RemovableDriveManager::RemovableDriveManager():
    m_initialized(false),
	m_drives_count(0),
    m_last_update(0),
    m_last_save_path(""),
	m_last_save_name(""),
	m_last_save_path_verified(false),
	m_is_writing(false),
	m_did_eject(false),
	m_plater_ready_to_slice(true),
	m_ejected_path(""),
	m_ejected_name(""),
	m_thread_enumerate_start(false),
	m_thread_enumerate_finnished(false),
	m_thread_finnished(false)
#if __APPLE__
	, m_rdmmm(std::make_unique<RDMMMWrapper>())
#endif
{
}
RemovableDriveManager::~RemovableDriveManager()
{
	if (m_initialized)
	{
		m_initialized = false;
		{
			std::lock_guard<std::mutex> lck(m_enumerate_mutex);
			m_thread_enumerate_start = true;
		}
		m_thread_enumerate_cv.notify_one();
		{
			std::unique_lock<std::mutex> lck(m_enumerate_mutex);
			m_thread_enumerate_cv.wait(lck, [this] {return m_thread_finnished; });
		}
		m_thread.join();
	}
}
void RemovableDriveManager::init()
{
	if (m_initialized)
		return;
	m_initialized = true;
	//add_callback([](void) { RemovableDriveManager::get_instance().print(); });
#if _WIN32
	//register_window();
#elif __APPLE__
    m_rdmmm->register_window();
#endif
	m_thread = boost::thread((boost::bind(&RemovableDriveManager::thread_proc, this)));
	//update(0, true);
}
void RemovableDriveManager::update(const long time)
{
	if(time != 0) //time = 0 is forced update
	{
		long diff = m_last_update - time;
		if(diff <= -2)
		{
			m_last_update = time;
		}else
		{
			check_and_notify();
			return;
		}
	}
	check_and_notify();
	{
	std::lock_guard<std::mutex> lck(m_enumerate_mutex);
	m_thread_enumerate_start = true;
	}
	m_thread_enumerate_cv.notify_one();
}

void RemovableDriveManager::check_and_notify()
{
	if (m_thread_enumerate_finnished)
	{
		m_thread_enumerate_finnished = false;
		m_drives_mutex.lock();
		size_t drives_count = m_current_drives.size();
		m_drives_mutex.unlock();
		if (m_drives_count != drives_count)
		{
			if (m_drive_count_changed_callback)
			{
				m_drive_count_changed_callback(m_plater_ready_to_slice);
			}
			if (m_callbacks.size() != 0 && m_drives_count > drives_count && !is_drive_mounted(m_last_save_path))
			{
				for (auto it = m_callbacks.begin(); it != m_callbacks.end(); ++it)
				{
					(*it)();
				}
			}
			m_drives_count = drives_count;
		}
	}
}

bool RemovableDriveManager::is_drive_mounted(const std::string &path) 
{
	m_drives_mutex.lock();
	for (auto it = m_current_drives.begin(); it != m_current_drives.end(); ++it)
	{
		if ((*it).path == path)
		{
			m_drives_mutex.unlock();
			return true;
		}
	}
	m_drives_mutex.unlock();
	return false;
}
std::string RemovableDriveManager::get_drive_path() 
{
	m_drives_mutex.lock();
	size_t drives_count = m_current_drives.size();
	m_drives_mutex.unlock();
	if (drives_count == 0)
	{
		reset_last_save_path();
		return "";
	}
	if (m_last_save_path_verified)
		return m_last_save_path;
	m_drives_mutex.lock();
	std::string r = m_current_drives.back().path;
	m_drives_mutex.unlock();
	return std::move(r);
}
std::string RemovableDriveManager::get_last_save_path() const
{
	if (!m_last_save_path_verified)
		return "";
	return m_last_save_path;
}
std::string RemovableDriveManager::get_last_save_name() const
{
	return m_last_save_name;
}


void RemovableDriveManager::add_remove_callback(std::function<void()> callback)
{
	m_callbacks.push_back(callback);
}
void RemovableDriveManager::erase_callbacks()
{
	m_callbacks.clear();
}
void RemovableDriveManager::set_drive_count_changed_callback(std::function<void(const bool)> callback)
{
	m_drive_count_changed_callback = callback;
}
void RemovableDriveManager::set_plater_ready_to_slice(bool b)
{
	m_plater_ready_to_slice = b;
}
void RemovableDriveManager::set_last_save_path(const std::string& path)
{
	if(m_last_save_path_verified)// if old path is on drive 
	{
		if(get_drive_from_path(path) != "") //and new is too, rewrite the path
		{
			m_last_save_path_verified = false;
			m_last_save_path = path;
		}//else do nothing
	}else
	{
		m_last_save_path = path;
	}
}
void RemovableDriveManager::verify_last_save_path()
{
	std::string last_drive = get_drive_from_path(m_last_save_path);
	if (last_drive != "")
	{
		m_last_save_path_verified = true;
		m_last_save_path = last_drive;
		m_last_save_name = get_drive_name(last_drive);
	}else
	{
		reset_last_save_path();
	}
}
std::string RemovableDriveManager::get_drive_name(const std::string& path) {
	m_drives_mutex.lock();
	size_t drives_count = m_current_drives.size();
	m_drives_mutex.unlock();
	if (drives_count == 0)
	{
		return "";
	}
	m_drives_mutex.lock();
	for (auto it = m_current_drives.begin(); it != m_current_drives.end(); ++it)
	{
		if ((*it).path == path)
		{
			m_drives_mutex.unlock();
			return (*it).name;
		}
	}
	m_drives_mutex.unlock();
	return "";
}
bool RemovableDriveManager::is_last_drive_removed() 
{
	if(!m_last_save_path_verified)
	{
		return true;
	}
	bool r = !is_drive_mounted(m_last_save_path);
	if (r) 
	{
		reset_last_save_path();
	}
	return r;
}

void RemovableDriveManager::reset_last_save_path()
{
	m_last_save_path_verified = false;
	m_last_save_path = "";
	m_last_save_name = "";
}
void RemovableDriveManager::set_is_writing(const bool b)
{
	m_is_writing = b;
	if (b)
	{
		m_did_eject = false;
	}
}
bool RemovableDriveManager::get_is_writing() const
{
	return m_is_writing;
}
bool RemovableDriveManager::get_did_eject() const
{
	return m_did_eject;
}
void RemovableDriveManager::set_did_eject(const bool b) 
{
	m_did_eject = b;
}
size_t RemovableDriveManager::get_drives_count() 
{
	m_drives_mutex.lock();
	size_t ret = m_current_drives.size();
	m_drives_mutex.unlock();
	return ret;
}
std::string RemovableDriveManager::get_ejected_path() const
{
	return m_ejected_path;
}
std::string RemovableDriveManager::get_ejected_name() const
{
	return m_ejected_name;
}

void RemovableDriveManager::thread_proc()
{
	while (true)
	{
		// Wait until update needs enumeration
		std::unique_lock<std::mutex> lck(m_enumerate_mutex);
		m_thread_enumerate_cv.wait(lck, [this] {return m_thread_enumerate_start; });
		if (!m_initialized) // call from destructor to end the loop
		{
			
			m_thread_finnished = true;
			lck.unlock();
			m_thread_enumerate_cv.notify_one();//tell destructor loop is over
			break;
		}
		m_thread_enumerate_start = false;
		search_for_drives();
		m_thread_enumerate_finnished = true; //atomic to let main thread proceed with check_and_notify()
		lck.unlock();
	}
	
	

}
}}//namespace Slicer::Gui
