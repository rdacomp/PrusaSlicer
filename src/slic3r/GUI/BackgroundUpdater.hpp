#ifndef slic3r_BackgroundUpdater_hpp_
#define slic3r_BackgroundUpdater_hpp_

#include "Event.hpp"

#include <string>
#include <boost/thread.hpp>
//#include <mutex>
#include <tbb/mutex.h>

namespace Slic3r {
namespace GUI {

wxDECLARE_EVENT(EVT_BACKGROUND_UPDATER_MOUSE3D_NEW_DATA, Event<void*>);

class BackgroundUpdater
{
public:
	BackgroundUpdater();
	~BackgroundUpdater();
	void start();
	void stop();
	// Mouse3DController.hpp ----------------------
	void mouse3d_set_has_mouse(bool h);
	// ----------------------
private:
	boost::thread m_thread;
	bool m_running;
	void thread_proc_safe();
	void thread_proc();

	// Mouse3DController.hpp ----------------------
	tbb::mutex m_mouse3d_mutex_bool;
	bool m_mouse3d_has_mouse;
	void mouse3d_enumerate();
	//  ----------------------

};

}} // namaspace Slic3r::GUI
#endif