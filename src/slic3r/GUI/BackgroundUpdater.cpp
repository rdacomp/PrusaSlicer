#include "BackgroundUpdater.hpp"

#include "Mouse3DController.hpp"
#include "GUI_App.hpp"

#include <iostream>
#include <boost/chrono.hpp>
#include <boost/bind.hpp>
#include <boost/log/trivial.hpp>

namespace Slic3r {
namespace GUI {

wxDEFINE_EVENT(EVT_BACKGROUND_UPDATER_MOUSE3D_NEW_DATA, Event<void*>);

BackgroundUpdater::BackgroundUpdater():
	m_running(false)
	,m_mouse3d_has_mouse(false)
{
}
BackgroundUpdater::~BackgroundUpdater(){
	stop();
}
void BackgroundUpdater::start(){
	m_thread = boost::thread((boost::bind(&BackgroundUpdater::thread_proc_safe, this)));
	m_running = true;
}
void BackgroundUpdater::stop(){
	if(m_running){
		m_running = true;
		m_thread.join();
	}
		
}

void BackgroundUpdater::thread_proc_safe()
{
	thread_proc();
	/* TODO: make safe
	try {
		thread_proc();
	}
	catch (...) {
		//wxTheApp->OnUnhandledException();
	}*/
}
void BackgroundUpdater::thread_proc(){
	while (/*m_running*/true)
	{
		//here you can add your calls for background calculations
		mouse3d_enumerate();
		boost::this_thread::sleep_for(boost::chrono::seconds(2));
	}
}

// Mouse3DController.hpp ----------------------
void BackgroundUpdater::mouse3d_set_has_mouse(bool h) {
	m_mouse3d_mutex_bool.lock();
	m_mouse3d_has_mouse = h; 
	m_mouse3d_mutex_bool.unlock();
}
void BackgroundUpdater::mouse3d_enumerate()
{
	// Mouse3DController sets if to do enumeration
	bool procced = false;
	m_mouse3d_mutex_bool.lock();
	procced = !m_mouse3d_has_mouse;
	m_mouse3d_mutex_bool.unlock();

	if (procced) {
		hid_device_info* devices = hid_enumerate(0, 0);
		//send event to Mouse3DController with data from hid_enumerate
		wxPostEvent((wxEvtHandler*)wxGetApp().plater_, Event<void*>(EVT_BACKGROUND_UPDATER_MOUSE3D_NEW_DATA, devices));
	}
}
//  ----------------------

}} // namaspace Slic3r::GUI
