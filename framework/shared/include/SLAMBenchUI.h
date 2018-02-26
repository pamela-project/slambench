
#ifndef SLAMBENCHUI_H_
#define SLAMBENCHUI_H_

// #define DEBUG_MUTEXES

#include <SLAMBenchUITypes.h>
#include <outputs/OutputManager.h>

#include <cstdlib>
#include <cstring>
#include <map>
#include <vector>
#include <iostream>
#include <typeinfo>
#include <mutex>
#ifdef DEBUG_MUTEXES
#include <thread>
#endif
#include <cassert>


/*
 * ************************************************************************************************
 * ********************** SLAMBenchUI
 * ************************************************************************************************
 */

typedef std::vector<std::pair< VisibleElementIdentifier , void * >> ui_context;
typedef std::map<std::string, ui_context>                           ui_contexts_pool;



class SLAMBenchUI
{

protected :
	int frame  = 0;
    int width  = 0;
    int height = 0;
    float cx = 0,cy = 0,fx = 0,fy = 0;


public :
	typedef std::vector<std::pair<std::string, slambench::outputs::OutputManager*>> output_manager_container;

    virtual ~SLAMBenchUI() {};

    virtual bool process() = 0;

	virtual void stepFrame()        { this->frame += 1; }
	virtual bool IsFreeRunning()    { return true; }
	virtual bool CanFreeRun()       { return true; }
	virtual bool SetFreeRunning()   { return true; }
	virtual bool ClearFreeRunning() { return false; }
	virtual bool WaitForFrame()     { return false; }
	virtual bool CanStep()          { return false; }

    inline void update_camera(int height , int width, float fx, float  fy , float  cx , float  cy ) {
        this->width = width;
        this->height = height;
        this->fx = fx;
        this->fy = fy;
        this->cx = cx;
        this->cy = cy;
    };


	void AddOutputManager(const std::string &name, slambench::outputs::OutputManager *mgr) { output_managers_.push_back(std::make_pair(name, mgr)); }
	const output_manager_container &GetOutputManagers() { return output_managers_; }
private :
	output_manager_container output_managers_;
	
};
#endif
