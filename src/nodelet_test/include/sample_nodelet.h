#ifndef __SAMPLE_NODELET_H__
#define __SAMPLE_NODELET_H__
#include <nodelet/nodelet.h>
namespace lidar_cluster
{
	class SampleNodelet: public nodelet::Nodelet{
	private:
		virtual void onInit();
	};
}

#endif