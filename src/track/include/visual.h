//可视化显示
#ifndef _VISUAL_H_
#define _VISUAL_H_


#include"type.h"
#include<pcl/visualization/pcl_visualizer.h>
#include<vector>
#include<pthread.h>



namespace skywell {




	class Visual {
		typedef boost::function<void(boost::shared_ptr<pcl::visualization::PCLVisualizer>)> func_type;
		typedef boost::shared_ptr<std::vector<boost::shared_ptr<std::vector<int>>>> DBSCANVector;
	public:
		Visual();
		bool wasStopped(void);
		void spinOnce(void);

	public:
		//获取viewer
		pcl::visualization::PCLVisualizer* getviewer() {
			return m_viewer.get();
		}
		//绘制包围盒
		void drawBox(const boost::shared_ptr<std::vector<Object>> cubes,double frontsize ,int viewport = 0);
	private:
		boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
	};
}

#endif