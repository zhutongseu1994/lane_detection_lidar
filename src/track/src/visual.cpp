#include "visual.h"
#include <pcl/point_types.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/make_shared.hpp>
#include <limits>

namespace skywell
{

std::string DoubleToString(const double value, unsigned int precisionAfterPoint)
{
	std::ostringstream out;
	// 清除默认精度
	out.precision(std::numeric_limits<double>::digits10);
	out << value;

	std::string res = std::move(out.str());
	auto pos = res.find('.');
	if (pos == std::string::npos)
		return res;

	auto splitLen = pos + 1 + precisionAfterPoint;
	if (res.size() <= splitLen)
		return res;

	return res.substr(0, splitLen);
}

static std::string int2str(int number)
{
	static char c[20];
	snprintf(c, sizeof(c), "%d", number);
	return c;
}

Visual::Visual()
{
	int v = 0;
	m_viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("3D Viewer");

	m_viewer->setBackgroundColor(0.0, 0.0, 0.0); //设置背景色为黑色
	m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target track");
	m_viewer->addCoordinateSystem(1.0); //建立空间直角坐标系
	//m_viewer->addText("text", 10, 10, "v text");	//标记
	//m_viewer->setCameraPosition(-41.37, 4.58, 39.76, 0, 0, 1, v);//设置相机参数
	//m_viewer->setCameraClipDistances(0.01, 100, v);
	m_viewer->initCameraParameters();
}

bool Visual::wasStopped(void)
{
	return m_viewer->wasStopped();
};
void Visual::spinOnce(void)
{
	m_viewer->spinOnce(100);
};

void Visual::drawBox(const boost::shared_ptr<std::vector<Object>> cubes, double frontsize, int viewport)
{
	std::string object_name;
	std::string cube_name;
	std::string line_name;
	std::string label;

	pcl::PointXYZ center;

	//double theta;
	//double slope;
	double speed;

	pcl::PointXYZ q;

	for (auto &cube : *cubes)
	{
		const int counter = cube.number;

		center.x = cube.transform.x();
		center.y = cube.transform.y();
		center.z = cube.transform.z();

		object_name = "object" + int2str(viewport) + int2str(counter);
		cube_name = "cube" + int2str(viewport) + int2str(counter);
		line_name = "line" + int2str(viewport) + int2str(counter);

		label = int2str(counter);

		//theta = acos(cube.rotate.w()) * 2.0;
		//slope = tan(theta);

		q.x = center.x + cube.speedx * 1.0;
		q.y = center.y + cube.speedy * 1.0;
		q.z = center.z;

		speed = sqrt(cube.speedx * cube.speedx + cube.speedy * cube.speedy);
		std::string stras = DoubleToString(speed, 4);
		m_viewer->addCube(cube.transform, cube.rotate, cube.width, cube.height, cube.depth, cube_name);
		//m_viewer->addLine<pcl::PointXYZ>(center, q, line_name);
		m_viewer->addText3D("num：" + label + "\n speed：" + stras, center, frontsize, 1.0, 1.0, 1.0, object_name);
		//m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,"cube");

		//m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube_name);
		//m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, cube_name);
	}
}

/*void Visual::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* junk)
	{
		if (event.getKeyCode() == ' ' and event.keyDown())
		{
			m_callback(m_viewer);
		}
	}*/

/*void Visual::registerFrameCallback(func_type f)
	{
		m_callback = f;
	}
	*/
/*
	void Visual::run()
	{
		while (!m_viewer->wasStopped())
		{
		}
	}*/
} // namespace skywell
