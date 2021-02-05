
#include "cluster3d.h"

namespace skywell {


ClusterPts::~ClusterPts() {}


void ClusterPts::clusterHelper(int ind, PtCdtr cloud, std::vector<int> &cluster, KdTree *tree) {
    processed[ind] = true;
    cluster.push_back(ind);

    std::vector<int> nearest_point = tree->search(cloud->points[ind], distanceTol);
    for (int nearest_id:nearest_point) {
        if (!processed[nearest_id]) {
            clusterHelper(nearest_id, cloud, cluster, tree);
        }
    }

}


void ClusterPts::EuclidCluster(PtCdtr cloud,ClusterVectorPtr midsave) {
    KdTree *tree = new KdTree;
    for (int ind = 0; ind < num_points; ind++) {
        tree->insert(cloud->points[ind], ind);
    }
    for (int ind = 0; ind < num_points; ind++) {
        if (processed[ind]) {
            ind++;
            continue;
        }
        std::vector<int> cluster_ind;
		boost::shared_ptr<std::vector<int>> category = boost::make_shared<std::vector<int>>();
        clusterHelper(ind, cloud, cluster_ind, tree);

        int cluster_size = cluster_ind.size();
        if (cluster_size >= minClusterSize && cluster_size <= maxClusterSize) {
            for (int i = 0; i < cluster_size; i++) {
				category->push_back(cluster_ind[i]);//存入索引
            }
        }
		if (category->size() >= minClusterSize) 
		{
			boost::shared_ptr<ClusterSave> cs = boost::make_shared<ClusterSave>(cloud, category);
			midsave->push_back(cs);
		}
    }
	delete tree;
	tree = NULL;
}

}