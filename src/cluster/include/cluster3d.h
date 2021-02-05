

#ifndef PLAYBACK_CLUSTER3D_H
#define PLAYBACK_CLUSTER3D_H

#include <pcl/common/common.h>
#include <chrono>
#include <string>
#include "kdtree3d.h"
#include "cluster.h"

namespace skywell {

    // shorthand for point cloud pointer
    using PtCdtr = typename pcl::PointCloud<pcl::PointXYZI>::Ptr;

    class ClusterPts {
    private:
        int num_points;
        float distanceTol;
        int minClusterSize;
        int maxClusterSize;
        std::vector<bool> processed;
    public:
        ClusterPts(int nPts, float cTol, int minSize, int maxSize) : num_points(nPts), distanceTol(cTol),
                                                                     minClusterSize(minSize), maxClusterSize(maxSize) {
            processed.assign(num_points, false);
        }
        ~ClusterPts();
        void
        clusterHelper(int ind, PtCdtr cloud, std::vector<int> &cluster, KdTree *tree);

        void EuclidCluster(PtCdtr cloud,ClusterVectorPtr midsave);

    };
}
#endif //PLAYBACK_CLUSTER3D_H
