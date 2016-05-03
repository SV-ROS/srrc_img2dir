// DisjointSet.h
#ifndef DISJOINTSET_H
#define DISJOINTSET_H

#include <algorithm>
#include <cassert>
//#include <cstdint>
#include <vector>

namespace clustering {

    typedef int ClusterIndex;

    static const int c_BlackThreshold = 100;

    struct ClusterInfo {
        enum {
            c_NoDataIndex = -1,
        };

        ClusterIndex parentClusterIndex;
        int num_of_points_;
        int num_of_boundary_points_;
        bool is_on_boundary_;

        bool isBoundaryCluster() const {
            return is_on_boundary_;
        }
        bool isNoDataCluster() const {
            return parentClusterIndex == c_NoDataIndex;
        }
        bool isRegularCluster() const {
            return parentClusterIndex >= 0;
        }

        void setBoundary(bool is_on_boundary = true) {
            is_on_boundary_ = is_on_boundary;
        }
        void setNoData() {
            parentClusterIndex = c_NoDataIndex;
        }
    };

    template<typename t_Cluster>
    struct ClusterInfoT : public ClusterInfo, public t_Cluster {
    };


    template<typename t_Cluster, typename t_Rules>
    class DisjointSet {
    public:
        typedef ClusterInfoT<t_Cluster> Cluster;
        typedef t_Rules Rules;

        DisjointSet(t_Rules const& rules)
            : rules_(rules) {}

        t_Rules const& getRules() const { return rules_; }
        t_Rules& getMutableRules() { return rules_; }

        template<typename t_Image>
        void clusterizeImage(t_Image const& image) {
            initCusters(image);
            mergeClusters(image);
            countTopClusters();
        }

        ClusterIndex getTopClusterIndex(ClusterIndex clusterIndex) const {
            //return findTopClusterIndex(clusterIndex);
            return clusters_[clusterIndex].isRegularCluster() ? clusters_[clusterIndex].parentClusterIndex : clusterIndex;
        }

        Cluster const& getTopCluster(ClusterIndex clusterIndex) const {
            ClusterIndex topClusterIndex = getTopClusterIndex(clusterIndex);
            return (topClusterIndex >= 0) ? clusters_[topClusterIndex] : clusters_[clusterIndex];
        }

        Cluster& getTopClusterMutable(ClusterIndex clusterIndex) {
            ClusterIndex topClusterIndex = getTopClusterIndex(clusterIndex);
            return (topClusterIndex >= 0) ? clusters_[topClusterIndex] : clusters_[clusterIndex];
        }

        std::vector<ClusterIndex> const& getTopClusterIndices() const { return topClusters_; }

    private:
        ClusterIndex findTopClusterIndex(ClusterIndex clusterIndex) const {
            assert(clusterIndex >= 0 && clusterIndex < clusters_.size());
            ClusterIndex result = clusterIndex;
            while(clusters_[result].isRegularCluster() && clusters_[result].parentClusterIndex != result) {
                result = clusters_[result].parentClusterIndex;
            }
            return result;
        }

        template<typename t_Image>
        void initCusters(t_Image const& image) {
            std::size_t numOfPoints = image.getTotal();
            std::size_t numOfRows = image.getNumOfRows();
            std::size_t numOfColumns = image.getNumOfColumns();
            clusters_.resize(numOfPoints);
            for(std::size_t i = 0, row = 0; row < numOfRows; ++row) {
                for(std::size_t column = 0; column < numOfColumns; ++column, ++i) {
                    rules_.initCluster(clusters_[i], i, row, column, image.at(row, column));
                    clusters_[i].parentClusterIndex = i;
                    clusters_[i].is_on_boundary_ = false;
                    clusters_[i].num_of_points_ = 1;
                    clusters_[i].num_of_boundary_points_ = 0;
                }
            }
        }
        template<typename t_Image>
        void mergeClusters(t_Image const& image) {
            std::size_t numOfRows = image.getNumOfRows();
            std::size_t numOfColumns = image.getNumOfColumns();
            for(std::size_t clusterIndex = 0, row = 0; row < numOfRows; ++row) {
                for(std::size_t column = 0; column < numOfColumns; ++column, ++clusterIndex) {
                    if(rules_.isInvalidData(clusters_[clusterIndex], clusterIndex, row, column, image.at(row, column)))
                        clusters_[clusterIndex].setNoData();
                    else if(isAtClusterBoundary(clusterIndex, row, column, image))
                        clusters_[clusterIndex].setBoundary();
                    else
                        mergeRegularNeighborClusters(clusterIndex, row, column, image);
                }
            }
        }

        template<typename t_Image>
        bool isAtClusterBoundary(ClusterIndex clusterIndex, std::size_t row, std::size_t column, t_Image const& image) const {
            ClusterIndex topClusterIndex = findTopClusterIndex(clusterIndex);
            typename t_Rules::NeighborIndicesEnumerator neighborIndicesEnumerator = rules_.getNeighborIndicesEnumerator(clusterIndex, row, column);
            while(!neighborIndicesEnumerator.empty()) {
                ClusterIndex neighborClusterIndex = *neighborIndicesEnumerator;
                ClusterIndex neighborTopClusterIndex = findTopClusterIndex(neighborClusterIndex);
                if(!rules_.isOfSameCluster(clusterIndex, neighborClusterIndex
                    , clusters_[topClusterIndex], clusters_[neighborTopClusterIndex]
                   , image.at(row, column)
                   , image.at(neighborIndicesEnumerator.getCurrentPixelRow(), neighborIndicesEnumerator.getCurrentPixelColumn())))
                {
                    return true;
                }
                ++neighborIndicesEnumerator;
            }
            return false;
        }

        template<typename t_Image>
        void mergeRegularNeighborClusters(ClusterIndex clusterIndex, std::size_t row, std::size_t column, t_Image const& image) {
            typename t_Rules::NeighborIndicesEnumerator neighborIndicesEnumerator = rules_.getNeighborIndicesEnumerator(clusterIndex, row, column);
            while(!neighborIndicesEnumerator.empty()) {
                ClusterIndex neighborClusterIndex = *neighborIndicesEnumerator;
                std::size_t cur_row = neighborIndicesEnumerator.getCurrentPixelRow();
                std::size_t cur_column = neighborIndicesEnumerator.getCurrentPixelColumn();
                ++neighborIndicesEnumerator;
                //: merge only with neighbor clusters with smaller indices:
                if(neighborClusterIndex >= clusterIndex)
                    continue;
                //: merge only regular clusters:
                if(!clusters_[neighborClusterIndex].isRegularCluster())
                    continue;
                ClusterIndex topClusterIndex = findTopClusterIndex(clusterIndex);
                ClusterIndex neighborTopClusterIndex = findTopClusterIndex(neighborClusterIndex);
                if(topClusterIndex == neighborTopClusterIndex)
                    //: nothing to do: the clusters are already merged;
                    continue;
                //: to avoid cycles always merge toward smaller index:
                ClusterIndex parentClusterIndex = std::min(topClusterIndex, neighborTopClusterIndex);
                ClusterIndex childClusterIndex  = std::max(topClusterIndex, neighborTopClusterIndex);
                if(rules_.mergeNeighborClusters(parentClusterIndex, childClusterIndex
                    , clusters_[parentClusterIndex], clusters_[childClusterIndex]
                    , image.at(row, column)
                    , image.at(cur_row, cur_column)))
                {
                    //: count points in top cluster:
                    clusters_[parentClusterIndex].num_of_points_ += clusters_[childClusterIndex].num_of_points_;
                    //: streamline links to top clusters:
                    clusters_[childClusterIndex].parentClusterIndex
                        = clusters_[clusterIndex].parentClusterIndex
                        = clusters_[neighborClusterIndex].parentClusterIndex
                        = parentClusterIndex;
                }
            }
        }

//        template<typename t_Image>
//        void mergeRegularNeighborClusters(ClusterIndex clusterIndex, std::size_t row, std::size_t column, t_Image const& image) {
//            ClusterIndex topClusterIndex = findTopClusterIndex(clusterIndex);
//            typename t_Rules::NeighborIndicesEnumerator neighborIndicesEnumerator = rules_.getNeighborIndicesEnumerator(clusterIndex, row, column);
//            while(!neighborIndicesEnumerator.empty()) {
//                ClusterIndex neighborClusterIndex = *neighborIndicesEnumerator;
//                std::size_t neighbor_row = neighborIndicesEnumerator.getCurrentPixelRow();
//                std::size_t neighbor_column = neighborIndicesEnumerator.getCurrentPixelColumn();
//                    ++neighborIndicesEnumerator;
//                //: merge only with neighbor clusters with smaller indices:
//                if(neighborClusterIndex > clusterIndex)
//                    continue;
//                //: merge only regular clusters:
//                if(!clusters_[neighborClusterIndex].isRegularCluster()) // || clusters_[neighborClusterIndex].is_on_boundary_)
//                    continue;
//                ClusterIndex neighborTopClusterIndex = findTopClusterIndex(neighborClusterIndex);
//                if(topClusterIndex == neighborTopClusterIndex)
//                    //: nothing to do: the clusters are already merged;
//                    continue;
//                if(!rules_.isOfSameCluster(clusterIndex, neighborClusterIndex
//                    , clusters_[topClusterIndex], clusters_[neighborTopClusterIndex]
//                   , image.at(row, column)
//                   , image.at(neighbor_row, neighbor_column)))
//                {//: not mergeable clusters:
//                    if(!clusters_[clusterIndex].is_on_boundary_) {
//                        clusters_[clusterIndex].is_on_boundary_ = true;
//                        clusters_[topClusterIndex].num_of_boundary_points_++;
//                    }
//                    if(!clusters_[neighborClusterIndex].is_on_boundary_) {
//                        clusters_[neighborClusterIndex].is_on_boundary_ = true;
//                        clusters_[neighborTopClusterIndex].num_of_boundary_points_++;
//                    }
//                    continue;
//                }
//                //: to avoid cycles always merge toward smaller index:
//                ClusterIndex parentClusterIndex = std::min(topClusterIndex, neighborTopClusterIndex);
//                ClusterIndex childClusterIndex  = std::max(topClusterIndex, neighborTopClusterIndex);
//                if(rules_.mergeNeighborClusters(parentClusterIndex, childClusterIndex
//                    , clusters_[parentClusterIndex], clusters_[childClusterIndex]
//                    , image.at(row, column)
//                    , image.at(neighbor_row, neighbor_column)))
//                {
//                    //: count points in top cluster:
//                    clusters_[parentClusterIndex].num_of_points_ += clusters_[childClusterIndex].num_of_points_;
//                    clusters_[parentClusterIndex].num_of_boundary_points_ += clusters_[childClusterIndex].num_of_boundary_points_;
//                    //: streamline links to top clusters:
//                    clusters_[childClusterIndex].parentClusterIndex
//                        = clusters_[clusterIndex].parentClusterIndex
//                        = clusters_[neighborClusterIndex].parentClusterIndex
//                        = parentClusterIndex;
//                }
//            }
//        }

        void countTopClusters() {
            topClusters_.clear();
            for(ClusterIndex i = 0; i < clusters_.size(); ++i) {
                ClusterIndex topClusterIndex = findTopClusterIndex(i);
                if(topClusterIndex == i) {
                    if(clusters_[topClusterIndex].isRegularCluster())
                        topClusters_.push_back(i);
                } else
                    //: streamline the link to the top cluster:
                    clusters_[i].parentClusterIndex = topClusterIndex;
            }
        }

    private:
        t_Rules rules_;
        std::vector<Cluster> clusters_;
        std::vector<ClusterIndex> topClusters_;
    };

    struct XyFrame {
        int width;
        int height;
        int size;

        XyFrame()
            : width()
            , height()
            , size()
        {
        }
        XyFrame(int w, int h)
            : width(w)
            , height(h)
            , size(w * h)
        {
        }

        int getIndex(int column, int row) const {
            return row * width + column;
        }
        int getRow(int index) const {
            return index / width;
        }
        int getColumn(int index) const {
            return index % width;
        }
    };

    struct XyNeighborhood {
        XyFrame frame;
        int halfWindowSize;
        int centerPixelIndex;
        int centerPixelRow;
        int centerPixelColumn;
        int startRow;
        int endRow;
        int startColumn;
        int endColumn;

        XyNeighborhood(XyFrame const& aFrame, int aHalfWindowSize, int aCenterPixelIndex)
            : frame(aFrame)
            , halfWindowSize(aHalfWindowSize)
            , centerPixelIndex(aCenterPixelIndex)
            , centerPixelRow(aFrame.getRow(aCenterPixelIndex))
            , centerPixelColumn(aFrame.getColumn(aCenterPixelIndex))
        {
            startRow = std::max(0, centerPixelRow - halfWindowSize);
            endRow = std::min(centerPixelRow + halfWindowSize + 1, frame.height);
            startColumn = std::max(0, centerPixelColumn - halfWindowSize);
            endColumn = std::min(centerPixelColumn + halfWindowSize + 1, frame.width);
        }
        XyNeighborhood(XyFrame const& aFrame, int aHalfWindowSize, int aCenterPixelColumn, int aCenterPixelRow)
            : frame(aFrame)
            , halfWindowSize(aHalfWindowSize)
            , centerPixelIndex(aFrame.getIndex(aCenterPixelColumn, aCenterPixelRow))
            , centerPixelRow(aCenterPixelRow)
            , centerPixelColumn(aCenterPixelColumn)
        {
            startRow = std::max(0, centerPixelRow - halfWindowSize);
            endRow = std::min(centerPixelRow + halfWindowSize + 1, frame.height);
            startColumn = std::max(0, centerPixelColumn - halfWindowSize);
            endColumn = std::min(centerPixelColumn + halfWindowSize + 1, frame.width);
        }

        int getStartPixelIndex() const {
            return frame.getIndex(startColumn, startRow);
        }
        int getEndPixelIndex() const {
            return 1 + frame.getIndex(endColumn - 1, endRow - 1);
        }
    };

    class XyNeighborIndicesEnumerator {
    private:
        XyNeighborhood neighborhood_;
        std::size_t currentPixelColumn_;
        std::size_t currentPixelRow_;

    public:
        XyNeighborIndicesEnumerator(XyNeighborhood const& neighborhood)
            : neighborhood_(neighborhood)
            , currentPixelColumn_(neighborhood.startColumn)
            , currentPixelRow_(neighborhood.startRow)
        {
        }
        bool empty() const {
            return currentPixelRow_ >= neighborhood_.endRow;
        }
        ClusterIndex operator*() const {
            return neighborhood_.frame.getIndex(currentPixelColumn_, currentPixelRow_);
        }

        std::size_t getCurrentPixelColumn() const { return currentPixelColumn_; }
        std::size_t getCurrentPixelRow() const { return currentPixelRow_; }

        XyNeighborIndicesEnumerator& operator ++() {
            if(++currentPixelColumn_ == neighborhood_.endColumn) {
                ++currentPixelRow_;
                currentPixelColumn_ = neighborhood_.startColumn;
            }
            return *this;
        }
    };

    template<typename t_ValueTraits>
    class XyRules {
    private:
        XyFrame frame_;
        int halfWindowSize_;
        t_ValueTraits valueTraits_;

    public:
        typedef t_ValueTraits ValueTraits;

        typedef XyNeighborIndicesEnumerator NeighborIndicesEnumerator;

        XyRules()
            : frame_()
            , halfWindowSize_()
            , valueTraits_()
        {
        }

        XyRules(XyFrame const& aFrame, int aHalfWindowSize, t_ValueTraits const& aValueTraits)
            : frame_(aFrame)
            , halfWindowSize_(aHalfWindowSize)
            , valueTraits_(aValueTraits)
        {
        }

        XyFrame const& getXyFrame() const { return frame_; }
        void setXyFrame(XyFrame const& frame) { frame_ = frame; }

        int getHalfWindowSize() const { return halfWindowSize_; }
        void setHalfWindowSize(int halfWindowSize) { halfWindowSize_ = halfWindowSize; }

        t_ValueTraits const& getValueTraits() const { return valueTraits_; }
        t_ValueTraits& getMutableValueTraits() { return valueTraits_; }

        NeighborIndicesEnumerator getNeighborIndicesEnumerator(ClusterIndex clusterIndex, std::size_t row, std::size_t column) const {
            return NeighborIndicesEnumerator(XyNeighborhood(frame_, halfWindowSize_, clusterIndex));
        }

        template<typename t_Cluster, typename t_Point>
        void initCluster(t_Cluster& cluster, int pointIndex, std::size_t row, std::size_t column, t_Point const& point) {
            cluster.init(point, row, column, pointIndex);
        }

        template<typename t_Cluster, typename t_Point>
        bool isInvalidData(t_Cluster const& cluster, ClusterIndex clusterIndex, std::size_t row, std::size_t column, t_Point const& point) const {
            return !valueTraits_.isValid(point, row, column);
        }
        template<typename t_Cluster, typename t_Point>
        bool isOfSameCluster(ClusterIndex clusterIndex, ClusterIndex neighborClusterIndex
                    , t_Cluster const& topCluster, t_Cluster const& neighborTopCluster
                    , t_Point const& point, t_Point const& neighborPoint) const {
            return valueTraits_.areClose(point, neighborPoint);
        }

        template<typename t_Cluster, typename t_Point>
        bool mergeNeighborClusters(ClusterIndex parentClusterIndex, ClusterIndex childClusterIndex
                    , t_Cluster& parentCluster, t_Cluster const& childCluster
                    , t_Point const& point, t_Point const& neighborPoint) {
            parentCluster.add(childCluster, *this);
            return true;
        }
    };

} // namespace clustering



#endif
