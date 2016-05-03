// PixelTraits.h
#ifndef PIXELTRAITS_H
#define PIXELTRAITS_H

#include <opencv/cv.h>

#include "DisjointSet.h"

namespace pixel_traits {

    enum ClusterPpStatus {
        PpStatusNotProcessed,
        PpStatusCrossesBorder,
        PpStatusBadShape,
        PpStatusTooSmall,
        PpStatusTooLarge,
        PpStatusTooDark,
        PpStatusTooFar,
        PpStatusNotBad,
        PpStatusNotBadInf,
        PpStatusNotBadSmall,
        PpStatusNotBadLarge,
        PpStatusGood,
    };

    static const int c_BlackThreshold = 100;

    struct ColorClustersParams {
        int halfWindowSize;
        double colorDistanceThreshold;
        int black_threshold;
        int minNumOfPixelsInBestCluster;
        int maxBoxSizeInBestCluster;
        int border_margin;
        double shape_ratio;

        ColorClustersParams()
            : halfWindowSize(2)
            , colorDistanceThreshold(20)
            , black_threshold(c_BlackThreshold)
            , minNumOfPixelsInBestCluster(100)
            , maxBoxSizeInBestCluster(300)
            , border_margin(5)
            , shape_ratio(3)
        {
        }
    };

    struct StereoClustersParams {
        double minObjSizeInM;
        double maxObjSizeInM;
        int maxInfObjSizeInPixels;

        StereoClustersParams()
            : minObjSizeInM(0.02)
            , maxObjSizeInM(0.18)
            , maxInfObjSizeInPixels(10)
        {
        }
    };

    typedef uint8_t uint8;

    template<typename t_Value>
    struct RangeT {
        typedef t_Value Value;
        Value min_;
        Value max_;

        void init(Value v, ...) {
            min_ = max_ = v;
        }
        void add(RangeT const& other) {
            if(min_ > other.min_)
                min_ = other.min_;
            if(max_ < other.max_)
                max_ = other.max_;
        }
    };

    struct PixelXy {
        int column;
        int row;

        int getSqDistTo(PixelXy const& other) const {
            int dc = column - other.column;
            int dr = row - other.row;
            return dr * dr + dc * dc;
        }
    };

    struct PixelXyRange {
        PixelXy min, max;

        PixelXy getTargetPoint() const {
            PixelXy res = { (min.column + max.column) / 2, (min.row + max.row) / 2 };
            return res;
        }
        PixelXy getTopPoint() const {
            PixelXy res = { (min.column + max.column) / 2, min.row };
            return res;
        }
        PixelXy getBottomPoint() const {
            PixelXy res = { (min.column + max.column) / 2, max.row };
            return res;
        }
        int getWidth() const {
            return max.column - min.column + 1;
        }
        int getHeight() const {
            return max.row - min.row + 1;
        }
        int getMaxSize() const {
            return std::max(getHeight(), getWidth());
        }
        int getArea() const {
            return getWidth() * getHeight();
        }

        template<typename t_Point>
        void init(t_Point const& v, int row, int column, int pointIndex) {
            min.column = max.column = column;
            min.row = max.row = row;
        }
        void add(PixelXyRange const& other) {
            if(min.column > other.min.column)
                min.column = other.min.column;
            if(max.column < other.max.column)
                max.column = other.max.column;

            if(min.row > other.min.row)
                min.row = other.min.row;
            if(max.row < other.max.row)
                max.row = other.max.row;
        }
    };


    struct ColorFormat {
        enum Value {
            Bgr8,
            Hsv8,
            Luv8,
        };
    };

    template<ColorFormat::Value c_ColorFormat>
    struct ColorPixelT {
        enum { c_Format = c_ColorFormat };

        typedef cv::Vec3b Data;
        Data data_;

        ColorPixelT() {}
        ColorPixelT(cv::Vec3b const& data) : data_(data) {}
    };

    typedef ColorPixelT<ColorFormat::Bgr8> BgrPixel;
    typedef ColorPixelT<ColorFormat::Hsv8> HsvPixel;

    template<ColorFormat::Value c_ColorFormat>
    class CvImageT {
    public:
        typedef ColorPixelT<c_ColorFormat> PixelData;

        CvImageT(cv::Mat const& image) : image_(image) {}

        std::size_t getTotal() const { return image_.total(); }
        std::size_t getNumOfRows() const { return image_.rows; }
        std::size_t getNumOfColumns() const { return image_.cols; }

        PixelData at(std::size_t row, std::size_t column) const {
            return image_.at<cv::Vec3b>(row, column);
        }

    private:
        cv::Mat const& image_;
    };

    typedef CvImageT<ColorFormat::Bgr8> BgrImage;
    typedef CvImageT<ColorFormat::Hsv8> HsvImage;

    template<ColorFormat::Value c_ColorFormat>
    struct ColorRangeT {
        typedef RangeT<uint8> ByteRange;
        typedef ColorPixelT<c_ColorFormat> Value;

        ByteRange ranges_[3];

        void init(Value const& v, ...) {
            ranges_[0].init(v.data_[0]);
            ranges_[1].init(v.data_[1]);
            ranges_[2].init(v.data_[2]);
        }
        void add(ColorRangeT const& other) {
            ranges_[0].add(other.ranges_[0]);
            ranges_[1].add(other.ranges_[1]);
            ranges_[2].add(other.ranges_[2]);
        }
    };

    struct ColorUtils {
        //: Bgr:
        //

        static bool isWhite(BgrPixel const& p, double white_threshold) {
            //return p.data_[2] + p.data_[1] + p.data_[0] > white_threshold;
            static const BgrPixel c_value(cv::Vec3b(255, 255, 255));
            return getDistance(p, c_value) > white_threshold;
        }
        static bool isBlack(BgrPixel const& p, double black_threshold) {
            //return p.data_[2] + p.data_[1] + p.data_[0] < black_threshold;
            static const ColorPixelT<ColorFormat::Bgr8> c_value(cv::Vec3b(0, 0, 0));
            return getDistance(p, c_value) < black_threshold;
        }

        static double getDistance(BgrPixel const& p1, BgrPixel const& p2) {
            return std::sqrt(sqRgbDistanceApprox((long)p1.data_[2], (long)p1.data_[1], (long)p1.data_[0], (long)p2.data_[2], (long)p2.data_[1], (long)p2.data_[0]));
        }

        static BgrPixel convertToBgr(BgrPixel const& p) {
            return p;
        }
        static HsvPixel convertToHsv(BgrPixel const& p) {
            HsvPixel res;
            //fixme: ng for mt
            static cv::Mat3b bgr(1, 1);
            static cv::Mat3b hsv(1, 1);
            bgr.at<BgrPixel::Data>(0, 0) = p.data_;
            cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
            res.data_ = hsv.at<HsvPixel::Data>(0, 0);
            return res;
        }

        static double sqRgbDistanceApprox(long r1, long g1, long b1, long r2, long g2, long b2) {
          ///: grabbed from http://www.compuphase.com/cmetric.htm
          long rmean = (r1 + r2) / 2;
          long r = r1 - r2;
          long g = g1 - g2;
          long b = b1 - b2;
          return (double)(((512+rmean)*r*r)>>8) + 4*g*g + (((767-rmean)*b*b)>>8);
        }

        //: Hsv:
        //

        static bool isWhite(ColorPixelT<ColorFormat::Hsv8> const& p, double white_threshold) {
            return (p.data_[1] < 80) && (p.data_[2] > white_threshold);
       }
        static bool isBlack(ColorPixelT<ColorFormat::Hsv8> const& p, double black_threshold) {
            return (p.data_[2] < black_threshold);
        }

        static double getDistance(ColorPixelT<ColorFormat::Hsv8> const& p1, ColorPixelT<ColorFormat::Hsv8> const& p2) {
            return std::abs(p1.data_[0] - p2.data_[0]); //fixme? care about red(0|180)? ok for now?
        }

        static BgrPixel convertToBgr(HsvPixel const& p) {
            BgrPixel res;
            //fixme: ng for mt
            static cv::Mat3b bgr(1, 1);
            static cv::Mat3b hsv(1, 1);
            hsv.at<BgrPixel::Data>(0, 0) = p.data_;
            cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
            res.data_ = bgr.at<BgrPixel::Data>(0, 0);
            return res;
        }
        static HsvPixel convertToHsv(HsvPixel const& p) {
            return p;
        }

    };

    struct ColorTraits {
        double color_distance_threshold_;
        double black_threshold_;

        ColorTraits(double distance_threshold, int black_threshold)
            : color_distance_threshold_(distance_threshold)
            , black_threshold_(black_threshold)
        {
        }

        template<typename t_PixelData>
        bool isValid(t_PixelData const& p, ...) const {
            return !ColorUtils::isBlack(p, black_threshold_);
        }

        template<typename t_PixelData>
        bool areClose(t_PixelData const& p1, t_PixelData const& p2) const {
            double d = ColorUtils::getDistance(p1, p2);
            return d < color_distance_threshold_;
        }
    };

    typedef clustering::XyRules<ColorTraits> ColorPixelRules;


    template<ColorFormat::Value c_ColorFormat>
    struct ColorClusterT {
        typedef ColorPixelT<c_ColorFormat> PixelData;

        PixelXyRange xy_range_;
//        ColorRangeT<c_ColorFormat> color_range_;

        HsvPixel hsv_color_sample_;

        ClusterPpStatus pp_status_;

        int pp_cluster_index_;

        void init(PixelData const& p, int row, int column, int pointIndex) {
            xy_range_.init(p, row, column, pointIndex);
//            color_range_.init(p, row, column, pointIndex);

            hsv_color_sample_ = ColorUtils::convertToHsv(p);

            pp_status_ = PpStatusNotProcessed;
            pp_cluster_index_ = -1;
        }
        void add(ColorClusterT const& other, ColorPixelRules const& rules) {
            xy_range_.add(other.xy_range_);
//            color_range_.add(other.color_range_);

            if(other.hsv_color_sample_.data_[1] > hsv_color_sample_.data_[1])
                //: select most saturated sample:
                hsv_color_sample_ = other.hsv_color_sample_;
//            if(!other.color_sample_.isBlack() && color_sample_.dotWhite() > other.color_sample_.dotWhite()
//                || color_sample_.isBlack() && color_sample_.sum() < other.color_sample_.sum())
//            if(color_sample_.dotWhite() > other.color_sample_.dotWhite()
//                || color_sample_.sum() < other.color_sample_.sum())

//            if(rules.getValueTraits().isValid(other.color_sample_) && (RgbTraits::sqDistanceToWhite(color_range_) < RgbTraits::sqDistanceToWhite(other.color_range_)))
//                color_sample_ = other.color_sample_;
        }

        BgrPixel::Data getSampleColor() const {
            HsvPixel faked_hsv = hsv_color_sample_;
            //: preserve hue, top saturation and value:
            faked_hsv.data_[2] = faked_hsv.data_[2] = 255;
            return ColorUtils::convertToBgr(faked_hsv).data_;
        }
    };

    template<ColorFormat::Value c_ColorFormat>
    class ColorClusterPpT {
    public:
        typedef ColorClusterT<c_ColorFormat> ClusterBase;
        typedef typename clustering::DisjointSet<ClusterBase, ColorPixelRules> DisjointSet;
        typedef typename DisjointSet::Cluster Cluster;

        int min_num_of_points_;
        int max_box_size_;
        int dark_threshold_;
        int border_margin_;
        double shape_ratio_;

        ColorClusterPpT(ColorClustersParams const& params)
            : min_num_of_points_(params.minNumOfPixelsInBestCluster)
            , max_box_size_(params.maxBoxSizeInBestCluster)
            , dark_threshold_(params.black_threshold)
            , border_margin_(params.border_margin)
            , shape_ratio_(params.shape_ratio)
        {
        }

        std::vector<clustering::ClusterIndex> postProcessAndFindCandidteClusters(DisjointSet& clusters) const {
            std::vector<clustering::ClusterIndex> res;
            for(int i = 0; i < clusters.getTopClusterIndices().size(); ++i) {
                clustering::ClusterIndex top_cluster_index = clusters.getTopClusterIndices()[i];
                Cluster& cluster = clusters.getTopClusterMutable(top_cluster_index);
                cluster.pp_status_ = getPpStatus(cluster, clusters.getRules().getXyFrame());
                if(cluster.pp_status_ == PpStatusNotBad) {
                    cluster.pp_cluster_index_ = res.size();
                    res.push_back(top_cluster_index);
//                    //fixme: searches for max luminous cluster
//                    uint8 v = cluster.hsv_color_sample_.data_[2];
//                    if(res == -1 || prev_v < v) {
//                        res = top_cluster_index;
//                        prev_v = v;
//                    }
                }
            }
            return res;
        }

    private:
        ClusterPpStatus getPpStatus(Cluster const& cluster, clustering::XyFrame const& frame) const {
            if(cluster.num_of_points_ < min_num_of_points_)
                return PpStatusTooSmall;
            else if(cluster.xy_range_.getMaxSize() > max_box_size_)
                return PpStatusTooLarge;
            else if(ColorUtils::isBlack(cluster.hsv_color_sample_, dark_threshold_))
                return PpStatusTooDark;
            else if(cluster.xy_range_.min.column < border_margin_ || cluster.xy_range_.min.row < border_margin_
                || cluster.xy_range_.max.column > frame.width - border_margin_ - 1
                || cluster.xy_range_.max.row    > frame.height - border_margin_ - 1)
                //: the object is on an image border
                return PpStatusCrossesBorder;
            else if(cluster.xy_range_.getArea() > shape_ratio_ * cluster.num_of_points_)
                //: the object form is suspicious
                return PpStatusBadShape;
            else
                return PpStatusNotBad;
        }
    };

    typedef ColorClusterPpT<ColorFormat::Bgr8> BgrClusterPp;



    typedef float DistanceInM;
    static const DistanceInM c_InfDistance = (DistanceInM)std::numeric_limits<float>::max();
    static const DistanceInM c_NoDistance  = (DistanceInM)0;

    inline bool isValidDistanceInM(DistanceInM d) {
        return (d != c_NoDistance && d != c_InfDistance);
    }

    struct DistanceRangeInM {
        DistanceInM min_, max_;
        bool has_inf_;

        DistanceRangeInM() {
            min_ = c_InfDistance;
            max_ = c_NoDistance;
            has_inf_ = false;
        }

        void add(DistanceInM v) {
            if(isValidDistanceInM(v)) {
                if(this->min_ > v)
                    this->min_ = v;
                if(this->max_ < v)
                    this->max_ = v;
            } else if(v == c_InfDistance)
                has_inf_ = true;
        }

        DistanceInM getAvgDistanceInM() const {
            //fixme?
            return (min_ + max_) / 2;
        }
//        void init(DistanceInM v, ...) {
//            min_ = max_ = v;
//        }
//        void add(DistanceRangeInM const& other) {
//            if(isValidDistanceInM(other.min_)) {
//                if(!isValidDistanceInM(this->min_) || this->min_ > other.min_)
//                    this->min_ = other.min_;
//            }
//            if(isValidDistanceInM(other.max_)) {
//                if(!isValidDistanceInM(this->max_) || this->max_ < other.max_)
//                    this->max_ = other.max_;
//            }

//        }

        bool isValid() const {
            //: both min_ and max_ are either invalid or valid, so it's enough to check one of them:
            return isValidDistanceInM(min_);
        }
    };

    inline DistanceInM getDistanceInM(const cv::Vec3f& p) {
      // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
      // and zero disparities (point mapped to infinity).
      if(p[2] > 1000.) //: image_geometry::StereoCameraModel::MISSING_Z)
        return c_NoDistance;
      if(std::isinf(p[2]))
        return c_InfDistance;
      double d = cv::norm(p);
      return (DistanceInM)d;
    }

    typedef cv::Mat_<cv::Vec3f> XyzMat;



//    template<ColorFormat::Value c_ColorFormat>
//    struct ColorAndDistancePixelT : public ColorPixelT<c_ColorFormat> {
//        typedef ColorPixelT<c_ColorFormat> Base;

//        DistanceInM distance_in_m_;

//        ColorAndDistancePixelT() {}
//        ColorAndDistancePixelT(cv::Vec3b const& color, DistanceInM distance_in_m)
//            : Base(color), distance_in_m_(distance_in_m) {}
//    };

//    typedef ColorAndDistancePixelT<ColorFormat::Bgr8> BgrDPixel;
//    typedef ColorAndDistancePixelT<ColorFormat::Hsv8> HsvDPixel;

//    template<ColorFormat::Value c_ColorFormat>
//    class ColorAndDistanceImageT : public CvImageT<c_ColorFormat> {
//    public:
//        typedef CvImageT<c_ColorFormat> Base;
//        typedef ColorAndDistancePixelT<c_ColorFormat> PixelData;

//        ColorAndDistanceImageT(cv::Mat const& image, cv::Mat const& d_image)
//            : Base(image), d_image_(d_image) {}

//        PixelData at(std::size_t row, std::size_t column) const {
//            return PixelData(Base::at(row, column), d_image_.at<DistanceInM>(row, column));
//        }

//    private:
//        cv::Mat const& d_image_;
//    };

//    struct ColorAndDistanceTraits : public ColorTraits {
//        ColorAndDistanceTraits(double color_distance_threshold, int black_threshold)
//            : ColorTraits(color_distance_threshold, black_threshold)
//        {
//        }
//    };

//    typedef clustering::XyRules<ColorAndDistanceTraits> ColorAndDistancePixelRules;


//    template<ColorFormat::Value c_ColorFormat>
//    struct ColorAndDistanceClusterT : public ColorClusterT<c_ColorFormat> {
//        typedef ColorClusterT<c_ColorFormat> Base;
//        typedef clustering::DisjointSet<ColorAndDistanceClusterT, ColorAndDistancePixelRules> DisjointSet;
//        typedef ColorAndDistancePixelT<c_ColorFormat> PixelData;

//        DistanceRangeInM d_range_in_m_;

//        void init(PixelData const& p, int row, int column, int pointIndex) {
//            Base::init(p, row, column, pointIndex);
//            d_range_in_m_.init(p.distance_in_m_);
//        }
//        void add(ColorAndDistanceClusterT const& other, ColorAndDistancePixelRules const& rules) {
//            Base::add(other, rules);
//            d_range_in_m_.add(other.d_range_in_m_);
//        }
//    };

//    template<ColorFormat::Value c_ColorFormat>
//    struct ColorAndDistanceClusterPpT : public ColorClusterPpT<c_ColorFormat> {
//        typedef ColorClusterPpT<c_ColorFormat> Base;
//        typedef ColorAndDistanceClusterT<c_ColorFormat> Cluster;
//        typedef typename Cluster::DisjointSet DisjointSet;

//        int min_obj_size_in_m_;
//        int max_obj_size_in_m_;

//        ColorAndDistanceClusterPpT(int min_num_of_points, int max_box_size, int dark_threshold = c_BlackThreshold)
//            : Base(min_num_of_points, max_box_size, dark_threshold)
//            , min_obj_size_in_m_(4)
//            , max_obj_size_in_m_(300) //fixme: make param
//        {
//        }

////        clustering::ClusterIndex postProcessAndFindBest(DisjointSet& clusters) const {
////            clustering::ClusterIndex res = -1;
////            double prev_best_sq_distance_to_color = 0;
////            for(int i = 0; i < clusters.getTopClusterIndices().size(); ++i) {
////                clustering::ClusterIndex top_cluster_index = clusters.getTopClusterIndices()[i];
////                Cluster& cluster = clusters.getTopClusterMutable(top_cluster_index);
////                cluster.pp_status_ = getPpStatus(cluster);
////                if(cluster.pp_status_ == PpStatusNotBad) {
//////                    double sq_distance_to_color = cluster.sqDistanceToWhite();
//////                    if(res == -1 || prev_best_sq_distance_to_color > sq_distance_to_color) {
//////                        res = top_cluster_index;
//////                        prev_best_sq_distance_to_color = sq_distance_to_color;
//////                    }
////                    res = top_cluster_index;
////                }
////            }
////            return res;
////        }

////        ClusterPpStatus getPpStatus(Cluster const& cluster) const {
////            if(cluster.num_of_points_ < min_num_of_points_)
////                return PpStatusTooSmall;
////            else if(cluster.xy_range_.getMaxSize() > max_box_size_)
////                return PpStatusTooLarge;
////            else if(cluster.rgb_sample.isBlack(dark_threshold_))
////                return PpStatusTooDark;
////            else
////                return PpStatusNotBad;
////        }
//    };

//    typedef ColorAndDistanceClusterPpT<ColorFormat::Bgr8> BgrDClusterPp;
//    typedef BgrClusterPp::DisjointSet BgrDDisjointSet;


} // namespace pixel_traits

#endif
