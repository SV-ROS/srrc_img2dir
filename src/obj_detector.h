// rs_obj_detector.h

#ifndef RS_OBJ_DETECTOR_H
#define RS_OBJ_DETECTOR_H


#include "PixelTraits.h"

namespace obj_detector {

    class ImageClusterizer {
    public:
        typedef pixel_traits::BgrClusterPp ClusterPp;
        typedef ClusterPp::DisjointSet Clusters;
        typedef Clusters::Cluster Cluster;
        typedef std::vector<clustering::ClusterIndex> ClusterIndices;

        ImageClusterizer()
            : clusters_(Clusters::Rules(clustering::XyFrame(0, 0), 0, Clusters::Rules::ValueTraits(0, 0)))
        {
        }
        ImageClusterizer(int w, int h)
            : clusters_(Clusters::Rules(clustering::XyFrame(w, h), 0, Clusters::Rules::ValueTraits(0, 0)))
        {
        }
        ~ImageClusterizer() {
        }

        Clusters const& getClusters() const { return clusters_; }

        ClusterIndices const& getCandidateClusterIndices() const { return candidate_clusters_; }
        std::size_t getNumOfCandidateClusters() const { return candidate_clusters_.size(); }
        std::vector<pixel_traits::DistanceRangeInM> const& getNotBadClusterDistances() const { return not_bad_cluster_distances_; }
        std::size_t getNumOfCandidatesWithDistance() const { return num_of_candidates_with_d_; }
        std::size_t getNumOfCandidatesWithInfDistance() const { return num_of_candidates_with_inf_d_; }
        std::size_t getNumOfBestCandidatesWithDistance() const { return num_of_best_candidates_with_d_; }
        std::size_t getNumOfBestCandidatesWithInfDistance() const { return num_of_best_candidates_with_inf_d_; }

        Cluster const& getCandidateCluster(std::size_t candidate_cluster_index) const {
            clustering::ClusterIndex top_cluster_index = candidate_clusters_[candidate_cluster_index];
            return clusters_.getTopCluster(top_cluster_index);
        }

        cv::Point2d getObjectCenterInPixels(std::size_t candidate_cluster_index) const {
            Cluster const& cluster = getCandidateCluster(candidate_cluster_index);
            pixel_traits::PixelXy p = cluster.xy_range_.getTargetPoint();
            cv::Point2d res;
            res.x = p.column;
            res.y = p.row;
            return res;
        }

        pixel_traits::ClusterPpStatus getObjectPpStatus(std::size_t candidate_cluster_index) const {
            Cluster const& cluster = getCandidateCluster(candidate_cluster_index);
            return cluster.pp_status_;
        }


        void clusterize(cv::Mat const& bgrImage, pixel_traits::ColorClustersParams const& params) {
            clusters_.getMutableRules().setXyFrame(clustering::XyFrame(bgrImage.cols, bgrImage.rows));
            clusters_.getMutableRules().setHalfWindowSize(params.halfWindowSize);
            clusters_.getMutableRules().getMutableValueTraits().color_distance_threshold_ = params.colorDistanceThreshold;
            clusters_.getMutableRules().getMutableValueTraits().black_threshold_ = params.black_threshold;

            pixel_traits::BgrImage image(bgrImage);
            clusters_.clusterizeImage(image);
        }

        void clusterizeAndPp(cv::Mat const& bgrImage, pixel_traits::ColorClustersParams const& params) {
            clusterize(bgrImage, params);
            ClusterPp pp(params);
            candidate_clusters_ = pp.postProcessAndFindCandidteClusters(clusters_);
        }

        //: filter not bad clusters found in mono image processing:
        void filterCandidatesByDistance(pixel_traits::XyzMat const& xyz_image) {
            not_bad_cluster_distances_ = std::vector<pixel_traits::DistanceRangeInM>(getNumOfCandidateClusters());
            for (int32_t u = 0, i = 0; u < xyz_image.rows; ++u) {
              for (int32_t v = 0; v < xyz_image.cols; ++v, ++i) {
                int not_bad_cluster_index = clusters_.getTopCluster(i).pp_cluster_index_;
                if(not_bad_cluster_index != -1) {
                  const cv::Vec3f& p = xyz_image(u,v);
                  pixel_traits::DistanceInM d = pixel_traits::getDistanceInM(p);
                  not_bad_cluster_distances_[not_bad_cluster_index].add(d);
                }
              }
            }
            ClusterIndices candidate_clusters_with_d;
            ClusterIndices candidate_clusters_with_inf_d;
            ClusterIndices candidate_clusters_without_d;
            std::vector<pixel_traits::DistanceRangeInM> not_bad_cluster_distances;
            for(int not_bad_cluster_index = 0; not_bad_cluster_index < getNumOfCandidateClusters(); ++not_bad_cluster_index) {
              pixel_traits::DistanceRangeInM const& d = not_bad_cluster_distances_[not_bad_cluster_index];
              clustering::ClusterIndex candidate_claster_index = candidate_clusters_[not_bad_cluster_index];
              if(d.isValid()) {
                  candidate_clusters_with_d.push_back(candidate_claster_index);
                  not_bad_cluster_distances.push_back(d);
              } else if(d.has_inf_)
                  candidate_clusters_with_inf_d.push_back(candidate_claster_index);
              else
                  candidate_clusters_without_d.push_back(candidate_claster_index);
            }

            not_bad_cluster_distances_.swap(not_bad_cluster_distances);

            ClusterIndices::iterator candidate_clusters_itr = candidate_clusters_.begin();
            candidate_clusters_itr = std::copy(candidate_clusters_with_d.begin(), candidate_clusters_with_d.end(), candidate_clusters_itr);
            candidate_clusters_itr = std::copy(candidate_clusters_with_inf_d.begin(), candidate_clusters_with_inf_d.end(), candidate_clusters_itr);
            std::copy(candidate_clusters_without_d.begin(), candidate_clusters_without_d.end(), candidate_clusters_itr);
            for(int not_bad_cluster_index = 0; not_bad_cluster_index < candidate_clusters_.size(); ++not_bad_cluster_index) {
              clustering::ClusterIndex candidate_claster_index = candidate_clusters_[not_bad_cluster_index];
              clusters_.getTopClusterMutable(candidate_claster_index).pp_cluster_index_ = not_bad_cluster_index;
            }
            num_of_best_candidates_with_d_ = num_of_candidates_with_d_ = candidate_clusters_with_d.size();
            num_of_best_candidates_with_inf_d_ = num_of_candidates_with_inf_d_ = candidate_clusters_with_inf_d.size();
        }

        template<typename t_PinholeCam>
        void filterCandidatesByEstimatedSize(t_PinholeCam const& cam_model, pixel_traits::StereoClustersParams const& params) {
            ClusterIndices good_candidate_clusters_with_d;
            ClusterIndices bad_candidate_clusters_with_d;
            ClusterIndices good_candidate_clusters_with_inf_d;
            ClusterIndices bad_candidate_clusters_with_inf_d;
            for(int not_bad_cluster_index = 0; not_bad_cluster_index < num_of_candidates_with_d_; ++not_bad_cluster_index) {
              pixel_traits::DistanceRangeInM const& d = not_bad_cluster_distances_[not_bad_cluster_index];
              clustering::ClusterIndex candidate_claster_index = candidate_clusters_[not_bad_cluster_index];
              Cluster const& cluster = clusters_.getTopCluster(candidate_claster_index);
              double obj_width_in_m = cam_model.getDeltaX(cluster.xy_range_.getWidth(), d.getAvgDistanceInM());
              if(params.minObjSizeInM <= obj_width_in_m && obj_width_in_m <= params.minObjSizeInM)
                  good_candidate_clusters_with_d.push_back(candidate_claster_index);
              else
                  bad_candidate_clusters_with_d.push_back(candidate_claster_index);
            }

            for(int not_bad_cluster_index = num_of_candidates_with_d_; not_bad_cluster_index < num_of_candidates_with_d_ + num_of_candidates_with_inf_d_; ++not_bad_cluster_index) {
              pixel_traits::DistanceRangeInM const& d = not_bad_cluster_distances_[not_bad_cluster_index];
              clustering::ClusterIndex candidate_claster_index = candidate_clusters_[not_bad_cluster_index];
              Cluster const& cluster = clusters_.getTopCluster(candidate_claster_index);
              if(cluster.xy_range_.getMaxSize() <= params.maxInfObjSizeInPixels)
                  good_candidate_clusters_with_inf_d.push_back(candidate_claster_index);
              else
                  bad_candidate_clusters_with_inf_d.push_back(candidate_claster_index);
            }

            ClusterIndices::iterator candidate_clusters_itr = candidate_clusters_.begin();
            candidate_clusters_itr = std::copy(good_candidate_clusters_with_d.begin(), good_candidate_clusters_with_d.end(), candidate_clusters_itr);
            candidate_clusters_itr = std::copy(bad_candidate_clusters_with_d.begin(), bad_candidate_clusters_with_d.end(), candidate_clusters_itr);
            candidate_clusters_itr = std::copy(good_candidate_clusters_with_inf_d.begin(), good_candidate_clusters_with_inf_d.end(), candidate_clusters_itr);
            candidate_clusters_itr = std::copy(bad_candidate_clusters_with_inf_d.begin(), bad_candidate_clusters_with_inf_d.end(), candidate_clusters_itr);
            for(int not_bad_cluster_index = 0; not_bad_cluster_index < candidate_clusters_.size(); ++not_bad_cluster_index) {
              clustering::ClusterIndex candidate_claster_index = candidate_clusters_[not_bad_cluster_index];
              clusters_.getTopClusterMutable(candidate_claster_index).pp_cluster_index_ = not_bad_cluster_index;
            }
            num_of_best_candidates_with_d_ = good_candidate_clusters_with_d.size();
            num_of_best_candidates_with_inf_d_ = good_candidate_clusters_with_inf_d.size();
        }

    private:
        Clusters clusters_;
        ClusterIndices candidate_clusters_;
        std::vector<pixel_traits::DistanceRangeInM> not_bad_cluster_distances_;
        std::size_t num_of_candidates_with_d_;
        std::size_t num_of_candidates_with_inf_d_;
        std::size_t num_of_best_candidates_with_d_;
        std::size_t num_of_best_candidates_with_inf_d_;
    };

    class Colorizer {
    public:
        Colorizer() {
            cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
        }

        static cv::Mat preprocessImage(cv::Mat& src_bgr_image, int saturation_threshold, int brightness_threshold) {
            cv::Mat res_image = src_bgr_image.clone();
            cv::cvtColor(res_image, res_image, cv::COLOR_BGR2HSV);
            std::vector<cv::Mat> hsv_channels;
            cv::split(res_image, hsv_channels);
            hsv_channels[1] = (hsv_channels[1] >= saturation_threshold);
            hsv_channels[2] = (hsv_channels[2] >= brightness_threshold);
            cv::merge(hsv_channels, res_image);
//            int rows = res_image.size().height;
//            int cols = res_image.size().width;
//            for(int row = 0; row < rows; ++row) {
//                for(int col = 0; col < cols; ++col) {
//                    cv::Vec3b hsv = res_image.at<cv::Vec3b>(row, col);
//                    hsv[1] = (hsv[1] >= saturation_threshold) ? 255 : 0;
//                    hsv[2] = (hsv[2] >= brightness_threshold) ? 255 : 0;
//                    res_image.at<cv::Vec3b>(row, col) = hsv;
//                }
//            }
            cv::cvtColor(res_image, res_image, cv::COLOR_HSV2BGR);
            return res_image;
        }

        static cv::Mat preprocessToSimpleBwImage(cv::Mat& src_bgr_image, int saturation_threshold, int brightness_threshold) {
            cv::Mat hsv_image = src_bgr_image.clone();
            cv::cvtColor(hsv_image, hsv_image, cv::COLOR_BGR2HSV);
            std::vector<cv::Mat> hsv_channels;
            cv::split(hsv_image, hsv_channels);
            return (hsv_channels[1] >= saturation_threshold) & (hsv_channels[2] >= brightness_threshold);
        }

        void drawInfoText(cv::Mat& result_bgr_image, std::string const& text) const {
            CvSize text_size;
            int baseline;
            cvGetTextSize(text.c_str(), &font_, &text_size, &baseline);
            CvPoint origin = cvPoint(0, result_bgr_image.rows - baseline - 6);
            cv:putText(result_bgr_image, text.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(255,0,0));
        }

        static void drawTarget(cv::Mat& result_bgr_image, cv::Point2d const& target_in_2d, pixel_traits::ClusterPpStatus status) {
            static const int c_radius = 3;
            cv::circle(result_bgr_image, target_in_2d, c_radius, CV_RGB(255,0,0), -1);
            if(status > pixel_traits::PpStatusNotBad) {
                //: draw box:
                static cv::Scalar cian = CV_RGB(0, 255, 255);
                static cv::Scalar magenta = CV_RGB(255, 0, 255);
                static const int c_small_offset = 3 * c_radius;
                static const int c_large_offset = 5 * c_radius;
                bool is_good = (status == pixel_traits::PpStatusGood);
                int offset = is_good ? c_large_offset : c_small_offset;
                cv::Vec2i offset_xy(offset, offset);
                cv::Vec2i center(target_in_2d.x, target_in_2d.y);
                cv::Rect box(center - offset_xy, center + offset_xy);
                cv::rectangle(result_bgr_image, box, is_good ? magenta : cian, is_good ? 3 : 1);
            }
        }

        static void colorPixelsBySampleColor(cv::Mat& result_bgr_image, ImageClusterizer const& od) {
            clustering::XyFrame const& frame = od.getClusters().getRules().getXyFrame();
            for(std::size_t row = 0, i = 0; row < frame.height; ++row) {
                for(std::size_t column = 0; column < frame.width; ++column, ++i) {
                    ImageClusterizer::Cluster const& pixel_cluster = od.getClusters().getTopCluster(i);
                    //**/if(!colorizeAuxPixel(result_bgr_image, row, column, pixel_cluster))
                        result_bgr_image.at<cv::Vec3b>(row, column) = pixel_cluster.getSampleColor();
                }
            }
        }

        static void colorPixelsByRandomColors(cv::Mat& result_bgr_image, ImageClusterizer const& od) {
            //fixme: NIY!
            clustering::XyFrame const& frame = od.getClusters().getRules().getXyFrame();
            for(std::size_t row = 0, i = 0; row < frame.height; ++row) {
                for(std::size_t column = 0; column < frame.width; ++column, ++i) {
                    ImageClusterizer::Cluster const& pixel_cluster = od.getClusters().getTopCluster(i);
                    //**/if(!colorizeAuxPixel(result_bgr_image, row, column, pixel_cluster))
                        result_bgr_image.at<cv::Vec3b>(row, column) = pixel_cluster.getSampleColor();
                }
            }
        }

        void colorPixelsByStatusColor(cv::Mat& result_bgr_image, ImageClusterizer const& od) {
            //enum ClusterPpStatus
            //{
            //    PpStatusNotProcessed,  dark red
            //    PpStatusCrossesBorder, dark cian
            //    PpStatusBadShape,      dark magenta
            //    PpStatusTooSmall,      dark yellow
            //    PpStatusTooLarge,      dark green
            //    PpStatusTooDark,       black
            //    PpStatusTooFar,        dark blue
            //    PpStatusNotBad,        light grey
            //    PpStatusNotBadInf,     blue
            //    PpStatusNotBadSmall,   yellow
            //    PpStatusNotBadLarge,   green
            //    PpStatusGood,          white
            //};
            static const uint8_t b[] = {   0, 128, 128,   0,   0,   0, 128, 192 , 255,   0,   0, 255 };
            static const uint8_t g[] = {   0, 128,   0, 128, 128,   0,   0, 192 ,   0, 255, 255, 255 };
            static const uint8_t r[] = { 128,   0, 128, 128,   0,   0,   0, 192 ,   0, 255,   0, 255 };

            clustering::XyFrame const& frame = od.getClusters().getRules().getXyFrame();
            for(std::size_t row = 0, i = 0; row < frame.height; ++row) {
                for(std::size_t column = 0; column < frame.width; ++column, ++i) {
                    ImageClusterizer::Cluster const& pixel_cluster = od.getClusters().getTopCluster(i);
                    pixel_traits::ClusterPpStatus status = pixel_cluster.pp_status_;
                    //**/if(!colorizeAuxPixel(result_bgr_image, row, column, pixel_cluster))
                        result_bgr_image.at<cv::Vec3b>(row, column) = cv::Vec3b(b[status], g[status], r[status]);
                }
            }
        }

    private:
        template<typename t_Cluster>
        static bool colorizeAuxPixel(cv::Mat& result_bgr_image, std::size_t row, std::size_t column, t_Cluster const& cluster) {
            if(cluster.isBoundaryCluster())
                //: gray:
                result_bgr_image.at<cv::Vec3b>(row, column) = cv::Vec3b(128, 128, 128);
            else if(cluster.isNoDataCluster())
                //: darkish magenta:
                result_bgr_image.at<cv::Vec3b>(row, column) = cv::Vec3b(192, 0, 192);
//            else if(best_cluster == &cluster)
//                //: white:
//                bgrImage.at<cv::Vec3b>(row, column) = cv::Vec3b(255, 255, 255);
            else //: not handled:
                return false;
            //: handled:
            return true;
        }

    private:
        CvFont font_;
    };

}

#endif
