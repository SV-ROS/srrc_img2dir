#include <boost/scoped_array.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/Pose.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "std_srvs/Empty.h"
#include <tf/transform_listener.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf/transform_listener.h>

#include <vector>

#include "srrc_cam_pitch_yaw_srv/pitch_yaw.h"
#include "srrc_cam_pitch_yaw_srv/get_cam_pitch_yaw.h"
#include "srrc_cam_pitch_yaw_srv/set_cam_pitch_yaw.h"

#include "srrc_img2dir/get_target_pitch_yaw.h"

#include "DisjointSet.h"
#include "utils.h"

#define IMG2DIR_DEBUG 1 //3
//#define IMG2DIR_NO_THREADS

namespace img2dir
{

std::string input_image_topic_name        = "/rpi_60/image_raw";
std::string input_image_transfer_hint     = "compressed";
std::string output_image_topic_name       = "clustered_image";
std::string output_direction_topic_name   = "target_dir";

double prm_camera_hight_in_m = 0.88;
double prm_max_distance_in_m = 10.0;
double prm_min_obj_size_in_m = 0.04;
double prm_max_obj_size_in_m = 0.18;

int    prm_border_margin = 5;
int    prm_max_num_of_threads = 2;
int    prm_max_num_of_candidates = 3;

typedef clustering::RgbDisjointSet Clusters;
typedef Clusters::Cluster Cluster;

enum PixelColoringMethodEnum
{
    None,
    ByClusterSampleColor,
    ByClusterStatusColor,
//        ByClusterMeanColor,
//        ByClusterMaxColor,
};

struct ColorClustersParams
{
    bool use_camera_info;
    int halfWindowSize;
    double colorDistanceThreshold;
    int black_threshold;
    int minNumOfPixelsInBestCluster;
    int maxBoxSizeInBestCluster;
    int pixelColoringMethod;

    ColorClustersParams()
        : use_camera_info(true)
        , halfWindowSize(2)
        , colorDistanceThreshold(20)
        , minNumOfPixelsInBestCluster(100)
        , maxBoxSizeInBestCluster(300)
        , pixelColoringMethod(ByClusterSampleColor)
    {
    }
};

struct TargetCandidate
{
    PitchYaw py_in_cam_base_system_;
    double sq_distance_to_white_;

    TargetCandidate() : sq_distance_to_white_(0) {}
    TargetCandidate(PitchYaw const& py, double sq_dist_to_white)
        : py_in_cam_base_system_(py)
        , sq_distance_to_white_(sq_dist_to_white)
    {
    }

    bool operator<(TargetCandidate const& rhs) const {
        return sq_distance_to_white_ < rhs.sq_distance_to_white_;
    }
};

struct ScanGrid
{
    std::vector<PitchYaw> py;

    ScanGrid(int rows = 3, int cols = 8, double min_pitch = 0.286, double max_pitch = 0.925, double min_yaw = -1.41, double max_yaw = 1.41)
    {
        py.resize(rows * cols);
        double d_pitch = (rows > 1) ? (max_pitch - min_pitch) / (rows - 1) : 0;
        double d_yaw   = (cols > 1) ? (max_yaw   - min_yaw  ) / (cols - 1) : 0;
        double yaw   = min_yaw;
        bool flipflop = false;
        double pitch = min_pitch;
        for(int i = 0, row = 0; row < rows; row++) {
            for(int col = 0; col < cols; col++, i++) {
                py[i].pitch_ = pitch;
                py[i].yaw_   = yaw;
                yaw += flipflop ? -d_yaw : d_yaw;
            }
            pitch += d_pitch;
            flipflop = !flipflop;
        }
    }
};

class Clusterizer
{
public:
    Clusterizer()
        : center_candidate_index_(-1)
    {
    }

    void reset(ColorClustersParams const& params
               , PitchYaw const& camera_py
               , sensor_msgs::ImageConstPtr const& image_msg
               , sensor_msgs::CameraInfoConstPtr const& info_msg) {
        params_ = params;
        camera_py_ = camera_py;
        good_candidates_.clear();
        center_candidate_index_ = -1;
        //: copy the image and camera info:
        try {
          clustered_image_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& ex){
          ROS_ERROR("[img2dir] Failed to convert image");
          return;
        }

        cam_model_.fromCameraInfo(info_msg);
    }

    void process() {
        Clusters clusters(clustering::RgbPointRules(clustering::XyFrame(clustered_image_->image.cols, clustered_image_->image.rows), 0, clustering::RgbTraits(0, 0)));
        doClusterize(clusters);
        doSearchForGoodCandidates(clusters);
        doColorPixels(clusters);
    }

    std::vector<TargetCandidate> const& getGoodCandidates() const { return good_candidates_; }
    std::vector<TargetCandidate> const& getGoodCandidatesSorted() {
        std::sort(good_candidates_.begin(), good_candidates_.end());
        return good_candidates_;
    }
    bool hasCenterCandidate() const { return center_candidate_index_ != -1; }
    TargetCandidate const& getCenterCandidate() const { return good_candidates_[center_candidate_index_]; }

    cv_bridge::CvImagePtr const& getClusteredImage() const { return clustered_image_; }

private:
    void doClusterize(Clusters& clusters) const {
        clusters.getMutableRules().setHalfWindowSize(params_.halfWindowSize);
        clusters.getMutableRules().getMutableValueTraits().distance_threshold_ = params_.colorDistanceThreshold;
        clusters.getMutableRules().getMutableValueTraits().black_threshold_ = params_.black_threshold;
        clusters.setData(clustered_image_->image);
    }

    void doColorPixels(Clusters const& clusters) {
        switch(params_.pixelColoringMethod) {
        case None:
            return;
        case ByClusterSampleColor:
            doColorPixelsBySampleColor(clusters, clustered_image_->image);
            break;
        case ByClusterStatusColor:
            doColorPixelsByStatusColor(clusters, clustered_image_->image);
            break;
        }
    }

    void doColorPixelsBySampleColor(Clusters const& clusters, cv::Mat& bgrImage) {
        clustering::XyFrame const& frame = clusters.getRules().getXyFrame();

        for(int row = 0, i = 0; row < frame.height; ++row) {
            for(int column = 0; column < frame.width; ++column, ++i) {
                Cluster const& pixelCluster = clusters.getTopCluster(i);
                if(pixelCluster.isBoundaryCluster()) {
                    //: gray:
                    bgrImage.at<cv::Vec3b>(row, column) = cv::Vec3b(128, 128, 128);
                } else if(pixelCluster.isNoDataCluster()) {
                    //: black:
                    bgrImage.at<cv::Vec3b>(row, column) = cv::Vec3b(0, 0, 0);
                } else if(pixelCluster.pp_status == clustering::PpStatusNotBad) {
                    //: white:
                    bgrImage.at<cv::Vec3b>(row, column) = cv::Vec3b(255, 255, 255);
                } else {
                    bgrImage.at<cv::Vec3b>(row, column) = cv::Vec3b(pixelCluster.rgb_sample.b, pixelCluster.rgb_sample.g, pixelCluster.rgb_sample.r);
                }
            }
        }
    }

    void doColorPixelsByStatusColor(Clusters const& clusters, cv::Mat& bgrImage) {
        //enum ClasterPpStatus
        //{
        //    PpStatusNotProcessed,
        //    PpStatusCrossesBorder,
        //    PpStatusBadShape,
        //    PpStatusTooSmall,
        //    PpStatusTooLarge,
        //    PpStatusTooDark,
        //    PpStatusTooFar,
        //    PpStatusNotBad,
        //};
        static const uint8_t b[] = { 128, 255, 255,   0,   0,   0, 255, 200 };
        static const uint8_t g[] = { 128, 255,   0, 255,   0,   0,   0, 200 };
        static const uint8_t r[] = { 128,   0, 255, 255, 255,   0,   0, 200 };

        clustering::XyFrame const& frame = clusters.getRules().getXyFrame();

        for(int row = 0, i = 0; row < frame.height; ++row) {
            for(int column = 0; column < frame.width; ++column, ++i) {
                clustering::ClasterPpStatus status = clusters.getTopCluster(i).pp_status;
                bgrImage.at<cv::Vec3b>(row, column) = cv::Vec3b(r[status], g[status], b[status]);
            }
        }
    }


    PitchYaw getPixelPitchYawWrtCameraSystem(clustering::PixelXy const& p) const {
        cv::Point2d pp(p.column, p.row);
        cv::Point3d dir_in_cam_system = cam_model_.projectPixelTo3dRay(pp);
        return getPitchYawFromDirInCameraSystem(dir_in_cam_system.x, dir_in_cam_system.y, dir_in_cam_system.z);
    }
    PitchYaw getPixelPitchYaw(clustering::PixelXy const& p) const {
        PitchYaw dir_wrt_cam = getPixelPitchYawWrtCameraSystem(p);
        return PitchYaw(camera_py_.pitch_ + dir_wrt_cam.pitch_, camera_py_.yaw_ + dir_wrt_cam.yaw_);
    }
    double getApproxDistanceToPixel(clustering::PixelXy const& p) const {
        PitchYaw py = getPixelPitchYaw(p);
        double sin_pitch = std::max(0.001, std::sin(py.pitch_));
        return prm_camera_hight_in_m / sin_pitch;
    }
    bool checkCluster(Cluster& cluster, int max_row, int max_col, int dark_threshold = clustering::c_BlackThreshold) const {
#if (IMG2DIR_DEBUG > 0)
        std::stringstream str;
        str << "Cluster: [rgb: " << (int)cluster.rgb_sample.r << "," << (int)cluster.rgb_sample.g << "," << (int)cluster.rgb_sample.b << "; "
            << cluster.pixelXy.min.column << ", " << cluster.pixelXy.min.row << ", " << cluster.pixelXy.max.column << ", " << cluster.pixelXy.max.row << "; "
            << cluster.numOfPoints << "]";
        std::string info = str.str();
#endif
#if (IMG2DIR_DEBUG > 1)
        ROS_INFO("%s", info.c_str());
#endif
        if(cluster.numOfPoints < 4) {
            //: the object form is suspicious
            cluster.pp_status = clustering::PpStatusTooSmall;
#if (IMG2DIR_DEBUG > 3)
            ROS_INFO("    -> PpStatusTooSmall (nop: %d)", cluster.numOfPoints);
#endif
            return false;
        }
        if(cluster.rgb_sample.isBlack(dark_threshold)) {
            //: the object form is suspicious
            cluster.pp_status = clustering::PpStatusTooDark;
#if (IMG2DIR_DEBUG > 1)
            ROS_INFO("    -> PpStatusTooDark");
#endif
            return false;
        }
        if(cluster.pixelXy.min.column < prm_border_margin || cluster.pixelXy.min.row < prm_border_margin
            || cluster.pixelXy.max.column > max_col - prm_border_margin - 1
            || cluster.pixelXy.max.row    > max_row - prm_border_margin - 1) {
            //: the object is on an image border
            cluster.pp_status = clustering::PpStatusCrossesBorder;
#if (IMG2DIR_DEBUG > 1)
            ROS_INFO("    -> PpStatusCrossesBorder");
#endif
            return false;
        }
        if(cluster.pixelXy.getArea() > 3 * cluster.numOfPoints) {
            //: the object form is suspicious
            cluster.pp_status = clustering::PpStatusBadShape;
#if (IMG2DIR_DEBUG > 1)
            ROS_INFO("    -> PpStatusBadShape");
#endif
            return false;
        }
        double distance_to_obj_in_m = getApproxDistanceToPixel(cluster.pixelXy.getBottomPoint());
        if(prm_max_distance_in_m < distance_to_obj_in_m) {
            //: the object is too far
            cluster.pp_status = clustering::PpStatusTooFar;
#if (IMG2DIR_DEBUG > 1)
            ROS_INFO("    -> PpStatusTooFar (%f)", distance_to_obj_in_m);
#endif
            return false;
        }
        double obj_width_in_m = cam_model_.getDeltaX(cluster.pixelXy.getWidth(), distance_to_obj_in_m);
        if(obj_width_in_m > prm_max_obj_size_in_m) {
            //: too wide object
            cluster.pp_status = clustering::PpStatusTooLarge;
#if (IMG2DIR_DEBUG > 1)
            ROS_INFO("    -> PpStatusTooLarge (w: %f)", obj_width_in_m);
#endif
            return false;
        }
        if(obj_width_in_m < prm_min_obj_size_in_m) {
            //: too narrow object
            cluster.pp_status = clustering::PpStatusTooSmall;
#if (IMG2DIR_DEBUG > 1)
            ROS_INFO("    -> PpStatusTooSmall (w: %f)", obj_width_in_m);
#endif
            return false;
        }
        double obj_height_in_m = cam_model_.getDeltaY(cluster.pixelXy.getHeight(), distance_to_obj_in_m);
        if(obj_height_in_m > prm_max_obj_size_in_m) {
            //: too tall object
            cluster.pp_status = clustering::PpStatusTooLarge;
#if (IMG2DIR_DEBUG > 1)
            ROS_INFO("    -> PpStatusTooLarge (h: %f)", obj_height_in_m);
#endif
            return false;
        }
        if(obj_height_in_m < prm_min_obj_size_in_m) {
            //: too small object
            cluster.pp_status = clustering::PpStatusTooSmall;
#if (IMG2DIR_DEBUG > 1)
            ROS_INFO("    -> PpStatusTooSmall (h: %f)", obj_height_in_m);
#endif
            return false;
        }
        cluster.pp_status = clustering::PpStatusNotBad;
#if (IMG2DIR_DEBUG == 1)
        ROS_INFO("%s", info.c_str());
#endif
#if (IMG2DIR_DEBUG > 0)
        ROS_INFO("    -> PpStatusGood (d: %f, w: %f, h: %f)", distance_to_obj_in_m, obj_width_in_m, obj_height_in_m);
        PitchYaw py_wrt_cam = getPixelPitchYawWrtCameraSystem(cluster.pixelXy.getTargetPoint());
        PitchYaw py = getPixelPitchYaw(cluster.pixelXy.getTargetPoint());
        ROS_INFO("                    py_wrt_cam: {%f, %f}",  py_wrt_cam.pitch_, py_wrt_cam.yaw_);
        ROS_INFO("                    py_cam:     {%f, %f},", camera_py_.pitch_, camera_py_.yaw_);
        ROS_INFO("                    py:         {%f, %f})", py.pitch_, py.yaw_);
#endif
        return true;
    }

    TargetCandidate asTargetCandidate(Cluster const& cluster) const {
        return TargetCandidate(getPixelPitchYaw(cluster.pixelXy.getTargetPoint()), cluster.sqDistanceToWhite());
    }

    void doSearchForGoodCandidates(Clusters& clusters, int max_dist_to_center = 200) {
        int prev_sq_dist_to_center = max_dist_to_center * max_dist_to_center;
        clustering::PixelXy center = {clustered_image_->image.cols / 2, clustered_image_->image.rows / 2};
        std::vector<clustering::ClusterIndex> top_cluster_indices = clusters.getTopClusterIndices();
        for(int i = 0; i < top_cluster_indices.size(); ++i) {
            Cluster& cluster = clusters.getTopClusterMutable(top_cluster_indices[i]);
            if(checkCluster(cluster, clusters.getRules().getXyFrame().height, clusters.getRules().getXyFrame().width)) {
                good_candidates_.push_back(asTargetCandidate(cluster));
                int sq_dist_to_center = cluster.pixelXy.getTargetPoint().getSqDistTo(center);
                if(prev_sq_dist_to_center > sq_dist_to_center) {
                    prev_sq_dist_to_center = sq_dist_to_center;
                    center_candidate_index_ = good_candidates_.size() - 1;
                }
            }
        }
#if (IMG2DIR_DEBUG > 0)
        ROS_INFO("Img2dir TRACE: in doSearchForGoodCandidates() got %d good candidates of %d", (int)good_candidates_.size(), (int)top_cluster_indices.size());
#endif
    }

private:
    cv_bridge::CvImagePtr clustered_image_;
    image_geometry::PinholeCameraModel cam_model_;
    ColorClustersParams params_;
    PitchYaw camera_py_;

    std::vector<TargetCandidate> good_candidates_;
    int center_candidate_index_;
};

class SingleCamWithMotors
{
private:
    Nh nh_;


    image_transport::ImageTransport it_;

    ros::ServiceClient get_cam_pitch_yaw_client_;
    ros::ServiceClient set_cam_pitch_yaw_client_;

    tf::TransformListener tf_listener_;

    ros::ServiceServer srv_get_target_py_;

    bool is_waiting_for_image_;

    ColorClustersParams params_;
    Clusterizer clusterizer_;

    image_transport::CameraSubscriber dbg_camera_sub_;
    image_transport::Publisher        dbg_clustered_image_pub_;

public:
    SingleCamWithMotors()
      : nh_()
      , it_(nh_.nh_)
      , is_waiting_for_image_(false)
      , clusterizer_()
    {
        updateParams();
        updateParam("camera_hight_in_m",      prm_camera_hight_in_m);
        updateParam("min_obj_size_in_m",      prm_min_obj_size_in_m);
        updateParam("max_obj_size_in_m",      prm_max_obj_size_in_m);
        updateParam("max_distance_in_m",      prm_max_distance_in_m);

        updateParam("border_margin",          prm_border_margin);
        updateParam("max_num_of_threads",     prm_max_num_of_threads);
        updateParam("max_num_of_candidates",  prm_max_num_of_candidates);

        printParams();
        printParam("input_image_topic_name",       input_image_topic_name);
        printParam("input_image_transfer_hint",    input_image_transfer_hint);

        get_cam_pitch_yaw_client_ = nh_.nh_.serviceClient<srrc_cam_pitch_yaw_srv::get_cam_pitch_yaw>("/srrc_cam_pitch_yaw_srv/get_pitch_yaw");
        set_cam_pitch_yaw_client_ = nh_.nh_.serviceClient<srrc_cam_pitch_yaw_srv::set_cam_pitch_yaw>("/srrc_cam_pitch_yaw_srv/set_pitch_yaw");

        srv_get_target_py_ = nh_.nh_.advertiseService("img2dir/get_target_py", &SingleCamWithMotors::srv_get_target_py_cb, this);

#if (IMG2DIR_DEBUG > 1)
        dbg_camera_sub_          = it_.subscribeCamera(input_image_topic_name, 1, &SingleCamWithMotors::dbg_camera_image_cb, this, image_transport::TransportHints(input_image_transfer_hint));
#endif
        dbg_clustered_image_pub_ = it_.advertise(output_image_topic_name, 10);
    }
    
private:
    void dbg_camera_image_cb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
        //: stop subscribing to images:
        //dbg_camera_sub_ = image_transport::CameraSubscriber();
        clusterizer_.reset(params_, getCameraPitchAndYaw(), image_msg, info_msg);
        clusterizer_.process();
        dbg_clustered_image_pub_.publish(clusterizer_.getClusteredImage()->toImageMsg());
    }

    class ImageProcessFunctor
    {
    private:
        Clusterizer* clusterizer_ptr_;

        static int& global_counter() {
            static int g_counter = 0;
            return g_counter;
        }

    public:
        ImageProcessFunctor(Clusterizer* clusterizer_ptr)
            : clusterizer_ptr_(clusterizer_ptr)
        {
        }

        void operator()() {
            global_counter()++;
            clusterizer_ptr_->process();
            global_counter()--;
        }

        Clusterizer const& getClusterizer() { return *clusterizer_ptr_; }

        static int getGlobalCounter() {
            return global_counter();
        }

    };

    class ImageReqest
    {
    public:
        static void obtain(Clusterizer* clusterizer_ptr, SingleCamWithMotors* parent_ptr) {
            ImageReqest request(clusterizer_ptr, parent_ptr);
        }

    private:
        image_transport::CameraSubscriber camera_sub_;
        bool is_waiting_for_image_;
        Clusterizer* clusterizer_ptr_;
        SingleCamWithMotors* parent_ptr_;

        ImageReqest(Clusterizer* clusterizer_ptr, SingleCamWithMotors* parent_ptr)
            : is_waiting_for_image_(true)
            , clusterizer_ptr_(clusterizer_ptr)
            , parent_ptr_(parent_ptr)
        {
            camera_sub_ = parent_ptr_->it_.subscribeCamera(input_image_topic_name, 10, &ImageReqest::on_camera_image_cb, this, image_transport::TransportHints(input_image_transfer_hint));
            ros::Rate r(10); // 10 hz
            while(isWaiting()) {
                ros::spinOnce();
                r.sleep();
            }
        }

        bool isWaiting() const { return is_waiting_for_image_; }

        void on_camera_image_cb(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
            if(!is_waiting_for_image_) {
                return;
            }
            //: stop subscribing to images:
            is_waiting_for_image_ = false;
            camera_sub_.shutdown();

            parent_ptr_->updateParams();
            clusterizer_ptr_->reset(parent_ptr_->params_, parent_ptr_->getCameraPitchAndYaw(), image_msg, info_msg);
        }
    };

    class ClusterizerGrid
    {
    private:
        SingleCamWithMotors* parent_ptr_;
        ScanGrid const py_grid_;
        std::vector<TargetCandidate> good_candidates_;

    public:
        ClusterizerGrid(SingleCamWithMotors* parent_ptr, ScanGrid const& py_grid)
            : parent_ptr_(parent_ptr)
            , py_grid_(py_grid)
        {
        }

        std::vector<TargetCandidate> const& getGoodCandidates() const { return good_candidates_; }

        void scan() {
            doScan(prm_max_num_of_candidates, prm_max_num_of_threads);
            doCheckCandidates(prm_max_num_of_candidates, prm_max_num_of_threads);
#if (IMG2DIR_DEBUG > 0)
            ROS_INFO("Img2dir TRACE: exitting ClusterizerGrid::scan()");
#endif
        }

    private:
        void doScan(int max_num_of_candidates, int max_num_of_threads) {
            //: get images and process them in different threads:
            good_candidates_.clear();
            boost::scoped_array<Clusterizer> clusterizers(new Clusterizer[py_grid_.py.size()]);
#ifndef IMG2DIR_NO_THREADS
            boost::thread_group thread_group;
#endif
            for(int i = 0; i < py_grid_.py.size(); ++i) {
                parent_ptr_->setCameraPitchAndYaw(py_grid_.py[i]);
                ImageReqest::obtain(&clusterizers[i], parent_ptr_);
                ImageProcessFunctor process_functor(&clusterizers[i]);
#ifdef IMG2DIR_NO_THREADS
                process_functor();
                parent_ptr_->dbg_clustered_image_pub_.publish(process_functor.getClusterizer().getClusteredImage()->toImageMsg());
#else
                while(ImageProcessFunctor::getGlobalCounter() > max_num_of_threads)
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
                thread_group.create_thread(process_functor);
#endif
            }
#ifndef IMG2DIR_NO_THREADS
            thread_group.join_all();
#endif
            //: collect all best candidates:
            for(int i = 0; i < py_grid_.py.size(); ++i) {
                Clusterizer const& clusterizer = clusterizers[i];
                good_candidates_.insert(good_candidates_.end(), clusterizer.getGoodCandidates().begin(), clusterizer.getGoodCandidates().end());
            }
            std::sort(good_candidates_.begin(), good_candidates_.end());
#if (IMG2DIR_DEBUG > 0)
            ROS_INFO("Img2dir TRACE: in ClusterizerGrid::scan() got %d candidates", (int)good_candidates_.size());
            for(int i = 0; i < good_candidates_.size(); ++i) {
                TargetCandidate const& item = good_candidates_[i];
                ROS_INFO("     %d# {%f, %f}, %f", i+1, item.py_in_cam_base_system_.pitch_, item.py_in_cam_base_system_.yaw_, item.sq_distance_to_white_);
            }
#endif
            if(good_candidates_.size() > max_num_of_candidates) {
#if (IMG2DIR_DEBUG > 0)
                ROS_INFO("Img2dir TRACE: in ClusterizerGrid::scan() dropped last %d candidates", (int)(good_candidates_.size() - max_num_of_candidates));
#endif
                good_candidates_.resize(max_num_of_candidates);
            }
        }

        void doCheckCandidates(int max_num_of_candidates, int max_num_of_threads) {
            std::vector<TargetCandidate> checked_candidates;
            //: get images and process them in different threads:
            boost::scoped_array<Clusterizer> clusterizers(new Clusterizer[py_grid_.py.size()]);
#ifndef IMG2DIR_NO_THREADS
            boost::thread_group thread_group;
#endif
            for(int i = 0; i < good_candidates_.size(); ++i) {
                parent_ptr_->setCameraPitchAndYaw(good_candidates_[i].py_in_cam_base_system_);
                ImageReqest::obtain(&clusterizers[i], parent_ptr_);
                ImageProcessFunctor process_functor(&clusterizers[i]);
#ifdef IMG2DIR_NO_THREADS
                process_functor();
                parent_ptr_->dbg_clustered_image_pub_.publish(process_functor.getClusterizer().getClusteredImage()->toImageMsg());
#else
                while(ImageProcessFunctor::getGlobalCounter() > max_num_of_threads)
                    boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
                thread_group.create_thread(process_functor);
#endif
            }
#ifndef IMG2DIR_NO_THREADS
            thread_group.join_all();
#endif
            //: collect all center candidates:
            for(int i = 0; i < good_candidates_.size(); ++i) {
                Clusterizer const& clusterizer = clusterizers[i];
                if(clusterizer.hasCenterCandidate())
                    checked_candidates.push_back(clusterizer.getCenterCandidate());
            }
            good_candidates_ = checked_candidates;
            std::sort(good_candidates_.begin(), good_candidates_.end());
#if (IMG2DIR_DEBUG > 0)
            ROS_INFO("Img2dir TRACE: in ClusterizerGrid::doCheckCandidates() got %d candidates", (int)good_candidates_.size());
            for(int i = 0; i < good_candidates_.size(); ++i) {
                TargetCandidate const& item = good_candidates_[i];
                ROS_INFO("     %d# {%f, %f}, %f", i+1, item.py_in_cam_base_system_.pitch_, item.py_in_cam_base_system_.yaw_, item.sq_distance_to_white_);
            }
#endif
            if(good_candidates_.size() > max_num_of_candidates) {
#if (IMG2DIR_DEBUG > 0)
                ROS_INFO("Img2dir TRACE: in ClusterizerGrid::doCheckCandidates() dropped %d candidates", (int)(good_candidates_.size() - max_num_of_candidates));
#endif
                good_candidates_.resize(max_num_of_candidates);
            }
        }
    };


    void handleClusterizerResults(PitchYaw const& py, srrc_img2dir::get_target_pitch_yaw::Request& request, srrc_img2dir::get_target_pitch_yaw::Response& response) {
        response.valid = true;
        response.pitch = py.pitch_;
        response.yaw   = py.yaw_;
//        //: publish the target direction:
//        srrc_cam_pitch_yaw_srv::pitch_yaw msg;
//        msg.pitch = py.pitch_;
//        msg.yaw   = py.yaw_;
//        target_dir_pub_.publish(msg);
        //: move the camera:
        if(request.center)
            setCameraPitchAndYaw(py);
    }



    bool srv_get_target_py_cb(srrc_img2dir::get_target_pitch_yaw::Request& request, srrc_img2dir::get_target_pitch_yaw::Response& response) {
        if(is_waiting_for_image_) {
            ROS_INFO("Img2dir WARNING: busy with other request");
            return false;
        }
        if(request.scan) {
            ClusterizerGrid cg(this, ScanGrid());
            cg.scan();
#if (IMG2DIR_DEBUG > 0)
            ROS_INFO("Img2dir TRACE: exited ClusterizerGrid::scan()");
#endif
            if(!cg.getGoodCandidates().empty())
                handleClusterizerResults(cg.getGoodCandidates()[0].py_in_cam_base_system_, request, response);
            else
                response.valid = false;
        } else {
            //: request one image:
            ImageReqest::obtain(&clusterizer_, this);
            ImageProcessFunctor process_functor(&clusterizer_);
            process_functor();
            std::vector<TargetCandidate> const& sorted_candidates = clusterizer_.getGoodCandidatesSorted();
            if(!sorted_candidates.empty())
                handleClusterizerResults(sorted_candidates[0].py_in_cam_base_system_, request, response);
            else
                response.valid = false;
            if(request.publish_clustered_img)
                dbg_clustered_image_pub_.publish(clusterizer_.getClusteredImage()->toImageMsg());
        }
        return true;
    }

    PitchYaw getCameraPitchAndYaw() {
        srrc_cam_pitch_yaw_srv::get_cam_pitch_yaw request;
        if(get_cam_pitch_yaw_client_.call(request)) {
#if (IMG2DIR_DEBUG > 0)
            ROS_INFO("Img2dir TRACE: getCameraPitchAndYaw(%f, %f)", request.response.py.pitch, request.response.py.yaw);
#endif
            return PitchYaw(request.response.py.pitch, request.response.py.yaw);
        }
        ROS_INFO("Img2dir ERROR: coud not call srrc_cam_pitch_yaw_srv::get_cam_pitch_yaw");
        return PitchYaw();
    }

    void setCameraPitchAndYaw(PitchYaw const& py) {
        srrc_cam_pitch_yaw_srv::set_cam_pitch_yaw request;
        request.request.py.pitch = py.pitch_;
        request.request.py.yaw   = py.yaw_;
#if (IMG2DIR_DEBUG > 0)
        ROS_INFO("Img2dir TRACE: setCameraPitchAndYaw(%f, %f)", py.pitch_, py.yaw_);
#endif
        if(!set_cam_pitch_yaw_client_.call(request))
            ROS_INFO("Img2dir ERROR: coud not call srrc_cam_pitch_yaw_srv::set_cam_pitch_yaw");
    }


    template<typename t_Param>
    static void printParam(std::string const& param_name, t_Param& param, std::string const& prefix = " * ") {
        Nh::printParam(param_name, param, prefix);
    }

    template<typename t_Param>
    void updateParam(std::string const& param_name, t_Param& param) {
        nh_.updateParam(param_name, param);
    }

    void updateParams() {
      updateParam("clusterizer_color_distance_threshold", params_.colorDistanceThreshold);
      updateParam("clusterizer_black_threshold",          params_.black_threshold);
      updateParam("clusterizer_half_window_size",         params_.halfWindowSize);
      updateParam("best_cluster_max_box_size",            params_.maxBoxSizeInBestCluster);
      updateParam("best_cluster_min_num_of_pixels",       params_.minNumOfPixelsInBestCluster);
      updateParam("visualizer_method",                    params_.pixelColoringMethod);
    }

    void printParams() {
      ROS_INFO("Starting img2dir node with parameters:");
      printParam("clusterizer_color_distance_threshold", params_.colorDistanceThreshold);
      printParam("clusterizer_black_threshold",          params_.black_threshold);
      printParam("clusterizer_half_window_size",         params_.halfWindowSize);
      printParam("best_cluster_max_box_size",            params_.maxBoxSizeInBestCluster);
      printParam("best_cluster_min_num_of_pixels",       params_.minNumOfPixelsInBestCluster);
      printParam("visualizer_method",                    params_.pixelColoringMethod);
      printParam("camera_use_camera_info",               params_.use_camera_info);
    }

};

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "three_raspicams");
  img2dir::SingleCamWithMotors node;
  ros::spin();
  return 0;
}
