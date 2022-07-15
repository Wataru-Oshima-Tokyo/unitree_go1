// ros dependencies
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

// from pcl library
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/ModelCoefficients.h>

#include <visualization_msgs/MarkerArray.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

class PclNode
{
private:
  ros::NodeHandle _nh;
  ros::NodeHandle _pnh;
  std::string _topic_name;

  ros::Subscriber _sub_points;
  std::string _pub_topic_name;
  std::string _target_frame;
  tf::TransformListener _tf_listener;
  //ros::Publisher _pub_transformed;
  PointCloud::Ptr _cloud_tranformed;

  //pcl::PassThrough<PointT> _pass;
  //PointCloud::Ptr _cloud_passthrough;
  //ros::Publisher _pub_passthrough;

  pcl::VoxelGrid<PointT> _voxel;
  PointCloud::Ptr _cloud_voxel;
  ros::Publisher _pub_voxel;

  pcl::SACSegmentation<PointT> _seg;
  pcl::PointIndices::Ptr _inliers;
  pcl::ModelCoefficients::Ptr _coefficients;
  pcl::ExtractIndices<PointT> _extract;
  PointCloud::Ptr _cloud_seg;
  ros::Publisher _pub_seg;

  //pcl::search::KdTree<PointT>::Ptr _tree;
  //pcl::EuclideanClusterExtraction<PointT> _ec;
  //ros::Publisher _pub_clusters;

  void cbPoints(const PointCloud::ConstPtr &msg)
  {
    try
    {
      // read message
      std::string frame_id = msg->header.frame_id;
      PointCloud::ConstPtr cloud_src = msg;

      if (_target_frame.empty() == false)
      {
        frame_id = _target_frame;
        if (pcl_ros::transformPointCloud(
                _target_frame, *msg, *_cloud_tranformed, _tf_listener) == false)
        {
          ROS_ERROR("Failed pcl_ros::transformPointCloud. target_frame = %s",
                    _target_frame.c_str());
          return;
        }
        //_pub_transformed.publish(_cloud_tranformed);
        cloud_src = _cloud_tranformed;
      }

      // src --> passthrough
      //_pass.setInputCloud(cloud_src);
      //_pass.filter(*_cloud_passthrough);
      //_pub_passthrough.publish(_cloud_passthrough);

      // passthrough --> voxel
      //_voxel.setInputCloud(_cloud_passthrough);
      //_voxel.filter(*_cloud_voxel);
      //_pub_voxel.publish(_cloud_voxel);

      //PointCloud cloud;
      //pcl::copyPointCloud(*_cloud_voxel, cloud);

      // voxel --> SAC segmentation
      _seg.setOptimizeCoefficients(true);
      _seg.setModelType(pcl::SACMODEL_PLANE);
      _seg.setMethodType(pcl::SAC_RANSAC);
      _seg.setDistanceThreshold(0.04);
      _seg.setInputCloud(cloud_src);

      _seg.segment(*_inliers, *_coefficients);

      _extract.setInputCloud(cloud_src);
      _extract.setIndices(_inliers);
      _extract.setNegative(true);
      _extract.filter(*_cloud_seg);
      _pub_seg.publish(_cloud_seg);


 //     // voxel --> clustered
 //     std::vector<pcl::PointIndices> cluster_indices;
 //     _tree->setInputCloud(_cloud_voxel);
 //     _ec.setInputCloud(_cloud_voxel);
 //     _ec.extract(cluster_indices);
 //     visualization_msgs::MarkerArray marker_array;
 //
 //     int marker_id = 0;
 //     size_t ok = 0;
 //
 //     for (std::vector<pcl::PointIndices>::const_iterator 
 //           it = cluster_indices.begin(), it_end = cluster_indices.end();
 //           it != it_end; ++it, ++marker_id)
 //     {
 //         Eigen::Vector4f min_pt, max_pt;
 //         pcl::getMinMax3D(*_cloud_voxel, *it, min_pt, max_pt);
 //         Eigen::Vector4f cluster_size = max_pt - min_pt;
 //         
 //         if (cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0)
 //         {
 //             bool is_ok = true;
 //             if (cluster_size.x() < 0.05 || cluster_size.x() > 0.5)
 //             {
 //                 is_ok = false;
 //             }
 //             if (cluster_size.y() < 0.05 || cluster_size.y() > 0.5)
 //             {
 //                 is_ok = false;
 //             }
 //             if (cluster_size.z() < 0.05 || cluster_size.z() > 0.5)
 //             {
 //                 is_ok = false;
 //             }
 //
 //             visualization_msgs::Marker marker = 
 //               makeMarker(
 //                   frame_id, 
 //                   "cluster", 
 //                   marker_id, 
 //                   min_pt, 
 //                   max_pt, 
 //                   0.0f, 1.0f, 0.0f, 0.5f);
 //             
 //             if (is_ok)
 //             {
 //                 marker.ns = "ok_cluster";
 //                 marker.color.r = 1.0f;
 //                 marker.color.g = 0.0f;
 //                 marker.color.b = 0.0f;
 //                 marker.color.a = 0.5f;
 //                 ok++;
 //             }
 //
 //             marker_array.markers.push_back(marker);
 //
 //         }
 //     }
 //
 //     if (marker_array.markers.empty() == false)
 //     {
 //         _pub_clusters.publish(marker_array);
 //     }
 //
 //     ROS_INFO("points (src: %zu, passthrough: %zu, voxelgrid: %zu, cluster: %zu, ok_cluster: %zu)", 
 //       cloud_src->size(), _cloud_passthrough->size(), _cloud_voxel->size(), cluster_indices.size(), ok);
    }
    catch (std::exception &e)
    {
      ROS_ERROR("%s", e.what());
    }
  }

 // visualization_msgs::Marker makeMarker(
 //     const std::string &frame_id, const std::string &marker_ns,
 //     int marker_id,
 //     const Eigen::Vector4f &min_pt, const Eigen::Vector4f &max_pt,
 //     float r, float g, float b, float a) const
 // {
 //   visualization_msgs::Marker marker;
 //   marker.header.frame_id = frame_id;
 //   marker.header.stamp = ros::Time::now();
 //   marker.ns = marker_ns;
 //   marker.id = marker_id;
 //   marker.type = visualization_msgs::Marker::CUBE;
 //   marker.action = visualization_msgs::Marker::ADD;
 //
 //   marker.pose.position.x = (min_pt.x() + max_pt.x()) / 2;
 //   marker.pose.position.y = (min_pt.y() + max_pt.y()) / 2;
 //   marker.pose.position.z = (min_pt.z() + max_pt.z()) / 2;
 //
 //   marker.pose.orientation.x = 0.0;
 //   marker.pose.orientation.y = 0.0;
 //   marker.pose.orientation.z = 0.0;
 //   marker.pose.orientation.w = 1.0;
 //
 //   marker.scale.x = max_pt.x() - min_pt.x();
 //   marker.scale.y = max_pt.y() - min_pt.y();
 //   marker.scale.z = max_pt.z() - min_pt.z();
 //
 //   marker.color.r = r;
 //   marker.color.g = g;
 //   marker.color.b = b;
 //   marker.color.a = a;
 //
 //   marker.lifetime = ros::Duration(0.3);
 //   return marker;
 // }

public:
  PclNode()
    : _nh()
    , _pnh("~")
  {
    this->_pnh.param("target_frame", this->_target_frame, std::string(""));
    this->_pnh.param("topic_name", this->_topic_name, std::string("/camera1/point_cloud_face"));
    this->_pub_topic_name = this->_topic_name +"/seg";
    ROS_INFO("target_frame = '%s'", this->_target_frame.c_str());
    ROS_INFO("topic_name = '%s'", this->_topic_name.c_str());
    this->_sub_points = this->_nh.subscribe(this->_topic_name, 5, &PclNode::cbPoints, this);
    //_pub_transformed = _nh.advertise<PointCloud>("/obj/cloud_transformed", 1);
    this->_cloud_tranformed.reset(new PointCloud());

    //this->_pass.setFilterFieldName("z");
    //this->_pass.setFilterLimits(0.0, 2.0);
    //this->_cloud_passthrough.reset(new PointCloud());
    //_pub_passthrough = _nh.advertise<PointCloud>("/obj/passthrough", 1);

    //this->_voxel.setLeafSize(0.025f, 0.025f, 0.025f);
    //this->_cloud_voxel.reset(new PointCloud());
    //this->_pub_voxel = this->_nh.advertise<PointCloud>("/obj/voxel", 10);

    this->_cloud_seg.reset(new PointCloud());
    this->_coefficients.reset(new pcl::ModelCoefficients);
    this->_inliers.reset(new pcl::PointIndices);
    this->_pub_seg = this->_nh.advertise<PointCloud>(this->_pub_topic_name, 10);


    //this->_tree.reset(new pcl::search::KdTree<PointT>());
    //this->_ec.setClusterTolerance(0.4);  // unit: m
    //this->_ec.setMinClusterSize(10);  // min points
    //this->_ec.setMaxClusterSize(5000);  // max points
    //this->_ec.setSearchMethod(_tree);  // use KDtree
    //_pub_clusters = _nh.advertise<visualization_msgs::MarkerArray>("/obj/clusters", 1);
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pcl_node");

  PclNode pcl_node;
  ros::spin();

  return 0;
}