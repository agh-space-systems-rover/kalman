#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Dense>
#include <Eigen/Geometry> // for Quaternionf etc.

class PlaneFilterNode : public rclcpp::Node
{
public:
  PlaneFilterNode()
  : Node("plane_filter_node")
  {
    // Declare parameters
    distance_threshold_ = this->declare_parameter<double>("distance_threshold", 0.01);
    eps_angle_deg_      = this->declare_parameter<double>("eps_angle_deg", 10.0);
    keep_above_         = this->declare_parameter<bool>("keep_above", true);

    // We'll assume "up" is Z
    axis_ = Eigen::Vector3f(0.0f, 0.0f, 1.0f);

    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "input_cloud", rclcpp::SensorDataQoS(),
        std::bind(&PlaneFilterNode::cloudCallback, this, std::placeholders::_1));

    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);

    // Publisher for a visualization marker
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("plane_marker", 10);

    RCLCPP_INFO(this->get_logger(), "PlaneFilterNode started with plane visualization.");
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_cloud);

    if (input_cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty cloud, skipping...");
      return;
    }

    // 1) Segment the plane with RANSAC + orientation constraint
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(2000);
    seg.setDistanceThreshold(distance_threshold_);

    seg.setAxis(axis_);
    float eps_angle_rad = static_cast<float>(eps_angle_deg_ * M_PI / 180.0);
    seg.setEpsAngle(eps_angle_rad);

    seg.setInputCloud(input_cloud);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
      RCLCPP_WARN(this->get_logger(), "No plane found!");
      return;
    }
    if (coefficients->values.size() < 4) {
      RCLCPP_ERROR(this->get_logger(), "Not enough plane coefficients!");
      return;
    }

    float a = coefficients->values[0];
    float b = coefficients->values[1];
    float c = coefficients->values[2];
    float d = coefficients->values[3];

    // 2) Filter points above (or below) the plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->header = input_cloud->header;

    for (const auto &pt : input_cloud->points) {
      float dist = a * pt.x + b * pt.y + c * pt.z + d; // Signed dist * ||n||
      bool is_above = (dist >= 0.15f);
      if ((keep_above_ && is_above) || (!keep_above_ && !is_above)) {
        filtered_cloud->push_back(pt);
      }
    }

    // Added just now
    filtered_cloud = normalizePointCloudRotation(a, b, c, d, filtered_cloud);

    // Convert back to ROS and publish the filtered cloud
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*filtered_cloud, out_msg);
    sensor_msgs::msg::PointCloud2::_header_type header = msg->header;
    out_msg.header = msg->header;
    pc_pub_->publish(out_msg);

    // 3) Publish the plane as a marker in RViz
    publishPlaneMarker(a, b, c, d, msg->header);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr normalizePointCloudRotation(float a, float b, float c, float d, const pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud){
    Eigen::Vector3f normal(a, b, c);
    normal.normalize(); // Now normal has unit length
    Eigen::Vector3f zAxis(0.0f, 0.0f, 1.0f);
    float angle = acos(normal.dot(zAxis)); // angle in radians
    Eigen::Vector3f rotationAxis = normal.cross(zAxis);

    if (rotationAxis.norm() > 1e-6) {
        rotationAxis.normalize();
    } else {
        // normal is already parallel to z-axis:
        // angle = 0 or M_PI; rotationAxis can be any valid perpendicular
        RCLCPP_WARN(this->get_logger(), "Scan was already normalized");
        return filtered_cloud;
    }

    Eigen::AngleAxisf rotation(angle, rotationAxis);
    Eigen::Matrix3f rotationMatrix = rotation.toRotationMatrix();

    // original normal is (a, b, c) unnormalized:
    float normSq = a*a + b*b + c*c;  // squared magnitude of (a,b,c)
    Eigen::Vector3f planePoint = -(d / normSq) * Eigen::Vector3f(a, b, c);

    // Rotate the chosen plane point
    Eigen::Vector3f rotatedPoint = rotationMatrix * planePoint;

    // We want rotatedPoint.z() == 0, so define a translation
    Eigen::Vector3f translationVec(0.0f, 0.0f, -rotatedPoint.z());

    // Build a 4x4 transform
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.linear() = rotationMatrix;
    transform.translation() = translationVec;

    // Now apply to your PCL point cloud:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);  // output
    pcl::transformPointCloud(*filtered_cloud, *cloud_out, transform);

    return cloud_out;
  }

  // Create and publish a Marker representing the plane
  void publishPlaneMarker(float a, float b, float c, float d,
                          const std_msgs::msg::Header & cloud_header)
  {
    // The plane eq is: a*x + b*y + c*z + d = 0
    // Normal vector
    Eigen::Vector3f normal(a, b, c);
    float norm_length = normal.norm();
    if (norm_length < 1e-6f) {
      RCLCPP_WARN(this->get_logger(), "Plane normal is too small to create marker.");
      return;
    }
    // Normalized
    Eigen::Vector3f normal_norm = normal / norm_length;

    // The plane's distance from origin is -d / norm_length
    // The closest point on plane to origin is that distance along normal_norm
    float plane_dist = -d / norm_length;
    Eigen::Vector3f plane_center = plane_dist * normal_norm; // center of plane marker

    // Construct orientation: we want Z-axis of the marker to align with the plane normal
    // That means we rotate from (0,0,1) to normal_norm
    Eigen::Vector3f z_axis(0.0f, 0.0f, 1.0f);
    // If normal_norm is nearly the same or opposite as z_axis, handle special cases
    // Otherwise, use the "fromTwoVectors" approach
    Eigen::Quaternionf orientation;
    if (fabs(z_axis.dot(normal_norm)) > 0.9999f) {
      // Aligned or anti-aligned
      // If they are almost identical
      if (z_axis.dot(normal_norm) > 0.0f) {
        orientation = Eigen::Quaternionf::Identity();
      } else {
        // 180 deg flip about X (or any axis perpendicular to z)
        orientation = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX());
      }
    } else {
      // General case
      Eigen::Vector3f axis = z_axis.cross(normal_norm);
      float angle = acos(z_axis.dot(normal_norm));
      axis.normalize();
      orientation = Eigen::AngleAxisf(angle, axis);
    }

    // Build the ROS Marker
    visualization_msgs::msg::Marker marker;
    marker.header = cloud_header; // same frame_id, same stamp for convenience
    marker.ns = "plane_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Pose
    marker.pose.position.x = plane_center.x();
    marker.pose.position.y = plane_center.y();
    marker.pose.position.z = plane_center.z();

    // Convert Eigen quaternion to geometry_msgs
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();

    // Scale: X and Y define the "extent" of the plane, Z is thickness
    marker.scale.x = 5.0;  // 5 meters in X
    marker.scale.y = 5.0;  // 5 meters in Y
    marker.scale.z = 0.01; // very thin

    // Color, e.g. semi-transparent cyan
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.5f; // 50% transparent

    marker.lifetime = rclcpp::Duration(0,0); // 0 => forever
    marker.frame_locked = false;

    // Publish
    marker_pub_->publish(marker);
  }

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // Parameters
  double distance_threshold_;
  double eps_angle_deg_;
  bool keep_above_;
  Eigen::Vector3f axis_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaneFilterNode>());
  rclcpp::shutdown();
  return 0;
}
