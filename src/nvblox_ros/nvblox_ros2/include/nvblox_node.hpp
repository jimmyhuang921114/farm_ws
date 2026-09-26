// ros2 msg 
#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/ocupancy_grid.hpp>
#include <


// other lib
#include <mutex>
#include <vector>
#include <string>
#include <map>




namespace nvblox{

    class NvbloxNode :public rclcpp::Node
    {
        public:
            void initializeMultiMapper();
            void subscribeToTopics();
            void advertiseTopics();
            void advertiseServices();
            void setupTimers();
            using ImageSegmentationMaskMsgTuple = 
            std



            """sensor callback"""
            // segment image callback
            void SegmentImageCallback(
                const sensor_msgs::msg::Image::ConstSharedPtr & segment_image,
                const sensor_msgs::msg::Image::ConstSharedPtr & segment_image_info
            );
            
            // depth image callback
            void DepthImageCallback(
                const sensor_msgs::msgs::Image::ConstSharedPtr & depth_image,
                const sensor_msgs::msgs::CameraInfo::ConstSharedPtr & depth_camera_info
            );
            
            // color image callback
            void ColorImageCallback(
                const sensor_msgs::msgs::Image::ConstSharedPtr & color_camera_image,
                const sensor_msgs::msgs::Image::ConstSharedPtr & color_camera_info,
            );

            // pointcloud callback
            void PointCloudeCallback(
                const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud
            );


            //
            void getEsdfAndGradientService(
                const std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Request> request,
                std::shared_ptr<nvblox_msgs::srv::EsdfAndGradients::Response> response
            );
            
            virtual void tick();
            // Publish data on fiexd frequency
            void publishLayers();
            // PUblish debug visualizerfor rviz2
            void publishDebugVisualizations();

            void 

    }
}