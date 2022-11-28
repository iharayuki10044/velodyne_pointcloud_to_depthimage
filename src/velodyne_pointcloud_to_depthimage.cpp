#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

class VelodynePointcloudToDepthimage
{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscriber*/
		ros::Subscriber _sub_pc;

		ros::Subscriber _sub_rgb;

		/*publisher*/
		ros::Publisher _pub_img_64f;
		ros::Publisher _pub_img_16u;
		ros::Publisher _pub_img_8u;
		ros::Publisher _pub_img_front;
		ros::Publisher _pub_img_rgbd;

		/*image*/
		cv::Mat _img_cv_64f;
		cv::Mat _img_cv_16u;
		cv::Mat _img_cv_8u;
		cv::Mat _img_cv_front;
		cv::Mat _img_sub_rgb;
		cv::Mat _img_pub_rgbd;

		cv::Mat _img_sub_r;
		cv::Mat _img_sub_g;
		cv::Mat _img_sub_b;
		cv::Mat _img_sub_a;

		cv::Mat _img_sub_d;

		/*point cloud*/
		std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _rings;
		/*counter*/
		int _save_counter = 0;
		/*parameter*/
		int _num_ring;
		int _points_per_ring;
		double _depth_resolution;
		double _max_range;
		int _save_limit;

		int _cliped_points_x;
		int _cliped_width;

		double _field_of_view;

		std::string _save_root_path;
		std::string _save_img_name;

		bool is_velodyne_ok = false;
		bool is_rgb_ok = false;
		bool is_depthimage_show = false;
		bool is_depthimage_save = false;
		int depthimage_show_rate = 1;

		int pub_rgbd_width = 0;
		int pub_rgbd_height = 0;

	public:
		VelodynePointcloudToDepthimage();
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);

		void callbackRGB(const sensor_msgs::CompressedImageConstPtr& msg);

		void pcToRings(const sensor_msgs::PointCloud2& pc_msg);
		void ringsToImage(void);
		void publication(std_msgs::Header header);

		void process();

};

VelodynePointcloudToDepthimage::VelodynePointcloudToDepthimage()
	: _nhPrivate("~")
{
	std::cout << "--- velodyne_pointcloud_to_depthimage ---" << std::endl;
	/*parameter*/
	_nhPrivate.param("num_ring", _num_ring, 32);
	_nhPrivate.param("points_per_ring", _points_per_ring, 1092);
	_nhPrivate.param("depth_resolution", _depth_resolution, 0.01);
	_nhPrivate.param("max_range", _max_range, 100.0);
	_nhPrivate.param("save_limit", _save_limit, -1);
	_nhPrivate.param("field_of_view", _field_of_view, 90.0);
	_nhPrivate.param("save_root_path", _save_root_path, std::string("saved"));
	_nhPrivate.param("save_img_name", _save_img_name, std::string("depth_"));
	_nhPrivate.param("is_depthimage_show", is_depthimage_show, false);
	_nhPrivate.param("is_depthimage_save", is_depthimage_save, false);	
	_nhPrivate.param("depthimage_show_rate", depthimage_show_rate, 1);
	_nhPrivate.param("pub_rgbd_width", pub_rgbd_width, 1280);
	_nhPrivate.param("pub_rgbd_height", pub_rgbd_height, 720);

	std::cout << "_num_ring = " << _num_ring << std::endl;
	std::cout << "_points_per_ring = " << _points_per_ring << std::endl;
	std::cout << "_depth_resolution = " << _depth_resolution << std::endl;
	std::cout << "_max_range = " << _max_range << std::endl;
	std::cout << "_save_limit = " << _save_limit << std::endl;
	std::cout << "_field_of_view = " << _field_of_view << std::endl;
	std::cout << "_save_root_path = " << _save_root_path << std::endl;
	std::cout << "_save_img_name = " << _save_img_name << std::endl;
	std::cout << "is_depthimage_show = " << is_depthimage_show << std::endl;
	std::cout << "is_depthimage_save = " << is_depthimage_save << std::endl;
	std::cout << "depthimage_show_rate = " << depthimage_show_rate << std::endl;

	is_velodyne_ok = false;
	is_rgb_ok = false;

	/*sub*/
	_sub_pc = _nh.subscribe("/velodyne_points", 1, &VelodynePointcloudToDepthimage::callbackPC, this);

	/*pub*/
	_pub_img_64f = _nh.advertise<sensor_msgs::Image>("/depth_image/64fc1", 1);
	_pub_img_16u = _nh.advertise<sensor_msgs::Image>("/depth_image/16uc1", 1);
	_pub_img_8u = _nh.advertise<sensor_msgs::Image>("/depth_image/8uc1", 1);

	_pub_img_front = _nh.advertise<sensor_msgs::Image>("/depth_image/front", 1);
	_pub_img_rgbd = _nh.advertise<sensor_msgs::Image>("/depth_image/rgbd", 1);

	/*initialize*/
	_rings.resize(_num_ring);
	for(size_t i=0 ; i<_rings.size() ; ++i){
		pcl::PointCloud<pcl::PointXYZI>::Ptr tmp (new pcl::PointCloud<pcl::PointXYZI>);
		_rings[i] = tmp;
	}

	// width height channel 
	// _img_pub_rgbd = cv::Mat::zeros(720, 1280, CV_8UC4);

	_cliped_points_x =  _points_per_ring * (180 - _field_of_view/2) /360.0; 
	_cliped_width = _points_per_ring * _field_of_view /360.0;

}

void VelodynePointcloudToDepthimage::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{

	for(size_t i=0 ; i<_rings.size() ; ++i){
		_rings[i]->points.clear();
	}
	pcToRings(*msg);
	ringsToImage();
	publication(msg->header);
}

void VelodynePointcloudToDepthimage::pcToRings(const sensor_msgs::PointCloud2& pc_msg)
{
	sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(pc_msg,"ring");
	sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc_msg,"x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc_msg,"y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc_msg,"z");
	sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(pc_msg,"intensity");

	for( ; iter_ring!=iter_ring.end() ; ++iter_ring, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity){
		pcl::PointXYZI tmp;
		tmp.x = *iter_x;
		tmp.y = *iter_y;
		tmp.z = *iter_z;
		tmp.intensity = *iter_intensity;
		_rings[*iter_ring]->points.push_back(tmp);
	}
}

void VelodynePointcloudToDepthimage::ringsToImage(void)
{
	/*reset*/
	// _img_cv_64f = cv::Mat::zeros(_num_ring, _points_per_ring, CV_64FC1);
	// _img_cv_64f = cv::Mat(_num_ring, _points_per_ring, CV_64FC1, cv::Scalar(-1));
	// _img_cv_64f = cv::Mat(_num_ring, _points_per_ring, CV_64FC1, cv::Scalar(1.797693134862315e+308));

	_img_cv_8u = cv::Mat(_num_ring, _points_per_ring, CV_8UC1, cv::Scalar(255));

	/*input*/
	double angle_resolution = 2*M_PI/(double)_points_per_ring;

	// std::cout << "ring seize : " << _rings.size() << std::endl;
	// std::cout << "col size : " << _rings[0]->size() << std::endl;

	for(size_t i=0 ; i<_rings.size() ; ++i){
		int row = (int)_rings.size() - i - 1;

		double max_distance = 100000;
		double pixel;
		int past_col;

		for(size_t j=0 ; j<_rings[i]->points.size() ; ++j){

			if((std::isnan(_rings[i]->points[j].x) || std::isnan(_rings[i]->points[j].y)) == true){

				if( (i+1 <= _rings.size()) && (i-1 >= 0) ){
					pixel = (pixel + sqrt(_rings[i]->points[j].x*_rings[i]->points[j].x + _rings[i]->points[j].y*_rings[i]->points[j].y)) /2;
					_img_cv_8u.at<uint8_t>(row, past_col) = (uint8_t)(pixel);											
				}
				else{
					_img_cv_8u.at<uint8_t>(row, past_col +1) = pixel;
				}
			}else{
				double angle = atan2(_rings[i]->points[j].y, _rings[i]->points[j].x);
				int col = _points_per_ring - (int)((angle + M_PI)/angle_resolution) - 1;
				past_col = col;	
				// _img_cv_64f.at<double>(row, col) = sqrt(_rings[i]->points[j].x*_rings[i]->points[j].x + _rings[i]->points[j].y*_rings[i]->points[j].y);

				// _img_cv_8u.at<u_int8_t> (row, col) = (u_int8_t)(sqrt(_rings[i]->points[j].x*_rings[i]->points[j].x + _rings[i]->points[j].y*_rings[i]->points[j].y));
					
				pixel = sqrt(_rings[i]->points[j].x*_rings[i]->points[j].x + _rings[i]->points[j].y*_rings[i]->points[j].y);
				if(pixel > 255){
					// std::cout << "infinity" << std::endl;
					_img_cv_8u.at<uint8_t>(row, col) = 255;
				}
				else{
					_img_cv_8u.at<u_int8_t> (row, col) = (u_int8_t) pixel;
					pixel = pixel /_max_range *255;
				}
				
			}
		}
		if(i == _rings.size() -1){	
			// std::cout << i << " ban "  << "max distance : " << max_distance << std::endl;
		}	
	}
	/*convert*/
	// _img_cv_8u.convertTo(_img_cv_16u, CV_16UC1, 1/_depth_resolution, 0);
	// _img_cv_8u.convertTo(_img_cv_16u, CV_16UC1, 1/_depth_resolution, 0);
	// _img_cv_64f.convertTo(_img_cv_8u, CV_8UC1, 255/_max_range, 0);

	// std::cout << "------------------" << std::endl;
	// std::cout << "image size 64 " << _img_cv_64f.size() << std::endl;
	// std::cout << "image type " << _img_cv_64f.type() << std::endl;
	// std::cout << "image size 16 " << _img_cv_16u.size() << std::endl;
	// std::cout << "image type " << _img_cv_16u.type() << std::endl;

	// std::cout << "image size 8 " << _img_cv_8u.size() << std::endl;
	// std::cout << "image type " << _img_cv_8u.type() << std::endl;
	// _img_cv_8u.resize(720, 1280);
	// std::cout << "image big size 8 " << _img_cv_8u.size() << std::endl;

	// std::cout << "clip x : " << _cliped_points_x << std::endl;
	// std::cout << "clip width : " << _cliped_width << std::endl;

	cv::Mat output;
	output = cv::Mat(_img_cv_8u, cv::Rect(_cliped_points_x, 0, _cliped_width, _num_ring));

	// show sub d
	// for(int i =0; output.rows > i; i++){
	// 	for(int j=0; output.cols > j; j++){
	// 		std::cout << (int)output.at<uint8_t>(i, j) << " ";
	// 	}
	// 	std::cout << "" <<std::endl;	
	// }


	cv::resize(output, _img_sub_d, cv::Size(pub_rgbd_width, pub_rgbd_height), 0, 0, cv::INTER_NEAREST);

	// std::cout << "img resize size : " << _img_sub_d.size() << std::endl;

	// std::cout << "------------------" << std::endl;

	_save_counter++;
	// if((_save_counter % depthimage_show_rate == 0)&&(is_depthimage_save)){
	// 	std::string filename = "/home/amsl/rgbd_test/front_depth_" + std::to_string(_save_counter) + ".png";
	// 	std::string filename_resize = "/home/amsl/rgbd_test/front_depth_resize" + std::to_string(_save_counter) + ".png";
	// 	cv::imwrite(filename, output);
	// }

	// if((_save_counter % depthimage_show_rate == 0)&&(is_depthimage_show)){
	// 	cv::imshow("depth_moto", output);
	// 	cv::imshow("depth_resize", _img_sub_d);
	// 	cv::waitKey(2000);
	// }


	is_velodyne_ok = true;

	// std::cout << "save limit " << _save_limit << std::endl;
	// std::cout << "save count " << _save_counter << std::endl;
	// std::cout << "------------------" << std::endl;

}

void VelodynePointcloudToDepthimage::publication(std_msgs::Header header)
{
	sensor_msgs::ImagePtr img_ros_front = cv_bridge::CvImage(header, "mono8", _img_sub_d).toImageMsg();
	_pub_img_front.publish(img_ros_front);

}

void VelodynePointcloudToDepthimage::callbackRGB(const sensor_msgs::CompressedImageConstPtr &msg)
{
	
	std::cout <<" callbackRGB " << std::endl;

	try {
		_img_sub_rgb = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA16)->image;

		// std::cout << "rgb image type : " << _img_sub_rgb.type() << std::endl;	
		// std::cout << "rgb image size : " << _img_sub_rgb.size() << std::endl;		
		// std::cout << "rgb image channels : " << _img_sub_rgb.channels() << std::endl;
		// std::cout << "rgb image width : " << _img_sub_rgb.cols << std::endl;
		// std::cout << "rgb image depth : " << _img_sub_rgb.depth() << std::endl;
		// std::cout << "rgb image at 0,0 : " << (int)_img_sub_rgb.at<unsigned char>(0, 0) << std::endl;

		// cv::Mat output;
		// output = cv::Mat(_img_sub_rgb, cv::Rect(240, 120, 120, 240));

		// _save_counter++;
		// if(_save_counter %20 == 1){
		// 	std::string save_file = "/home/amsl/rgbd_test/test" + std::to_string(_save_counter) + ".png";
		// 	cv::imwrite(save_file, output);
		// }

		// channel convetrt 4 to 5	
		// _img_sub_rgb = _img_sub_rgb.reshape(5, 0);

		// _img_pub_rgbd = _img_sub_rgb.clone();

		// int width = _img_sub_rgb.cols;
		// int heght = _img_sub_rgb.rows;

		// _img_sub_r.zeros(width, heght, CV_8UC1);
		// _img_sub_g.zeros(width, heght, CV_8UC1);
		// _img_sub_b.zeros(width, heght, CV_8UC1);
		// _img_sub_a.zeros(width, heght, CV_8UC1);

		std::vector<cv::Mat> _img_sub_rgb_channels;
		cv::split(_img_sub_rgb, _img_sub_rgb_channels);
		_img_sub_g = _img_sub_rgb_channels[0];
		_img_sub_r = _img_sub_rgb_channels[1];
		_img_sub_b = _img_sub_rgb_channels[2];
		_img_sub_a = _img_sub_rgb_channels[3];

		// std::cout << "chnanel num : " << _img_sub_rgb_channels.size() << std::endl;
		// std::cout << "converted rgbD image size : " << _img_pub_rgbd.size() << std::endl;
		// std::cout << "converted rgbD image channels : " << _img_pub_rgbd.channels() << std::endl;

		is_rgb_ok = true;	
	}	
	catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}

	// std::cout << "img size b : " << _img_sub_b.size() << std::endl;
	// std::cout << "img size g : " << _img_sub_g.size() << std::endl;
	// std::cout << "img size r : " << _img_sub_r.size() << std::endl;
	// std::cout << "img size a : " << _img_sub_a.size() << std::endl;
	// std::cout << "img size d : " << _img_sub_d.size() << std::endl;

	// std::cout << "img channel b : " << _img_sub_b.channels() << std::endl;
	// std::cout << "img channel g : " << _img_sub_g.channels() << std::endl;
	// std::cout << "img channel r : " << _img_sub_r.channels() << std::endl;
	// std::cout << "img channel a : " << _img_sub_a.channels() << std::endl;
	
	// std::cout << "img type b : " << _img_sub_b.type() << std::endl;
	// std::cout << "img type g : " << _img_sub_g.type() << std::endl;
	// std::cout << "img type r : " << _img_sub_r.type() << std::endl;
	// std::cout << "img type a : " << _img_sub_a.type() << std::endl;
	// std::cout << "img type d : " << _img_sub_d.type() << std::endl;

	_img_sub_d.convertTo(_img_sub_d, CV_16UC1, 1/_depth_resolution, 0);

	if(is_velodyne_ok){	

		cv::Mat output;
		
		std::vector<cv::Mat> _img_combine;
		_img_combine.push_back(_img_sub_b);
		_img_combine.push_back(_img_sub_g);
		_img_combine.push_back(_img_sub_r);
		_img_combine.push_back(_img_sub_d);
		// _img_combine.push_back(_img_sub_a);
		cv::merge(_img_combine, output);

		std::cout << "output size : " << output.size() << std::endl;
		std::cout << "output channels : " << output.channels() << std::endl;
		std::cout << "output type : " << output.type() << std::endl;

		// cv::merge(std::vector<cv::Mat>{_img_sub_g, _img_sub_r, _img_sub_b, _img_sub_d, _img_sub_a}, _img_pub_rgbd);
		
		sensor_msgs::ImagePtr img_ros_rgbd = cv_bridge::CvImage(msg->header, "rgbd16", output).toImageMsg();
		_pub_img_rgbd.publish(img_ros_rgbd);
		is_velodyne_ok = false;	
		ROS_INFO("publish rgbd image");	
	}

}

void VelodynePointcloudToDepthimage::process()
{
	ros::Rate loop_rate(10);

	while(ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodyne_pointcloud_to_depthimage");
	
	VelodynePointcloudToDepthimage velodyne_pointcloud_to_depthimage;
	velodyne_pointcloud_to_depthimage.process();

	ros::spin();
}
