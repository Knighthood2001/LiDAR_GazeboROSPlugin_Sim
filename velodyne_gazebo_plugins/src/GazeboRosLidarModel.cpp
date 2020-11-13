#include "velodyne_gazebo_plugins/GazeboRosLidarModel.h"
#include <tf/tf.h>
namespace gazebo {

    GazeboRosMultiLaser::GazeboRosMultiLaser() : nh_(NULL), gaussian_noise_(0)
    {
        ;
    }
    GazeboRosMultiLaser::~GazeboRosMultiLaser()
    {
        laser_queue_.clear();
        laser_queue_.disable();
        if (nh_) {
            nh_->shutdown();
            delete nh_;
            nh_ = NULL;
        }
        callback_laser_queue_thread_.join();
    }
    void GazeboRosMultiLaser::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
        this->model = _parent;
        string print = this->model->GetScopedName() + "::" + _sdf->Get<string>("name");
        ROS_INFO("----------");
        ROS_INFO("Model plugin loaded : %s", print.c_str());

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&GazeboRosMultiLaser::OnUpdate, this));
        /*
         * 获取传入参数
         */
        min_intensity_ = std::numeric_limits<double>::lowest();
        if (!_sdf->HasElement("min_intensity")) {
            //ROS_INFO("plugin missing <min_intensity>, defaults to no clipping");
        } else {
            min_intensity_ = _sdf->GetElement("min_intensity")->Get<double>();
        }

        if (!_sdf->HasElement("gaussianNoise")) {
            ROS_INFO("Velodyne laser plugin missing <gaussianNoise>, defaults to 0.0");
            gaussian_noise_ = 0;
        } else {
            gaussian_noise_ = _sdf->GetElement("gaussianNoise")->Get<double>();
        }

        if (!_sdf->HasElement("min_range")) {
            ROS_INFO("Velodyne laser plugin missing <min_range>, defaults to 0");
            min_range_ = 0;
        } else {
            min_range_ = _sdf->GetElement("min_range")->Get<double>();
        }

        if (!_sdf->HasElement("max_range")) {
            ROS_INFO("Velodyne laser plugin missing <max_range>, defaults to infinity");
            max_range_ = INFINITY;
        } else {
            max_range_ = _sdf->GetElement("max_range")->Get<double>();
        }

        if (!_sdf->HasElement("frameName")) {
            ROS_INFO("Velodyne laser plugin missing <frameName>, defaults to /world");
            frame_name_ = "/world";
        } else {
            frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
        }

        if (!_sdf->HasElement("topicName")) {
            ROS_INFO("missing <topicName>, defaults to /points");
            ros_pub_topic_name_ = "/points";
        } else {
            ros_pub_topic_name_ = _sdf->GetElement("topicName")->Get<string>();
        }
        /*
         * 处理雷达信息收集
         */
        robotscope = this->model->GetWorld()->Name()+"::"+ this->model->GetScopedName();
        string linkname = _sdf->GetParent()->GetElement("link")->Get<string>("name");
        string sectorname = "sensor-sector";
        for (int i = 1;; i++) {
            string sensor_name = robotscope + "::" + linkname + "::" + sectorname + '-' + to_string(i);
            auto tempSectorPtr = gazebo::sensors::get_sensor(sensor_name);
            if (tempSectorPtr) {
                ROS_INFO("%s",sensor_name.c_str());
                multlaser.push_back(std::dynamic_pointer_cast<sensors::RaySensor>(tempSectorPtr));
                multlaser[i-1]->SetActive(false);
            } else {
                ROS_INFO("have %d sectors",(int)multlaser.size());
                break;
            }
        }

        if (!_sdf->HasElement("frameName")) {
            ROS_INFO("Velodyne laser plugin missing <frameName>, defaults to /world");
            frame_name_ = "/world";
        } else {
            frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
        }

        /*
         * gazebo node
         */
        gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
        gazebo_node_->Init();
        gazebo_subs_= vector<gazebo::transport::SubscriberPtr>(multlaser.size(),transport::SubscriberPtr(NULL));

        /*
         * ros node
         */
        if (!ros::isInitialized()) {
            ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                                     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }



        nh_ = new ros::NodeHandle(/*robot_namespace*/);
        std::string topic = ros_pub_topic_name_;
        cout<<ros_pub_topic_name_<<endl;
        if (ros_pub_topic_name_ != "") {
            ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
                    topic, 1,
                    boost::bind(&GazeboRosMultiLaser::ConnectCb, this),
                    boost::bind(&GazeboRosMultiLaser::ConnectCb, this),
                    ros::VoidPtr(), &laser_queue_);
            ros_pub_ = nh_->advertise(ao);
        }

        msg.header.frame_id = frame_name_;
        msg.fields.resize(6);
        msg.fields[0].name = "x";
        msg.fields[0].offset = 0;
        msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[0].count = 1;
        msg.fields[1].name = "y";
        msg.fields[1].offset = 4;
        msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[1].count = 1;
        msg.fields[2].name = "z";
        msg.fields[2].offset = 8;
        msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[2].count = 1;
        msg.fields[3].name = "intensity";
        msg.fields[3].offset = 12;
        msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[3].count = 1;
        msg.fields[4].name = "ring";
        msg.fields[4].offset = 16;
        msg.fields[4].datatype = sensor_msgs::PointField::UINT16;
        msg.fields[4].count = 1;
        msg.fields[5].name = "time";
        msg.fields[5].offset = 18;
        msg.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
        msg.fields[5].count = 1;
        for(int i=0;i<multlaser.size();i++){
            totalLidarPoints+=multlaser[i]->VerticalRangeCount()*multlaser[i]->RayCount();
        }
        msg.data.resize(totalLidarPoints * POINT_STEP);

        /*
         * thread
         */
        callback_laser_queue_thread_ = boost::thread(
                boost::bind(&GazeboRosMultiLaser::laserQueueThread, this));
        ROS_INFO("laser plugin loaded");
    }

    void GazeboRosMultiLaser::ConnectCb() {
        boost::lock_guard<boost::mutex> lock(lock_);
        if (ros_pub_.getNumSubscribers()) {
            for(int i=0;i<multlaser.size();i++) {
                ROS_INFO("%s",multlaser[i]->Topic().c_str());
                if (!gazebo_subs_[i]) {
                    //gazebo_subs_[i] = gazebo_node_->Subscribe(multlaser[0]->Topic(), &GazeboRosMultiLaser::OnScan,this);
                    CntHelper[i] = {this, &GazeboRosMultiLaser::OnScan, i};
                    gazebo_subs_[i] = gazebo_node_->Subscribe(multlaser[i]->Topic(),
                                                              &ConnectHelper<gazebo::msgs::LaserScanStamped>::callback,
                                                              &(CntHelper[i]));
                }
                multlaser[i]->SetActive(true);
            }
        } else {
            for(int i=0;i<multlaser.size();i++){
                if (!gazebo_subs_[i]) {
                    gazebo_subs_[i] ->Unsubscribe();
                    gazebo_subs_[i].reset();
                }
                multlaser[0]->SetActive(false);
            }
        }
    }
    void GazeboRosMultiLaser::laserQueueThread()
    {
        while (nh_->ok()) {
            laser_queue_.callAvailable(ros::WallDuration(0.01));
        }
    }
    void GazeboRosMultiLaser::OnScan(ConstLaserScanStampedPtr& _msg,int _topic_idx)
    {
        int i=_topic_idx;
        const ignition::math::Angle maxAngle = multlaser[i]->AngleMax();
        const ignition::math::Angle minAngle = multlaser[i]->AngleMin();

        const double maxRange = multlaser[i]->RangeMax();
        const double minRange = multlaser[i]->RangeMin();
        const int rangeCount = multlaser[i]->RangeCount();

        const int verticalRayCount = multlaser[i]->VerticalRayCount();
        const int verticalRangeCount = multlaser[i]->VerticalRangeCount();

        const ignition::math::Angle verticalMaxAngle = multlaser[i]->VerticalAngleMax();
        const ignition::math::Angle verticalMinAngle = multlaser[i]->VerticalAngleMin();

        const double yDiff = maxAngle.Radian() - minAngle.Radian();
        const double pDiff = verticalMaxAngle.Radian() - verticalMinAngle.Radian();

        const double MIN_RANGE = std::max(min_range_, minRange);
        const double MAX_RANGE = std::min(max_range_, maxRange);
        const double MIN_INTENSITY = min_intensity_;

        int offset_merge = 0;
        int j_offset_merge=0;
        for(int i=0;i<_topic_idx;i++){
            offset_merge+=multlaser[i]->VerticalRangeCount()*multlaser[i]->RayCount();
            j_offset_merge+=multlaser[i]->VerticalRangeCount();
        }
        uint8_t *ptr = msg.data.data()+offset_merge*POINT_STEP;

        for (int i = 0; i < rangeCount; i++) {
            for (int j = 0; j < verticalRangeCount; j++) {
                double r = _msg->scan().ranges(i + j * rangeCount);
                double intensity = _msg->scan().intensities(i + j * rangeCount);
                // Ignore points that lay outside range bands or optionally, beneath a
                // minimum intensity level.
                if ((MIN_RANGE >= r) || (r >= MAX_RANGE) || (intensity < MIN_INTENSITY) ) {
                    continue;
                }

                // Noise
                if (gaussian_noise_ != 0.0) {
                    r += gaussianKernel(0,gaussian_noise_);
                }

                // Get angles of ray to get xyz for point
                double yAngle;
                double pAngle;

                if (rangeCount > 1) {
                    yAngle = i * yDiff / (rangeCount -1) + minAngle.Radian();
                } else {
                    yAngle = minAngle.Radian();
                }

                if (verticalRayCount > 1) {
                    pAngle = j * pDiff / (verticalRangeCount -1) + verticalMinAngle.Radian();
                } else {
                    pAngle = verticalMinAngle.Radian();
                }

                // pAngle is rotated by yAngle:
                if ((MIN_RANGE < r) && (r < MAX_RANGE)) {
                    *((float*)(ptr + 0)) = r * cos(pAngle) * cos(yAngle);
                    *((float*)(ptr + 4)) = r * cos(pAngle) * sin(yAngle);
                    *((float*)(ptr + 8)) = r * sin(pAngle);
                    *((float*)(ptr + 12)) = intensity;
                    *((uint16_t*)(ptr + 16)) = j+j_offset_merge; // ring
                    *((float*)(ptr + 18)) = 0.0; // time
                    ptr += POINT_STEP;
                }
            }
        }

        if(_topic_idx==multlaser.size()-1)
        {
            // Populate message with number of valid points
            msg.point_step = POINT_STEP;
            msg.row_step = ptr - msg.data.data();
            msg.height = 1;
            msg.width = msg.row_step / POINT_STEP;
            msg.is_bigendian = false;
            msg.is_dense = true;
            msg.data.resize(msg.row_step); // Shrink to actual size

            // Publish output
            msg.header.stamp=ros::Time(_msg->time().sec(), _msg->time().nsec());
            ros_pub_.publish(msg);
        }
    }
}