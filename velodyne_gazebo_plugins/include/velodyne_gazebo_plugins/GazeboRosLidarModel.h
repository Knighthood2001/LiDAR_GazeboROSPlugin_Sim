//
// Created by ou on 2020/11/12.
//

#ifndef VELODYNE_GAZEBO_PLUGINS_GAZEBOROSLIDARMODEL_H
#define VELODYNE_GAZEBO_PLUGINS_GAZEBOROSLIDARMODEL_H

//
// Created by ou on 2020/11/12.
//
#include <functional>
#include <sdf/Param.hh>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/laserscan_stamped.pb.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/PointCloud2.h>



#include <boost/algorithm/string/trim.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
using namespace std;
namespace gazebo
{
    class GazeboRosMultiLaser;
    template <typename GazeboMsgT>
    struct ConnectHelper{
        GazeboRosMultiLaser *ptr;
        void (GazeboRosMultiLaser::*fp)(
                const boost::shared_ptr<GazeboMsgT const>&,int _topic_idx);
        int topic_idx;

        void callback(const boost::shared_ptr<GazeboMsgT const>& msg_ptr) {
            (ptr->*fp)(msg_ptr, topic_idx);
        }
    };
    class GazeboRosMultiLaser : public ModelPlugin
    {
    public:
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
        void OnUpdate(){;}
        GazeboRosMultiLaser();
        ~GazeboRosMultiLaser();

    private:
        physics::ModelPtr model;
        gazebo::transport::NodePtr gazebo_node_;
        vector<gazebo::transport::SubscriberPtr> gazebo_subs_;
        void OnScan(const ConstLaserScanStampedPtr &_msg,int topic_idx);
        ConnectHelper<gazebo::msgs::LaserScanStamped> CntHelper[5];
        event::ConnectionPtr updateConnection;

    private:
        ros::Publisher ros_pub_;
        string ros_pub_topic_name_;
        ros::NodeHandle* nh_;
        ros::CallbackQueue laser_queue_;
        void ConnectCb();
        int totalLidarPoints=0;
        const uint32_t POINT_STEP = 22;
        sensor_msgs::PointCloud2 msg;
    private:
        boost::mutex lock_;
        void laserQueueThread();
        boost::thread callback_laser_queue_thread_;
    private:
        vector<gazebo::sensors::RaySensorPtr> multlaser;
        string frame_name_;
        double min_intensity_;
        double gaussian_noise_;
        double min_range_;
        double max_range_;
        string robotscope;
        static double gaussianKernel(double mu, double sigma){
            // using Box-Muller transform to generate two independent standard normally distributed normal variables
            // see wikipedia
            double U = (double)rand() / (double)RAND_MAX; // normalized uniform random variable
            double V = (double)rand() / (double)RAND_MAX; // normalized uniform random variable
            return sigma * (sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V)) + mu;
        }
    };


    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(GazeboRosMultiLaser)
}
#endif //VELODYNE_GAZEBO_PLUGINS_GAZEBOROSLIDARMODEL_H
