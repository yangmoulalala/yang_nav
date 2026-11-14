#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "parameters.hpp"
#include "zlio/odometry.hpp"
template<class T>T read(std::istream& in){T v{};in.read(reinterpret_cast<char*>(&v),sizeof(v));if(!in)throw std::runtime_error("truncated fixture");return v;}
int main(int argc,char**argv){
 if(argc<3)return 2;
 const std::string input=argv[1],output=argv[2];rclcpp::init(argc,argv);
 auto node=std::make_shared<rclcpp::Node>("zlio", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));auto params=zlio::ros::loadNodeParameters(*node);
 std::unique_ptr<zlio::Odometry> core;
 auto reset=[&](){core=std::make_unique<zlio::Odometry>(params.config,params.input_limits);core->setOutputResolution(params.output_resolution);};reset();
 std::ifstream in(input,std::ios::binary);std::ofstream out(output);out<<std::setprecision(17);char header[8];in.read(header,8);if(std::string(header,8)!="ZLIOIN01")return 3;
 unsigned long epoch=0,count=0; bool active=true;
 while(in.peek()!=EOF){char type=in.get();
  try{
   if(type=='R'){epoch=read<uint64_t>(in);reset();active=true;continue;}
   if(type=='I'){zlio::Imu a;a.time=read<int64_t>(in);for(int i=0;i<3;++i)a.angular_velocity[i]=read<double>(in);for(int i=0;i<3;++i)a.acceleration[i]=read<double>(in);if(active)core->addImu(a);}
   else if(type=='C'){uint32_t n=read<uint32_t>(in);zlio::Scan scan(n);for(auto& a:scan){a.time=read<int64_t>(in);for(int i=0;i<3;++i)a.position[i]=read<double>(in);}if(active)core->addScan(std::move(scan));}
   else throw std::runtime_error("unknown input type");
   if(!active)continue; for(auto& f:core->processAvailable()){
    auto& x=f.estimate.state;auto& q=x.pose.rotation;
    out<<"{\"epoch\":"<<epoch<<",\"stamp\":"<<zlio::seconds(x.time)<<",\"pose\":["<<x.pose.position.x()<<","<<x.pose.position.y()<<","<<x.pose.position.z()<<","<<q.x()<<","<<q.y()<<","<<q.z()<<","<<q.w()<<"],\"velocity\":["<<x.velocity.x()<<","<<x.velocity.y()<<","<<x.velocity.z()<<"],\"gyro_bias\":["<<x.gyro_bias.x()<<","<<x.gyro_bias.y()<<","<<x.gyro_bias.z()<<"],\"accel_bias\":["<<x.accel_bias.x()<<","<<x.accel_bias.y()<<","<<x.accel_bias.z()<<"],\"gravity\":["<<core->gravity().x()<<","<<core->gravity().y()<<","<<core->gravity().z()<<"],\"matches\":"<<f.matches<<",\"updated\":"<<f.lidar_updated<<"}\n";
    ++count;
   }
  }catch(const std::exception&e){std::cerr<<"input type "<<type<<", epoch "<<epoch<<": "<<e.what()<<"\n";if(epoch==0){active=false;continue;} return 4;}
 }
 std::cerr<<"Replayed "<<count<<" frames\n";rclcpp::shutdown();
}