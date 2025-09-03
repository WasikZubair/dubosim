// Copyright (c) 2009, Willow Garage, Inc.
// Modified for Dubosim

#include "dubosim/turtle_frame.hpp"

#include <QPointF>
#include <QPainter>
#include <QPixmap>
#include <QTimer>
#include <cstdlib>
#include <ctime>
#include <string>

#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include "turtlesim_msgs/srv/kill.hpp"
#include "turtlesim_msgs/srv/spawn.hpp"

#define DEFAULT_BG_R 0x45
#define DEFAULT_BG_G 0x56
#define DEFAULT_BG_B 0xff

namespace turtlesim
{

TurtleFrame::TurtleFrame(rclcpp::Node::SharedPtr & node_handle, QWidget * parent, Qt::WindowFlags f)
: QFrame(parent, f)
  , path_image_(500, 500, QImage::Format_ARGB32)
  , path_painter_(&path_image_)
  , frame_count_(0)
  , id_counter_(0)
{
  setFixedSize(500, 500);
  setWindowTitle("Dubosim");

  srand(static_cast<unsigned int>(time(NULL)));

  // Timer for updates
  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();
  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  nh_ = node_handle;

  // --- Parameters for background color ---
  nh_->declare_parameter("background_r", DEFAULT_BG_R);
  nh_->declare_parameter("background_g", DEFAULT_BG_G);
  nh_->declare_parameter("background_b", DEFAULT_BG_B);

  // --- Load robot image ---
  QString images_path = ament_index_cpp::get_package_share_directory("dubosim").c_str();
  images_path += "/images/";
  QImage robot_img;
  if (!robot_img.load(images_path + "turtle.png")) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to load turtle.png!");
  }
  turtle_images_.append(robot_img);

  meter_ = turtle_images_[0].height();

  // --- Clear canvas ---
  clear();

  // --- Services ---
  clear_srv_ = nh_->create_service<std_srvs::srv::Empty>(
      "clear", std::bind(&TurtleFrame::clearCallback, this, std::placeholders::_1, std::placeholders::_2));
  reset_srv_ = nh_->create_service<std_srvs::srv::Empty>(
      "reset", std::bind(&TurtleFrame::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
  spawn_srv_ = nh_->create_service<turtlesim_msgs::srv::Spawn>(
      "spawn", std::bind(&TurtleFrame::spawnCallback, this, std::placeholders::_1, std::placeholders::_2));
  kill_srv_ = nh_->create_service<turtlesim_msgs::srv::Kill>(
      "kill", std::bind(&TurtleFrame::killCallback, this, std::placeholders::_1, std::placeholders::_2));

  // --- Parameter event subscriber ---
  rclcpp::QoS qos(rclcpp::KeepLast(100), rmw_qos_profile_sensor_data);
  parameter_event_sub_ = nh_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", qos,
    std::bind(&TurtleFrame::parameterEventCallback, this, std::placeholders::_1));

  RCLCPP_INFO(nh_->get_logger(), "Starting Dubosim with node name %s", nh_->get_fully_qualified_name());

  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;

  // --- Spawn initial turtle ---
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
}

TurtleFrame::~TurtleFrame()
{
  delete update_timer_;
}

std::string TurtleFrame::spawnTurtle(const std::string & name, float x, float y, float angle)
{
  std::string real_name = name;
  if (real_name.empty()) {
    do {
      std::stringstream ss;
      ss << "turtle" << ++id_counter_;
      real_name = ss.str();
    } while (hasTurtle(real_name));
  } else {
    if (hasTurtle(real_name)) {
      return "";
    }
  }

  TurtlePtr t = std::make_shared<Turtle>(
      nh_, real_name, turtle_images_[0], QPointF(x, height_in_meters_ - y), angle);
  turtles_[real_name] = t;
  update();

  RCLCPP_INFO(nh_->get_logger(), "Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]",
              real_name.c_str(), x, y, angle);

  return real_name;
}

bool TurtleFrame::hasTurtle(const std::string & name)
{
  return turtles_.find(name) != turtles_.end();
}

void TurtleFrame::onUpdate()
{
  if (!rclcpp::ok()) {
    close();
    return;
  }

  rclcpp::spin_some(nh_);
  updateTurtles();
}

void TurtleFrame::updateTurtles()
{
  if (last_turtle_update_.nanoseconds() == 0) {
    last_turtle_update_ = nh_->now();
    return;
  }

  bool modified = false;
  for (auto & pair : turtles_) {
    modified |= pair.second->update(0.001 * update_timer_->interval(),
                                    path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }

  if (modified) update();
  ++frame_count_;
}

void TurtleFrame::paintEvent(QPaintEvent *)
{
  QPainter painter(this);

  // --- Draw background ---
  QString images_path = ament_index_cpp::get_package_share_directory("dubosim").c_str();
  images_path += "/images/";
  QPixmap bg;
  if (!bg.load(images_path + "background.png")) {
    RCLCPP_WARN(nh_->get_logger(), "Failed to load background.png, using default color");
    int r = DEFAULT_BG_R, g = DEFAULT_BG_G, b = DEFAULT_BG_B;
    nh_->get_parameter("background_r", r);
    nh_->get_parameter("background_g", g);
    nh_->get_parameter("background_b", b);
    painter.fillRect(rect(), QColor(r, g, b));
  } else {
    painter.drawPixmap(0, 0, width(), height(), bg);
  }

  // --- Draw path and turtles ---
  painter.drawImage(QPoint(0, 0), path_image_);
  for (auto & pair : turtles_) {
    pair.second->paint(painter);
  }
}

void TurtleFrame::clear()
{
  path_image_.fill(qRgba(255, 255, 255, 0));
  update();
}

bool TurtleFrame::clearCallback(const std_srvs::srv::Empty::Request::SharedPtr,
                                std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Clearing Dubosim.");
  clear();
  return true;
}

bool TurtleFrame::resetCallback(const std_srvs::srv::Empty::Request::SharedPtr,
                                std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Resetting Dubosim.");
  turtles_.clear();
  id_counter_ = 0;
  spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}

bool TurtleFrame::spawnCallback(const turtlesim_msgs::srv::Spawn::Request::SharedPtr req,
                                turtlesim_msgs::srv::Spawn::Response::SharedPtr res)
{
  std::string name = spawnTurtle(req->name, req->x, req->y, req->theta);
  if (name.empty()) {
    RCLCPP_ERROR(nh_->get_logger(), "A turtle named [%s] already exists", req->name.c_str());
    return false;
  }

  res->name = name;
  return true;
}

bool TurtleFrame::killCallback(const turtlesim_msgs::srv::Kill::Request::SharedPtr req,
                               turtlesim_msgs::srv::Kill::Response::SharedPtr)
{
  auto it = turtles_.find(req->name);
  if (it == turtles_.end()) {
    RCLCPP_ERROR(nh_->get_logger(), "Tried to kill turtle [%s], which does not exist", req->name.c_str());
    return false;
  }

  turtles_.erase(it);
  update();
  return true;
}

void TurtleFrame::parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
{
  if (event->node == nh_->get_fully_qualified_name()) {
    update();
  }
}

}  // namespace turtlesim

