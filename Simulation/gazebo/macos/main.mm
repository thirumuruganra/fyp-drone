// Native app identity makes the Gazebo window accessible from Dock and app switcher.
#import <Cocoa/Cocoa.h>
#include <cstdlib>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <iostream>
#include <functional>
#include <cmath>
#include <csignal>
#include <gz/sim/gui/Gui.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <gz/msgs/cameratrack.pb.h>
#include <gz/msgs/gui_camera.pb.h>
#include <gz/msgs/pose_v.pb.h>

// Keep both camera position-follow and look-at attached for the whole session.
// Gazebo releases tracking on Escape / selection changes; recording mode restores it.
static void TrackDrone(const std::string &target, double x, double y, double z,
    bool takeoff, const std::string &world, double flightX, double flightY,
    double flightZ, double switchAltitude, int parentPid = 0)
{
  std::atomic<double> altitude{0.0};
  std::atomic<bool> trackCorrect{false}, followCorrect{false};
  const double fx = takeoff ? flightX : x;
  const double fy = takeoff ? flightY : y;
  const double fz = takeoff ? flightZ : z;
  gz::transport::Node node;
  auto alive = [parentPid]() {
    return parentPid == 0 || kill(parentPid, 0) == 0;
  };
  std::function<void(const gz::msgs::CameraTrack &)> statusCallback =
    [&](const gz::msgs::CameraTrack &status) {
      trackCorrect.store(status.track_mode() == gz::msgs::CameraTrack::TRACK &&
                         status.track_target().name() == target);
      const auto &offset = status.follow_offset();
      followCorrect.store(status.track_mode() == gz::msgs::CameraTrack::FOLLOW &&
          status.follow_target().name() == target &&
          std::abs(offset.x() - fx) < 0.01 && std::abs(offset.y() - fy) < 0.01 &&
          std::abs(offset.z() - fz) < 0.01);
    };
  node.Subscribe("/gui/currently_tracked", statusCallback);
  auto publisher = node.Advertise<gz::msgs::CameraTrack>("/gui/track");
  auto tick = []() { std::this_thread::sleep_for(std::chrono::milliseconds(250)); };
  while (alive() && !publisher.HasConnections()) tick();
  if (!alive()) return;

  if (takeoff) {
    std::function<void(const gz::msgs::Pose_V &)> positionCallback =
      [&altitude, target](const gz::msgs::Pose_V &poses) {
        for (const auto &pose : poses.pose())
          if (pose.name() == target) altitude.store(pose.position().z());
      };
    node.Subscribe("/world/" + world + "/dynamic_pose/info", positionCallback);
    // Position near pad once. TRACK rotates camera as drone rises from ground.
    bool positioned = false;
    while (alive() && !positioned) {
      gz::msgs::GUICamera camera;
      auto position = camera.mutable_pose()->mutable_position();
      position->set_x(x); position->set_y(y); position->set_z(z);
      gz::msgs::Boolean response;
      bool result = false;
      positioned = node.Request("/gui/move_to/pose", camera, 1000, response, result)
                   && result && response.data();
      tick();
    }
    gz::msgs::CameraTrack track;
    track.set_track_mode(gz::msgs::CameraTrack::TRACK);
    track.mutable_track_target()->set_name(target);
    track.set_track_pgain(0.2);
    bool announced = false;
    while (alive() && altitude.load() < switchAltitude) {
      if (!trackCorrect.load()) publisher.Publish(track);
      else if (!announced) {
        std::cout << "[Camera] Ground takeoff shot ready (TRACK)." << std::endl;
        announced = true;
      }
      tick();
    }
    // Remove callback before its std::function goes out of scope.
    node.Unsubscribe("/world/" + world + "/dynamic_pose/info");
  }

  gz::msgs::CameraTrack follow;
  follow.set_track_mode(gz::msgs::CameraTrack::FOLLOW);
  follow.mutable_follow_target()->set_name(target);
  follow.mutable_follow_offset()->set_x(fx);
  follow.mutable_follow_offset()->set_y(fy);
  follow.mutable_follow_offset()->set_z(fz);
  follow.set_follow_pgain(0.1);
  follow.set_track_pgain(0.1);
  bool announced = false;
  while (alive()) {
    // Keep observing after acquisition, including turns, descent and landing.
    if (!followCorrect.load()) {
      publisher.Publish(follow);
      announced = false;
    } else if (!announced) {
      std::cout << "[Camera] Continuous FOLLOW locked to " << target
                << " offset (" << fx << "," << fy << "," << fz << ")." << std::endl;
      announced = true;
    }
    tick();
  }
}

int main(int argc, char **argv)
{
  // Attach updated camera controller to an existing GUI without restarting flight.
  if (argc == 7 && std::string(argv[1]) == "--camera-only") {
    const double x = std::stod(argv[3]), y = std::stod(argv[4]), z = std::stod(argv[5]);
    TrackDrone(argv[2], x, y, z, false, "", x, y, z, 0, std::stoi(argv[6]));
    return 0;
  }
  @autoreleasepool {
    // Bundle built at Simulation/.local/macos/FYP Gazebo.app.
    NSString *sim = [[[[NSBundle mainBundle] bundlePath]
      stringByDeletingLastPathComponent] stringByDeletingLastPathComponent];
    sim = [sim stringByDeletingLastPathComponent];
    const std::string root([sim UTF8String]);
    const std::string prefix = root + "/.gazebo-env";
    auto env = [](const char *name, const std::string &value) {
      if (std::getenv(name) == nullptr) setenv(name, value.c_str(), 1);
    };
    env("GZ_CONFIG_PATH", prefix + "/share/gz");
    env("QT_PLUGIN_PATH", prefix + "/plugins");
    env("QML2_IMPORT_PATH", prefix + "/qml");
    env("OGRE_RESOURCE_PATH", prefix + "/lib/OGRE");
    env("OGRE2_RESOURCE_PATH", prefix + "/lib/OGRE-Next");
    [NSApplication sharedApplication];
    [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
    [NSApp activateIgnoringOtherApps:YES];
    const char *config = argc > 1 ? argv[1] : nullptr;
    if (argc > 2) {
      const std::string target = argv[2];
      const double x = argc > 5 ? std::stod(argv[3]) : -30.0;
      const double y = argc > 5 ? std::stod(argv[4]) : -30.0;
      const double z = argc > 5 ? std::stod(argv[5]) : 20.0;
      const bool takeoff = argc > 6 && std::string(argv[6]) == "track";
      const std::string world = argc > 7 ? argv[7] : "";
      const double wideX = argc > 11 ? std::stod(argv[8]) : x;
      const double wideY = argc > 11 ? std::stod(argv[9]) : y;
      const double wideZ = argc > 11 ? std::stod(argv[10]) : z;
      const double overviewAlt = argc > 11 ? std::stod(argv[11]) : 35.0;
      std::thread([target, x, y, z, takeoff, world, wideX, wideY, wideZ, overviewAlt]() {
        TrackDrone(target, x, y, z, takeoff, world, wideX, wideY, wideZ, overviewAlt);
      }).detach();
    }
    int guiArgc = 1;
    return gz::sim::gui::runGui(guiArgc, argv, config, "ogre2");
  }
}
