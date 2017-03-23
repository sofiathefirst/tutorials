/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <urdf_parser/urdf_parser.h>
#include <moveit/collision_detection_fcl/collision_world_fcl.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/collision_detection_fcl/collision_common.h>
#include <geometric_shapes/shape_operations.h>

typedef collision_detection::CollisionWorldFCL DefaultCWorldType;
typedef collision_detection::CollisionRobotFCL DefaultCRobotType;

class MovePose
{
  ros::NodeHandle nh_;
  ros::Subscriber pose_target_;
  ros::Publisher display_publisher;
  ros::Publisher planning_scene_diff_publisher;
  ros::Subscriber target_match;
  moveit::planning_interface::MoveGroup *group_ptr;
  moveit_msgs::DisplayTrajectory display_trajectory;
  moveit::core::RobotStatePtr kinematic_state;
  robot_model::RobotModelPtr model;
//  planning_scene::PlanningScene planning;
//  planning_scene::PlanningScene planning_scene;
  moveit_msgs::PlanningScene planning_scene;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  boost::shared_ptr<collision_detection::CollisionRobot>        crobot;
  boost::shared_ptr<collision_detection::CollisionWorld>        cworld;

  collision_detection::AllowedCollisionMatrixPtr acm;

  public:
  MovePose():
	nh_("~")
  {
    group_ptr = new moveit::planning_interface::MoveGroup("manipulator");
    pose_target_ = nh_.subscribe("get_pose", 1, &MovePose::movetopose, this);
    target_match = nh_.subscribe("get_target", 1, &MovePose::addCollisionObject, this);
    display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

    group_ptr->setPlanningTime(3.0);
    group_ptr->allowReplanning(true);
    kinematic_state = moveit::core::RobotStatePtr(group_ptr->getCurrentState());
    kinematic_state->setToDefaultValues();
    
    // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group_ptr->getPlanningFrame().c_str());
  
  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group_ptr->getEndEffectorLink().c_str());

//  robot_model::RobotModelPtr model_;
  model.reset(new robot_model::RobotModel(group_ptr->getCurrentState()->getRobotModel()->getURDF(), group_ptr->getCurrentState()->getRobotModel()->getSRDF()));
  
//    collision_detection::CollisionRobotFCL crobot1_(model);
//    cworld_ = new DefaultCWorldType());
    crobot.reset(new DefaultCRobotType(model));
    cworld.reset(new DefaultCWorldType());
    acm.reset(new collision_detection::AllowedCollisionMatrix(model->getLinkModelNames(), true));

  }
  void checkCollision()
  {
    robot_state::RobotState state(model); 
    state.setToDefaultValues();
    state.update();

    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    collision_detection::CollisionResult res1;

    Eigen::Affine3d offset = Eigen::Affine3d::Identity();
    offset.translation().x() = .01;
    state.updateStateWithLinkAt("base_link", Eigen::Affine3d::Identity());     
    state.updateStateWithLinkAt("wrist_3_link", offset);
    state.update();

    
    acm->setEntry("base_link", "wrist_3_link", false);
    
    crobot->checkSelfCollision(req, res, state, *acm);
    bool success = res.collision;
    ROS_INFO("Robot Collision %s",success?"":"FAILED");
    
    Eigen::Affine3d pos1 = Eigen::Affine3d::Identity();
    pos1.translation().x() = 5.0;
    shapes::Shape* shape = new shapes::Box(.1,.1,.1);
    cworld->getWorld()->addToObject("box", shapes::ShapeConstPtr(shape), pos1);
  

//    res = collision_detection::CollisionResult();
    cworld->checkRobotCollision(req, res1, *crobot, state, *acm);
    
//    cworld->setWorld(cworld->getWorld());
//    cworld->checkCollision(req, res1, *crobot, state, *acm);
    success = res1.collision;
    ROS_INFO("World Collision %s",success?"":"FAILED");

  }

  void addMesh(std::string str, std::string name)
  {
    moveit_msgs::CollisionObject co;
    co.header.frame_id = group_ptr->getPlanningFrame();
    
    Eigen::Vector3d scale(0.005,0.005,0.005);
    shapes::Mesh* m = shapes::createMeshFromResource(str,scale);
    shape_msgs::Mesh co_mesh;
    shapes::ShapeMsg co_mesh_msg;
    shapes::constructMsgFromShape(m, co_mesh_msg);
    co_mesh = boost::get<shape_msgs::Mesh>(co_mesh_msg);
    
    if(name == "one")
    {
      co.id = name;
      co.meshes.resize(1);
      co.meshes[0] = co_mesh;
      co.mesh_poses.resize(1);
      co.mesh_poses[0].position.x = -0.39;
      co.mesh_poses[0].position.y = -0.39;
      co.mesh_poses[0].position.z = 0.39;
      co.mesh_poses[0].orientation.w = 0.0;
    }
    else if(name == "two")
     {
      co.id = name;
      co.meshes.resize(1);
      co.meshes[0] = co_mesh;
      co.mesh_poses.resize(1);
      co.mesh_poses[0].position.x = 0.39;
      co.mesh_poses[0].position.y = 0.39;
      co.mesh_poses[0].position.z = 0.5;
      co.mesh_poses[0].orientation.w = 0.0;
    }
    else if(name == "tag")
    {
      co.id = name;
      co.meshes.resize(1);
      co.meshes[0] = co_mesh;
      co.mesh_poses.resize(1);
      co.mesh_poses[0].position.x = 0.6;
      co.mesh_poses[0].position.y = -0.2;
      co.mesh_poses[0].position.z = 0.7;
      co.mesh_poses[0].orientation.w = 0.0;
    }
    co.meshes.push_back(co_mesh);
    co.mesh_poses.push_back(co.mesh_poses[0]);
    co.operation = co.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_object;
    collision_object.push_back(co);

    planning_scene_interface.addCollisionObjects(collision_object);
    sleep(2);
    
    }
  void addCollision(int num)
  {
    
    planning_scene::PlanningScene ps(model);
    collision_detection::World &world = *ps.getWorldNonConst();
    collision_detection::CollisionRequest req;
    collision_detection::CollisionResult res;
    moveit::planning_interface::MoveGroup::Plan my_plan;
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group_ptr->getPlanningFrame();

    shape_msgs::SolidPrimitive base;
    geometry_msgs::Pose base_pose;
    switch (num)
    {
      case 1:
      {
	collision_object.id = "box";
        base.type = base.BOX;
        base.dimensions.resize(3);
        base.dimensions[0] = 0.2;
        base.dimensions[1] = 0.1;
        base.dimensions[2] = 0.038;

        base_pose.orientation.w = 1.0;
        base_pose.position.x = -0.2;
        base_pose.position.y = 0.5;
        base_pose.position.z = 0.4;
        break;
      }
      case 2:
      {
        collision_object.id = "cylinder";
        base.type = base.CYLINDER;
        base.dimensions.resize(2);
        base.dimensions[0] = 0.15;
        base.dimensions[1] = 0.1;

        base_pose.orientation.w = 0.0;
        base_pose.position.x = -0.3;
        base_pose.position.y = 0.5;
        base_pose.position.z = 0.7;	
	break;
      }
      case 3:
      {
	collision_object.id = "sphere";
        base.type = base.SPHERE;
        base.dimensions.resize(1);
        base.dimensions[0] = 0.1;

        base_pose.orientation.w = 0.0;
        base_pose.position.x = 0.2;
        base_pose.position.y = 0.6;
        base_pose.position.z = 0.7;	
	break;
       }
    }

    collision_object.primitives.push_back(base);
    collision_object.primitive_poses.push_back(base_pose);

    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    planning_scene.world.collision_objects.push_back(collision_object);
    planning_scene.robot_state.is_diff = true;
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene); 

    ps.getPlanningSceneMsg(planning_scene);
    ps.setPlanningSceneMsg(planning_scene);
  
//    ps.checkCollision(req, res);
//    bool success = res.collision;
//    bool success = ps.isStateColliding();
//    planning_scene_interface.addCollisionObjects(collision_objects);
    sleep(2.0);

 //   group_ptr->setPlanningTime(10.0);

 //   group_ptr->setStartState(*group_ptr->getCurrentState());
 //   group_ptr->setPoseTarget(target_pose);
  //  bool success = group_ptr->plan(my_plan);
  //  ROS_INFO("Planning scene Collision %s",success?"":"FAILED");
    
  }
  void addCollisionObjects()
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = group_ptr->getPlanningFrame();

    collision_object.id = "box";

    shape_msgs::SolidPrimitive box_base;
    box_base.type = box_base.BOX;
    box_base.dimensions.resize(3);
    box_base.dimensions[0] = 1.47;
    box_base.dimensions[1] = 0.391;
    box_base.dimensions[2] = 0.038;

    geometry_msgs::Pose base_pose;
    base_pose.orientation.w = 1.0;
    base_pose.position.x = 0.0;
    base_pose.position.y = 0.78;
    base_pose.position.z = 0.019;

    geometry_msgs::Pose top_pose;
    top_pose = base_pose;
    top_pose.position.z += 1.432;

    shape_msgs::SolidPrimitive box_extrinsic;
    box_extrinsic.type = box_extrinsic.BOX;
    box_extrinsic.dimensions.resize(3);
    box_extrinsic.dimensions[0] = 0.038;
    box_extrinsic.dimensions[1] = 0.391;
    box_extrinsic.dimensions[2] = 1.396;

    geometry_msgs::Pose extrinsic_pose1;
    extrinsic_pose1.orientation.w = 1.0;
    extrinsic_pose1.position.x = 0.758-0.038;
    extrinsic_pose1.position.y = 0.78;
    extrinsic_pose1.position.z = 0.735;

    geometry_msgs::Pose extrinsic_pose2;
    extrinsic_pose2 = extrinsic_pose1;
    extrinsic_pose2.position.x = -extrinsic_pose1.position.x;

    shape_msgs::SolidPrimitive box_intrinsic1;
    box_intrinsic1.type = box_intrinsic1.BOX;
    box_intrinsic1.dimensions.resize(3);
    box_intrinsic1.dimensions[0] = 1.395;
    box_intrinsic1.dimensions[1] = 0.339;
    box_intrinsic1.dimensions[2] = 0.016;

    geometry_msgs::Pose intrinsic_pose1;
    intrinsic_pose1 = base_pose;
    intrinsic_pose1.position.z += 0.354;

    geometry_msgs::Pose intrinsic_pose2;
    intrinsic_pose2 = intrinsic_pose1;
    intrinsic_pose2.position.z += 0.354;
 
    geometry_msgs::Pose intrinsic_pose3;
    intrinsic_pose3 = intrinsic_pose2;
    intrinsic_pose3.position.z += 0.354;

    shape_msgs::SolidPrimitive box_intrinsic2;
    box_intrinsic2.type = box_intrinsic2.BOX;
    box_intrinsic2.dimensions.resize(3);
    box_intrinsic2.dimensions[0] = 0.016;
    box_intrinsic2.dimensions[1] = 0.332;
    box_intrinsic2.dimensions[2] = 1.398;

    geometry_msgs::Pose intrinsic_pose4;
    intrinsic_pose4 = extrinsic_pose1;
    intrinsic_pose4.position.x -= 0.354;
    intrinsic_pose4.position.z = 0.699+0.038;

    geometry_msgs::Pose intrinsic_pose5;
    intrinsic_pose5 = intrinsic_pose4;
    intrinsic_pose5.position.x -= 0.354;

    geometry_msgs::Pose intrinsic_pose6;
    intrinsic_pose6 = intrinsic_pose5;
    intrinsic_pose6.position.x -= 0.354;

    collision_object.primitives.push_back(box_base);
    collision_object.primitives.push_back(box_base);
    collision_object.primitives.push_back(box_extrinsic);
    collision_object.primitives.push_back(box_extrinsic);
    collision_object.primitives.push_back(box_intrinsic1);
    collision_object.primitives.push_back(box_intrinsic1);
    collision_object.primitives.push_back(box_intrinsic1);
    collision_object.primitives.push_back(box_intrinsic2);
    collision_object.primitives.push_back(box_intrinsic2);
    collision_object.primitives.push_back(box_intrinsic2);
    collision_object.primitive_poses.push_back(base_pose);
    collision_object.primitive_poses.push_back(top_pose);
    collision_object.primitive_poses.push_back(extrinsic_pose1);
    collision_object.primitive_poses.push_back(extrinsic_pose2);
    collision_object.primitive_poses.push_back(intrinsic_pose1);
    collision_object.primitive_poses.push_back(intrinsic_pose2);
    collision_object.primitive_poses.push_back(intrinsic_pose3);
    collision_object.primitive_poses.push_back(intrinsic_pose4);
    collision_object.primitive_poses.push_back(intrinsic_pose5);
    collision_object.primitive_poses.push_back(intrinsic_pose6);
    collision_object.operation = collision_object.ADD;

//    std::vector<moveit_msgs::CollisionObject> collision_objects;
//    collision_objects.push_back(collision_object);

//    planning_scene.world.collision_objects.push_back(collision_object);
//    planning_scene.is_diff = true;
//    planning_scene_diff_publisher.publish(planning_scene);
//    planning_scene::isStateColliding();
    sleep(2.0);

//    group_ptr->setPlanningTime(10.0);
  }
  void addCollisionObject(const std_msgs::String target)
  {
    ROS_INFO("I detected %s", target.data.c_str());
    if(target.data == "06077151230")
       this->addMesh("package://ur_description/meshes/ur5/collision/one.stl", "one");
    else if(target.data == "0772181589")
       this->addMesh("package://ur_description/meshes/ur5/collision/two.stl", "two");
    else if(target.data == "08114013289")
       this->addMesh("package://ur_description/meshes/ur5/collision/tag.stl", "tag");
    else ROS_INFO("No matching data");
  }
  void movetopose(const geometry_msgs::Pose& pose)
  {
    ROS_INFO("I heard: [%f,%f,%f,%f,%f,%f,%f]", pose.orientation.x,pose.orientation.y,pose.orientation.z,
pose.orientation.w,pose.position.x,pose.position.y,pose.position.z);

//    moveit::planning_interface::MoveGroup::Plan my_plan;
    move_group_interface::MoveGroup::Plan my_plan;
    moveit_msgs::RobotTrajectory trajectory;
    std::vector<geometry_msgs::Pose> waypoints;
    group_ptr->setStartStateToCurrentState();
    waypoints.push_back(this->group_ptr->getCurrentPose().pose);
    waypoints.push_back(pose);
   
    double fraction = group_ptr->computeCartesianPath(waypoints,
							0.01,	//eef_step
							0.0,	//jump_thredshold
							trajectory, true);
    
    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);

    robot_trajectory::RobotTrajectory rt(this->kinematic_state->getRobotModel(),"manipulator");
    rt.setRobotTrajectoryMsg(*(this->kinematic_state), trajectory);

    // Thrid create a IterativeParabolicTimeParameterization object
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    bool success = iptp.computeTimeStamps(rt);
    ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");
   // Get RobotTrajectory_msg from RobotTrajectory

    rt.getRobotTrajectoryMsg(trajectory);
    // Finally plan and execute the trajectory
    my_plan.trajectory_ = trajectory;
    success = this->group_ptr->execute(my_plan);
    ROS_INFO("Executing the plan %s",success?"SUCCEDED":"FAILED");
    
    this->kinematic_state = this->group_ptr->getCurrentState();
    waypoints.clear();
  }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_pose");
  MovePose move;
//  move.addCollision();
//  move.checkCollision();
//  move.addCollisionObjects();
  ros::spin();

  return 0;
}
