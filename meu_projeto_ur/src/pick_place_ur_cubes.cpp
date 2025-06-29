/*********************************************************************
 * pick_place_final.cpp
 *
 * Software License Agreement (BSD License)
 *********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra. Adaptado por Francisco e ajustado por ChatGPT. */

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Mensagens
#include <moveit_msgs/PlanningScene.h>
#include <geometry_msgs/Pose.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Estrutura para guardar os dados de cada cubo
struct Cube
{
  std::string name;
  geometry_msgs::Pose initial_pose;
  geometry_msgs::Pose target_pose;
  std_msgs::ColorRGBA color;
};

// Adiciona mesa e cubos à cena
void addCollisionObjects(ros::NodeHandle& nh, const std::vector<Cube>& cubes)
{
  static ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    ros::WallDuration(0.5).sleep();

  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;

  // Mesa
  moveit_msgs::CollisionObject table;
  table.id = "table";
  table.header.frame_id = "base_link";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions = {1.5, 1.5, 0.02};
  geometry_msgs::Pose table_pose;
  table_pose.position.z = -0.05; // Superfície em z = 0
  table_pose.orientation.w = 1.0;
  table.primitives.push_back(primitive);
  table.primitive_poses.push_back(table_pose);
  table.operation = table.ADD;
  planning_scene.world.collision_objects.push_back(table);

  // Cubos
  for (const auto& cube : cubes)
  {
    moveit_msgs::CollisionObject obj;
    obj.id = cube.name;
    obj.header.frame_id = "base_link";
    shape_msgs::SolidPrimitive shape;
    shape.type = shape.BOX;
    shape.dimensions = {0.05, 0.05, 0.05};
    obj.primitives.push_back(shape);
    obj.primitive_poses.push_back(cube.initial_pose);
    obj.operation = obj.ADD;
    planning_scene.world.collision_objects.push_back(obj);

    moveit_msgs::ObjectColor color;
    color.id = cube.name;
    color.color = cube.color;
    planning_scene.object_colors.push_back(color);
  }

  planning_scene_diff_publisher.publish(planning_scene);
  ROS_INFO("Scene published with table and cubes.");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ur_pick_place_final_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  move_group.setPlanningTime(20.0);
  move_group.setNumPlanningAttempts(5);

  // Lista de cubos
  std::vector<Cube> cubes_to_process;

  double cube_z_height = 0.035; // Cubo fica com topo em z = 0.06

  // Cubo 1 (vermelho)
  Cube cube1;
  cube1.name = "red_cube";
  cube1.initial_pose.position.x = 0.5;
  cube1.initial_pose.position.y = -0.2;
  cube1.initial_pose.position.z = cube_z_height;
  cube1.initial_pose.orientation.w = 1.0;
  cube1.target_pose = cube1.initial_pose;
  cube1.target_pose.position.x = -0.4;
cube1.color.r = 1.0; cube1.color.g = 0.0; cube1.color.b = 0.0; cube1.color.a = 1.0;
  cubes_to_process.push_back(cube1);

  // Cubo 2 (verde)
  Cube cube2;
  cube2.name = "green_cube";
  cube2.initial_pose.position.x = 0.5;
  cube2.initial_pose.position.y = 0.0;
  cube2.initial_pose.position.z = cube_z_height;
  cube2.initial_pose.orientation.w = 1.0;
  cube2.target_pose = cube2.initial_pose;
  cube2.target_pose.position.x = -0.4;
cube2.color.r = 0.0; cube2.color.g = 1.0; cube2.color.b = 0.0; cube2.color.a = 1.0;
  cubes_to_process.push_back(cube2);

  // Cubo 3 (azul)
  Cube cube3;
  cube3.name = "blue_cube";
  cube3.initial_pose.position.x = 0.5;
  cube3.initial_pose.position.y = 0.2;
  cube3.initial_pose.position.z = cube_z_height;
  cube3.initial_pose.orientation.w = 1.0;
  cube3.target_pose = cube3.initial_pose;
  cube3.target_pose.position.x = -0.4;
cube3.color.r = 0.0; cube3.color.g = 0.0; cube3.color.b = 1.0; cube3.color.a = 1.0;
  cubes_to_process.push_back(cube3);

  // Sobe antes de adicionar objetos
  move_group.setNamedTarget("up");
  move_group.move();
  ros::WallDuration(1.0).sleep();

  addCollisionObjects(nh, cubes_to_process);
  ros::WallDuration(1.0).sleep();
 // ANTES DO LOOP 'for'
tf2::Quaternion orientation_down;
orientation_down.setRPY(-M_PI, 0, 0); // Garra aponta para baixo (Roll de -180 graus)
geometry_msgs::Quaternion q_down = tf2::toMsg(orientation_down);

// Defina estas constantes antes do loop para facilitar a leitura
const double eef_step = 0.01;       // Resolução do caminho cartesiano (1 cm)
const double jump_threshold = 0.0;  // Desativa o filtro de salto de junta

// Loop de manipulação
for (const auto& cube : cubes_to_process)
{
    ROS_INFO("Processing cube: %s", cube.name.c_str());

    // Fase 1: Ir para a pose 'up' (Movimento grande, pode ser no espaço de juntas)
    move_group.setNamedTarget("up");
    if (move_group.move().val != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_ERROR("Failed to move to 'up' pose. Skipping cube.");
        continue;
    }

    // Fase 2: Mover para a pré-aproximação (Movimento grande, pode ser no espaço de juntas)
    geometry_msgs::Pose pre_approach_pose = cube.initial_pose;
    pre_approach_pose.position.z = 0.30;
    pre_approach_pose.orientation = q_down;
    move_group.setPoseTarget(pre_approach_pose);
    if (move_group.move().val != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_ERROR("Failed to move to pre-approach pose. Skipping cube.");
        continue;
    }

    // Fase 3 e 4: Descida final para a PEGA (Movimento preciso, usar Cartesiano)
    ROS_INFO("Executing Cartesian path for grasping...");
    std::vector<geometry_msgs::Pose> grasp_waypoints;
    geometry_msgs::Pose grasp_pose = pre_approach_pose; // Começa da pose atual
    
    // Waypoint intermediário (opcional, mas bom para suavizar)
    grasp_pose.position.z = 0.12;
    grasp_waypoints.push_back(grasp_pose);
    
    // Waypoint final da pega
    grasp_pose.position.z = 0.08;
    grasp_waypoints.push_back(grasp_pose);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(grasp_waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 0.9) {
        ROS_WARN("Could not compute a complete Cartesian path for grasping (got %.2f). Skipping cube.", fraction);
        continue;
    }

    move_group.execute(trajectory);
    ros::WallDuration(0.5).sleep();

    // Fase 5: Anexar o objeto
    move_group.attachObject(cube.name, "tool0");
    ros::WallDuration(0.5).sleep();

    // Fase 6: Subida vertical (Movimento preciso, usar Cartesiano)
    ROS_INFO("Executing Cartesian path for retreat...");
    std::vector<geometry_msgs::Pose> retreat_waypoints;
    retreat_waypoints.push_back(pre_approach_pose); // Recua para a pose de pré-aproximação

    move_group.computeCartesianPath(retreat_waypoints, eef_step, jump_threshold, trajectory);
    move_group.execute(trajectory);

    // Fase 7: Mover para a área de largar (Movimento grande, pode ser no espaço de juntas)
    geometry_msgs::Pose target_pose = cube.target_pose;
    target_pose.orientation = q_down;
    
    geometry_msgs::Pose target_approach = target_pose;
    target_approach.position.z = 0.30; // Aproxima-se por cima na área de destino

    move_group.setPoseTarget(target_approach);
    if (move_group.move().val != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_ERROR("Failed to move to target area. Detaching object and skipping.");
        move_group.detachObject(cube.name);
        continue;
    }

    // Fase 8: Descida final para LARGAR (Movimento preciso, usar Cartesiano)
    ROS_INFO("Executing Cartesian path for placing...");
    std::vector<geometry_msgs::Pose> place_waypoints;

    // Waypoint intermediário
    target_pose.position.z = 0.12;
    place_waypoints.push_back(target_pose);

    // Waypoint final para largar
    target_pose.position.z = 0.08;
    place_waypoints.push_back(target_pose);

    fraction = move_group.computeCartesianPath(place_waypoints, eef_step, jump_threshold, trajectory);

    if (fraction < 0.9) {
        ROS_WARN("Could not compute a complete Cartesian path for placing (got %.2f). Detaching object and skipping.", fraction);
        move_group.detachObject(cube.name);
        continue;
    }

    move_group.execute(trajectory);
    ros::WallDuration(0.5).sleep();

    // Fase 9: Soltar o objeto
    move_group.detachObject(cube.name);
    ros::WallDuration(0.5).sleep();

    // Fase 10: Subida vertical após largar (Movimento preciso, usar Cartesiano)
    move_group.computeCartesianPath({target_approach}, eef_step, jump_threshold, trajectory);
    move_group.execute(trajectory);
}

  // Ao final de tudo, volta para a pose 'up'
  move_group.setNamedTarget("up");
  move_group.move();

  ROS_INFO("All cubes processed. Shutting down.");
  ros::waitForShutdown();
  return 0;
}
