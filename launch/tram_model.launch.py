from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from tram_model import tram_model as model


def generate_launch_description() -> LaunchDescription:
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("tram_model"), "rviz", "tram_model.rviz"])

    args = [
        DeclareLaunchArgument("executable", default_value="train_model_ros",
                              description="train_model_ros или simple_model_ros"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Запускать rviz2"),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true, если bag проигрывается с --clock"),
        DeclareLaunchArgument("use_gnss_init", default_value="true",
                              description="Начальная выставка origin/курса по GNSS"),
        DeclareLaunchArgument("publish_gnss_reference", default_value="true",
                              description="Публиковать GNSS-эталон /reference/path для rviz"),
        DeclareLaunchArgument("gnss_fusion", default_value="true",
                              description="Корректировать позу EKF по GNSS, пока он есть"),
        DeclareLaunchArgument("gnss_fusion_max_time_s", default_value="0.0",
                              description="0 — всегда; иначе GNSS учитывается только первые N с"),
        DeclareLaunchArgument("velocity_source", default_value="ekf",
                              description="Источник /result/velocity: ekf или model"),
        DeclareLaunchArgument("position_source", default_value="ekf",
                              description="Источник /result/position: ekf или route"),
        DeclareLaunchArgument("tram_mass_kg", default_value=str(model.TRAM_WEIGHT)),
        DeclareLaunchArgument("traction_torque_per_notch_nm", default_value=str(model.K_POS)),
        DeclareLaunchArgument("brake_torque_per_notch_nm", default_value=str(model.K_NEG)),
        DeclareLaunchArgument("max_brake_torque_nm", default_value=str(model.MAX_BRAKE_TORQUE)),
        DeclareLaunchArgument("wheel_correction_gain", default_value="0.5"),
        DeclareLaunchArgument("adh_cond", default_value="0"),
    ]

    tram_model = Node(
        package="tram_model",
        executable=LaunchConfiguration("executable"),
        name="tram_model",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_gnss_init": LaunchConfiguration("use_gnss_init"),
            "publish_gnss_reference": LaunchConfiguration("publish_gnss_reference"),
            "gnss_fusion": LaunchConfiguration("gnss_fusion"),
            "gnss_fusion_max_time_s": LaunchConfiguration("gnss_fusion_max_time_s"),
            "velocity_source": LaunchConfiguration("velocity_source"),
            "position_source": LaunchConfiguration("position_source"),
            "tram_mass_kg": LaunchConfiguration("tram_mass_kg"),
            "traction_torque_per_notch_nm": LaunchConfiguration("traction_torque_per_notch_nm"),
            "brake_torque_per_notch_nm": LaunchConfiguration("brake_torque_per_notch_nm"),
            "max_brake_torque_nm": LaunchConfiguration("max_brake_torque_nm"),
            "wheel_correction_gain": LaunchConfiguration("wheel_correction_gain"),
            "adh_cond": LaunchConfiguration("adh_cond"),
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription([*args, tram_model, rviz])
