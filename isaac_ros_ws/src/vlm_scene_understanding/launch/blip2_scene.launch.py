from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # ── 1. BLIP-2 scene-understanding node ─────────────────────────
    blip2 = Node(
        package="vlm_scene_understanding",
        executable="blip2_node",
        name="blip2_scene_understanding",
        output="screen",
    )

    # ── 2. Button trigger node (added) ────────────────────────────
    button = Node(
        package="vlm_scene_understanding",          # or 'traffic_decision_engine'
        executable="button_scene_trigger",       # use 'button.py' if renamed
        name="scene_button",
        output="screen",
        # environment={"USE_GPIO": "1"},            # uncomment to force GPIO
    )

    return LaunchDescription([blip2, button])