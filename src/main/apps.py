from django.apps import AppConfig
import subprocess
import os

class MainConfig(AppConfig):
    default_auto_field = 'django.db.models.BigAutoField'
    name = 'main'

    def ready(self):
        try:
            ros_path = os.path.join(os.path.dirname(__file__), 'ros_runner.py')
            subprocess.Popen(['python3', ros_path])
            print("[ROS Listener] subprocess launched ✅")
        except Exception as e:
            print(f"[ROS Listener Error] {e}")