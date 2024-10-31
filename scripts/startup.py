# launch_manager.py
import subprocess
import signal
import sys
import time
import os

def main():
    # Launchファイルのパスを設定
    main_launch = 'ros2 launch amr_slam_nav_core startup_cartographer.launch.py'
    agent_launch = 'ros2 launch amr_slam_nav_core micro_ros_agent.launch.py'

    # Launchファイルをバックグラウンドで実行
    print("Starting micro_ros_agent_launch.py...")
    agent_proc = subprocess.Popen(agent_launch, shell=True, preexec_fn=os.setsid)
    time.sleep(2)  # micro_ros_agentが起動するまで待機（必要に応じて調整）

    print("Starting main_launch.py...")
    main_proc = subprocess.Popen(main_launch, shell=True, preexec_fn=os.setsid)

    try:
        while True:
            # メインプロセスとエージェントプロセスの状態を監視
            main_ret = main_proc.poll()
            agent_ret = agent_proc.poll()

            if main_ret is not None:
                print("Main launch file has exited.")
                break

            if agent_ret is not None:
                print("micro_ros_agent launch file has exited.")
                break

            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down launch files...")

        # メインLaunchファイルを終了させる
        try:
            print("Sending SIGINT to main_launch.py...")
            os.killpg(os.getpgid(main_proc.pid), signal.SIGINT)
            main_proc.wait(timeout=10)
            print("main_launch.py terminated.")
        except subprocess.TimeoutExpired:
            print("main_launch.py did not terminate gracefully, killing...")
            os.killpg(os.getpgid(main_proc.pid), signal.SIGKILL)

        # エージェントLaunchファイルを終了させる
        try:
            print("Sending SIGINT to micro_ros_agent_launch.py...")
            os.killpg(os.getpgid(agent_proc.pid), signal.SIGINT)
            agent_proc.wait(timeout=10)
            print("micro_ros_agent_launch.py terminated.")
        except subprocess.TimeoutExpired:
            print("micro_ros_agent_launch.py did not terminate gracefully, killing...")
            os.killpg(os.getpgid(agent_proc.pid), signal.SIGKILL)

    finally:
        # すべてのプロセスを終了させる
        if agent_proc.poll() is None:
            agent_proc.terminate()
        if main_proc.poll() is None:
            main_proc.terminate()
        try:
            agent_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            agent_proc.kill()
        try:
            main_proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            main_proc.kill()
        print("All launch files have been terminated.")

if __name__ == "__main__":
    main()
