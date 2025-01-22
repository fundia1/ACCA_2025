import subprocess

# 실행할 Python 파일 목록
python_scripts = [
    "/home/jinju/ws/src/python_sim/python_sim/map/occupied_grid/grid_map_markers.py",  # script1.py 파일 경로
    "/home/jinju/ws/src/python_sim/python_sim/path/HybridAStar/hybrid_astar_git.py",  # script2.py 파일 경로
    "/home/jinju/ws/src/python_sim/python_sim/path/parking/parking_astar_find_goal.py",  # script3.py 파일 경로
]


# 각 Python 파일을 subprocess로 실행
def run_scripts():
    processes = []
    for script in python_scripts:
        process = subprocess.Popen(
            ["python3", script], stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        processes.append(process)

    # 각 프로세스의 출력 결과와 에러를 확인
    for process in processes:
        stdout, stderr = process.communicate()
        print(stdout.decode())
        if stderr:
            print(f"Error: {stderr.decode()}")


if __name__ == "__main__":
    run_scripts()
