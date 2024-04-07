import matplotlib.pyplot as plt
import argparse
import zipfile
import os

def read_and_split_file(file_path):
    with open(file_path, "r") as file:
        return [float(line.strip()) for line in file]

def main():
    # Parser
    parser = argparse.ArgumentParser(description="Process a file and remove consecutive duplicate values")
    parser.add_argument('-f1', '--filepath1', required=True, help='Add path file')
    parser.add_argument('-f2', '--filepath2', required=True, help='Add path file')
    args = parser.parse_args()

    x1_file_path = f"{args.filepath1}/x.txt"
    y1_file_path = f"{args.filepath1}/y.txt"

    x1_data = read_and_split_file(x1_file_path)
    y1_data = read_and_split_file(y1_file_path)
    print(f"Odometry data(x, y): {len(x1_data), len(y1_data)}")
    
    x2_file_path = f"{args.filepath2}/x.txt"
    y2_file_path = f"{args.filepath2}/y.txt"

    x2_data = read_and_split_file(x2_file_path)
    y2_data = read_and_split_file(y2_file_path)
    print(f"SLAM data(x, y): {len(x2_data), len(y2_data)}")
    
    plt.figure(figsize=(8, 6))
    plt.plot(x1_data, y1_data, 'bo-', label='SLAM Path')
    plt.plot(x2_data, y2_data, 'ro-', label='GT Path')
    plt.title('Path')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.grid(True)
    plt.savefig("Trajectory")
    plt.show()
    

if __name__ == '__main__':
    main()
