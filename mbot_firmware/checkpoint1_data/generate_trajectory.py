import matplotlib.pyplot as plt
import argparse
import zipfile
import os

def read_and_split_file(file_path):
    with open(file_path, "r") as file:
        return [float(line.strip()) for line in file]

def remove_duplicate(data):
    filtered_data = [data[0]]

    for i in range(1, len(data)):
        if data[i] != filtered_data[-1]:
            filtered_data.append(data[i])

    return filtered_data

def main():
    # Parser
    parser = argparse.ArgumentParser(description="Process a file and remove consecutive duplicate values")
    parser.add_argument('-f', '--filepath', required=True, help='Add path file')
    args = parser.parse_args()

    x_file_path = f"{args.filepath}/x.txt"
    y_file_path = f"{args.filepath}/y.txt"

    x_data = read_and_split_file(x_file_path)
    y_data = read_and_split_file(y_file_path)
    print(f"Total data(x, y): {len(x_data), len(y_data)}")
    # filtered_x_data = remove_duplicate(x_data)
    # filtered_y_data = remove_duplicate(y_data)
    # print(f"Filtered data(x, y): {len(filtered_x_data), len(filtered_y_data)}")
    
    plt.figure(figsize=(8, 6))
    plt.plot(x_data, y_data, label='Odometry Path', marker='o')  # Using 'o' for points
    plt.title(f'{args.filepath} Odometry Path')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.legend()
    plt.grid(True)
    plt.savefig(f"Trajectory_{args.filepath}")
    plt.show()
    

if __name__ == '__main__':
    main()
