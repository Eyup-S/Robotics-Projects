import matplotlib.pyplot as plt

def read_data(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
        # Convert lines to floats and group them in threes (theta1, theta2, theta3)
        data = [float(line.strip()) for line in lines]
        theta1 = data[0::3]
        theta2 = data[1::3]
        theta3 = data[2::3]
        return theta1, theta2, theta3

def plot_data(theta1, theta2, title):
    plt.figure()
    plt.plot(theta1, label='inverse kinematics theta1')
    plt.plot(theta2, label='theta1_dot')
    plt.title(title)
    plt.xlabel('Index')
    plt.ylabel('Value')
    plt.legend()
    plt.show()

file1 = '../data/inv_vel.txt'
file2 = '../data/delta_thetas.txt'

theta1_file1, theta2_file1, theta3_file1 = read_data(file1)
theta1_file2, theta2_file2, theta3_file2 = read_data(file2)

plot_data(theta1_file1, theta1_file2, 'Theta1 Comparison')
plot_data(theta2_file1, theta2_file2, 'Theta2 Comparison')
plot_data(theta3_file1, theta3_file2, 'Theta3 Comparison')
