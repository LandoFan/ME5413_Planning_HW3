import rosbag
import matplotlib.pyplot as plt

bag_paths = ['error_lqr.bag', 'error_pid.bag','error_purepersuit.bag']
topics = ['/me5413_world/planning/abs_position_error', '/me5413_world/planning/rms_position_error',
          '/me5413_world/planning/abs_speed_error', '/me5413_world/planning/rms_speed_error',
          '/me5413_world/planning/abs_heading_error', '/me5413_world/planning/rms_heading_error']

plt.figure(figsize=(10, 8))  # 设置整个图的大小
num_plots = len(topics)

for i, topic in enumerate(topics, 1):
    plt.subplot(3, 2, i)  # 创建 subplot
    plt.xlabel('Time (s)')
    plt.title(f'{topic}')

    for bag_path in bag_paths:
        times = []
        data_values = []

        with rosbag.Bag(bag_path, 'r') as bag:
            for _, msg, t in bag.read_messages(topics=[topic]):
                times.append(t.to_sec())  # 将 ROS 时间转换为秒
                data_values.append(msg.data)
            bag.close()

        plt.plot(times, data_values, label=bag_path)

plt.tight_layout()  # 调整 subplot 之间的间距
plt.legend()
plt.show()