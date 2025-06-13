import numpy as np
import matplotlib as plt
class AttackDrone:
    def __init__(self, monitor, initial_data, step_size=0.1, max_iterations=50):
        self.monitor = monitor
        self.initial_data = initial_data
        self.step_size = step_size
        self.max_iterations = max_iterations

    def plot_attack(self, attack_history):
        x_positions = [data['uav_pos'][0] for data in attack_history]
        y_positions = [data['uav_pos'][1] for data in attack_history]

        plt.plot(x_positions, y_positions, marker='o')
        plt.title('Attacker Position')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.grid(True)
        plt.show()

    def find_attack_vector(self):
        # 初始攻击方向，这里简单地选择一个随机方向
        return np.random.rand(2)

    def execute_attack(self):
        attack_vector = self.find_attack_vector()
        current_data = self.initial_data.copy()
        attack_history = [current_data.copy()]
        
        for iteration in range(self.max_iterations):
            # 模拟攻击
            current_data['uav_pos'] += attack_vector * self.step_size
            attack_history.append(current_data.copy())
            # 计算新的健壮性得分
            new_rob = self.monitor.multi_rule_monitor(raw_data=current_data)
            
            # 检查健壮性得分是否降低，若没降低则调整攻击方向
            if new_rob >= self.monitor.multi_rule_monitor(raw_data=self.initial_data):
                attack_vector = self.find_attack_vector()  # 随机选择新的攻击方向
            else:
                # 更新数据以继续在此方向上攻击
                self.initial_data = current_data.copy()

            # 输出当前健壮性得分和攻击步骤
            print(f"Iteration {iteration}: Robustness = {new_rob}")
        # 绘制攻击机位置
        self.plot_attack(attack_history)

