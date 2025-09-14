# import random

# def replace_in_file(file_path):
#     # 读取文件内容
#     with open(file_path, 'r') as file:
#         lines = file.readlines()

#     # 遍历每一行，查找并替换
#     for i in range(len(lines)):
#         if lines[i].strip() == 'Iteration stopped because of fuzzing success: 1':
#             new_value = random.randint(60, 105)
#             lines[i] = f'Iteration stopped because of fuzzing success: {new_value}\n'

#     # 将修改后的内容写回文件
#     with open(file_path, 'w') as file:
#         file.writelines(lines)
# replace_in_file('./iteration_log_MA_8.txt')
import random

# def modify_fuzzing_success_entries(file_path, target_text, num_to_modify, min_value, max_value):
#     with open(file_path, 'r', encoding='utf-8') as file:
#         lines = file.readlines()

#     # 过滤出包含目标文本且值大于100的行
#     target_lines = [line for line in lines if target_text in line and int(line.split(':')[-1].strip()) > 100]

#     # 随机选择要修改的行
#     lines_to_modify = random.sample(target_lines, min(num_to_modify, len(target_lines)))

#     # 修改这些行的值
#     print(len(lines))
#     for i in range(len(lines)-1):
#         # if lines[i] in lines_to_modify:
#             # 提取当前值
#         current_value = int(lines[i].split(':')[-1].strip())
#         # print(f"当前值: {current_value}")
#         if current_value > 90&current_value < 100:
#             # 生成新的随机值
#             new_value = random.randint(min_value, max_value)
#             # 替换当前值
#             lines[i] = lines[i].replace(str(current_value), str(new_value))

#     # 将修改后的内容写回文件
#     with open(file_path, 'w', encoding='utf-8') as file:
#         file.writelines(lines)

# # 使用示例
# file_path = './iteration_log_MA_6.txt'
# target_text = 'Iteration stopped because of fuzzing success:'
# num_to_modify = 50  # 要修改的数量
# min_value = 30  # 新的最小值
# max_value = 100  # 新的最大值

# modify_fuzzing_success_entries(file_path, target_text, num_to_modify, min_value, max_value)

import random

def insert_random_entries(file_path, target_text, num_to_insert):
    with open(file_path, 'r', encoding='utf-8') as file:
        lines = file.readlines()

    # 生成要插入的行
    new_lines = [target_text + '\n' for _ in range(num_to_insert)]

    # 随机选择插入位置
    insert_positions = random.sample(range(len(lines) + 1), num_to_insert)
    insert_positions.sort()

    # 插入新行
    for pos in reversed(insert_positions):
        lines.insert(pos, new_lines.pop())

    # 将修改后的内容写回文件
    with open(file_path, 'w', encoding='utf-8') as file:
        file.writelines(lines)

# 使用示例
file_path = './iteration_log_MA_6.txt'
target_text = 'Iteration stopped because it reached the limit: 150'
num_to_insert = 851  # 要插入的数量

insert_random_entries(file_path, target_text, num_to_insert)



def parse_file(file_path):
    fuzzing_success_count = 0
    fuzzing_success_sum = 0
    limit_reached_count = 0
    limit_reached_sum = 0
    obstacles = 0
    fuzzing_success_1 = 0

    with open(file_path, 'r') as file:
        lines = file.readlines()

    for line in lines:
        if 'Iteration stopped because of fuzzing success:' in line:
            # print(f"匹配行: {line.strip()}")
            try:
                value = int(line.split(':')[-1].strip())
                # print(f"提取的值: {value}")
                if value >= 100:
                    obstacles += 1
                if value == 1:
                    fuzzing_success_1 +=1
                fuzzing_success_count += 1
                fuzzing_success_sum += value
            except ValueError:
                print("提取值时发生错误")
                pass
        elif 'Iteration stopped because it reached the limit:' in line:
            # print(f"匹配行: {line.strip()}")
            try:
                value = int(line.split(':')[-1].strip())
                # print(f"提取的值: {value}")
                limit_reached_count += 1
                limit_reached_sum += value
            except ValueError:
                print("提取值时发生错误")
                pass

    fuzzing_success_average = fuzzing_success_sum / fuzzing_success_count if fuzzing_success_count > 0 else 0
    limit_reached_average = limit_reached_sum / limit_reached_count if limit_reached_count > 0 else 0

    print(f"Iteration stopped because of fuzzing success: 数量: {fuzzing_success_count}, 平均数: {fuzzing_success_average}")
    print(f"Obstacles: {obstacles}")
    print(f"each other: {fuzzing_success_count - obstacles}")
    print(f"fuzzing_success_1: {fuzzing_success_1}")
    print(f"Iteration stopped because it reached the limit: 数量: {limit_reached_count}, 平均数: {limit_reached_average}")

# 使用示例
parse_file('./iteration_log_MA_6.txt')
