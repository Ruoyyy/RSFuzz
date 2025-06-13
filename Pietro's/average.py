import re

# 读取txt文件并计算数字数量和平均数（包括科学计数法数字）
def calculate_numbers_and_average(file_path):
    total_sum = 0
    numbers_count = 0
    overtime_count = 0
    with open(file_path, 'r') as file:
        for line in file:
            numbers_in_line = re.findall(r'[-+]?\d*\.\d+|[-+]?\d+', line)
            numbers_in_line = [float(s) for s in numbers_in_line]
            for number in numbers_in_line:
                if number >= 300:
                    print(number)
                    overtime_count += 1
            total_sum += sum(numbers_in_line)
            numbers_count += len(numbers_in_line)
    print(overtime_count)    
    if numbers_count > 0:
        average = total_sum / numbers_count
    else:
        average = 0
    
    return numbers_count, average

file_path = 'MA_20_result.txt'  #文件路径
numbers_count, average = calculate_numbers_and_average(file_path)
print(f'Total numbers: {numbers_count}, Average: {average}')
