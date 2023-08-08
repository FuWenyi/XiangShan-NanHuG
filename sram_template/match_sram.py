# -*- coding: utf-8 -*-

import re

def extract_data(file_path):
    pattern = r',<(\d+)>,(\d+),(\d+)'
    count_dict = {}

    with open(file_path, 'r') as f:
        lines = f.readlines()
        for line in lines:
            matches = re.findall(pattern, line)
            for match in matches:
                data = (int(match[0]), int(match[1]))
                way = int(match[2])
                if data in count_dict:
                    count_dict[data] += way
                else:
                    count_dict[data] = way

    return count_dict

# 提供文件路径调用函数提取数据并统计
count_dict = extract_data('sram.txt')

print("Matched data counts:")
print("(Data Width | Set | Num)")
for data, count in count_dict.items():
    print(data, ":", count)


total_bits_sum = 0

for data, count in count_dict.items():
    value1, value2 = data
    product = value1 * value2 * count
    total_bits_sum += product

print("Total bits sum:", total_bits_sum)

with open('sram_type.txt', 'w') as f:
    for data, count in count_dict.items():
        f.write(str(data) + ' ' + str(count) + '\n')