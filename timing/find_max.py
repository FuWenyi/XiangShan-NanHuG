# -*- coding: utf-8 -*-

filename = "way.rpt"  # 替换成您的文件名

max_value = None

with open(filename, 'r') as file:
    for line in file:
        value = float(line.split()[-1])
        if max_value is None or abs(value) > abs(max_value):
            max_value = value

if max_value is not None:
    print(abs(max_value))
else:
    print("文件为空或没有找到值")