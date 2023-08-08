# -*- coding: utf-8 -*-

import math
import re

def generate_verilog_file(bits, word_depth, add_width):
    file_name = 'TS5N28HPCPLVTA{}X{}M2F.v'.format(word_depth, bits)
    with open(file_name, 'w') as f:
        with open('../sram_template.v', 'r') as template_file:
            template = template_file.read()
            template = template.replace('TS5N28HPCPLVTA128X64M2F', 'TS5N28HPCPLVTA{}X{}M2F'.format(word_depth, bits))
            template = template.replace('Bits = 64', 'Bits = {}'.format(bits))
            template = template.replace('Word_Depth = 128', 'Word_Depth = {}'.format(word_depth))
            template = template.replace('Add_Width = 7', 'Add_Width = {}'.format(add_width))
            f.write(template)

def process_parameter_file(file_path):
    with open(file_path, 'r') as parameter_file:
        lines = parameter_file.readlines()
        for line in lines:
            line = line.strip()
            if re.match(r'\(\d+,\s*\d+\)', line):
                params = line[1:-1].split(',')
                bits = int(params[0].strip())
                word_depth = int(params[1].strip())
                add_width = int(math.ceil(math.log(word_depth, 2)))
                print(bits, word_depth, add_width)
                generate_verilog_file(bits, word_depth, add_width)

# 提供参数文件路径调用函数处理参数并生成相应的Verilog文件
process_parameter_file('../sram_type.txt')
