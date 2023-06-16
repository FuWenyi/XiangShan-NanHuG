# -*- coding: utf-8 -*-

import math

def generate_scala_files(template_code, parameter_file):
    with open("scala_sram.txt", 'w') as output_file:
        with open(parameter_file, 'r') as f:
            lines = f.readlines()
            for line in lines:
                params = line.strip()[1:-1].split(',')
                bits = int(params[0].strip())
                word_depth = int(params[1].strip())
                add_width = int(math.ceil(math.log(word_depth, 2)))

                verilog_code = template_code.replace("32", str(bits))
                verilog_code = verilog_code.replace("128", str(word_depth))
                verilog_code = verilog_code.replace("7", str(add_width))

                output_file.write(verilog_code)

# 读取模板代码文件
with open("sram_scala_template.txt", 'r') as template_file:
    template_code = template_file.read()

# 提供参数文件路径调用函数生成代码文件
generate_scala_files(template_code, "sram_type.txt")
