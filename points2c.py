#!/usr/bin/env python3
"""
points2c.py  —  把“x y”形式的文本点转为 C 的 float arr[][2] 初始化代码
用法:
    python3 points2c.py points.txt > points.h
"""

import sys
import pathlib

def main():
    if len(sys.argv) < 2:
        print('用法: python3 points2c.py <输入文件>')
        sys.exit(1)

    lines = pathlib.Path(sys.argv[1]).read_text().splitlines()
    points = []
    for ln in lines:
        ln = ln.strip()
        if not ln or ln.startswith('#'):       # 忽略空行和注释
            continue
        x, y = map(float, ln.split())
        points.append((x, y))

    print('// 由 points2c.py 自动生成，共 %d 个点' % len(points))
    print('float points[%d][2] = {' % len(points))
    for x, y in points:
        print('    { %.6ff, %.6ff },' % (x, y))
    print('};')

if __name__ == '__main__':
    main()