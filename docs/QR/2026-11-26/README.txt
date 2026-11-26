回报_2026-11-26_今日进展.md · RBQ2 分片
共 3 张，批次 B218B20E，PPMd8 order=16（PG），Base44 载荷
原始文件 17384 字节；压缩后 7124 字节

1) 逐张扫描，每行形如 RBQ2:01/03:B218B20E:PG:...
   头部固定 23 个字符，载荷里会有冒号和斜杠，不能按冒号切。
2) 把扫到的所有行存成 chunks.txt（每行一条）。
3) 解码：python3 QR/extract_rbq2.py chunks.txt --name 回报_2026-11-26_今日进展.md
   需要 pyppmd；仓库 QR/vendor 已内置。

all.png = 3 张排一起；print_A4.pdf = 每页一张，可直接打印。

原始文件 sha256  d7abe605ada1650ce2e6e7658d65e7b6e1ce30f02fc22e4aaa61c12e6ef05afd
压缩载荷 sha256  b218b20ee19120ed15b519886d60911262495fdaa8594099dbf07c071a743fdf
