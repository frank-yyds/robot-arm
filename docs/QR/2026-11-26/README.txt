回报_2026-11-26_今日进展.md · RBQ2 分片
共 4 张，批次 5769EE95，PPMd8 order=16（PG），Base44 载荷
原始文件 18812 字节；压缩后 7668 字节

1) 逐张扫描，每行形如 RBQ2:01/04:5769EE95:PG:...
   头部固定 23 个字符，载荷里会有冒号和斜杠，不能按冒号切。
2) 把扫到的所有行存成 chunks.txt（每行一条）。
3) 解码：python3 QR/extract_rbq2.py chunks.txt --name 回报_2026-11-26_今日进展.md
   需要 pyppmd；仓库 QR/vendor 已内置。

all.png = 4 张排一起；print_A4.pdf = 每页一张，可直接打印。

原始文件 sha256  ab448f39a5dabceac4416437a70f6ed28ec9b1134c32b31b9a145fd41aa1f368
压缩载荷 sha256  5769ee951cba1ea38c29a7bf2465a8e1d26c0423c9da140901a3d20019a14a42
