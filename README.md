# 2022秋计组实验

## 代码（/source）

1. SCCPU为单周期CPU代码
2. MCCPU为多周期CPU代码
3. TestFile为测试代码

## 说明

实现的CPU支持34条MIPS指令(数据在内存中以小端形式存储)：

- add/sub/and/or/slt/sltu/addu/subu
- addi/ori/lw/sw/beq
- j/jal
- sll/nor/lui/slti/bne/andi/srl/sllv/srlv/jr/jalr
- xor/sra/srav
- lb/lh/lbu/lhu/sb/sh

(最后一个指令sh实现的好像有点问题，代码历史久远，未debug)

## 参考文献

1. [支持 45 条 MIPS 指令的单周期处理器开发](https://blog.triplez.cn/posts/support-45-mips-instructions-single-cycle-cpu-dev/)
2. [多周期CPU设计与实现](https://blog.csdn.net/lllllyt/article/details/80854967)
