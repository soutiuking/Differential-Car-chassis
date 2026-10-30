# PCB 工程管理说明

## 1. 源文件

PCB 源工程：

```text
ProPrj_小车底盘_2026-10-30.epro
```

这是 EDA 工程归档，内部包含：

- `SHEET/`：原理图
- `PCB/`：PCB 布局
- `SYMBOL/`、`FOOTPRINT/`：符号和封装
- `project.json`：工程索引

`.epro` 是**源文件**，需要纳入 Git。它本身是 ZIP 归档，Git 只能按二进制文件管理，不能像文本代码那样查看逐行 diff。

## 2. 构建产物不纳入 Git

以下文件/目录是 EDA 导出的制造或预览产物，不提交：

```text
*.gbr
*.gbrjob
*.drl
*.nc
*.gtl / *.gbl / *.gts / *.gbs / *.gto / *.gbo / *.gko / *.gm1
*.rpt
/Gerber/
/Gerbers/
/Fabrication/
/fabrication/
/PCB_Export/
/pcb_output/
```

它们已经在仓库根目录 `.gitignore` 中排除。

建议把 Gerber/BOM/坐标文件导出到仓库外的目录，或者导出到上述忽略目录；发布 PCB 版本时把制造包作为 Release 附件，而不是提交进 Git。

## 3. Git 操作

首次加入：

```powershell
cd D:\guangdian
git add 'ProPrj_小车底盘_2026-10-30.epro' .gitignore .gitattributes
git commit -m "docs(pcb): add PCB source project"
```

查看状态：

```powershell
git status
git ls-files | Select-String -Pattern '\.epro$'
```

确认 Gerber 没有进入版本库：

```powershell
git status --ignored --short | Select-String -Pattern 'gbr|drl|Gerber|Fabrication'
```

## 4. 版本建议

- 原理图/PCB 每完成一个可评审版本，提交一次 `.epro`；
- 提交信息写清楚改了什么，例如：
  - `pcb: add power input protection`
  - `pcb: route TB6612 motor outputs`
  - `pcb: fix MPU6050 I2C pull-up`
- 发布打 tag，例如 `pcb-v0.1.0`；
- 制造包放在 Release 附件，文件名带日期和版本。

## 5. 注意

- 不要直接把编辑器自动保存的临时文件、备份文件提交。
- 如果 EDA 自动生成带日期的新 `.epro` 文件，建议只保留一个当前源文件并在提交信息里记录日期，避免仓库里堆积多个近似二进制归档。
- 原理图/PCB 的评审截图或 PDF 可以作为文档提交；Gerber/钻孔/BOM 等制造产物不提交。
