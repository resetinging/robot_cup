# 协作规则（本仓库）

## 语言

- 所有回答一律使用中文。

## 变更流程

1. 收到请求后，先做规划，不要立即修改文件或执行写操作。
2. 规划过程中若有不清楚的地方，先向用户提问确认。
3. 规划完成后，先把规划内容交给用户过目，得到用户明确同意后再执行修改。
4. 每次更新（代码、构建配置等任何改动）后，按以下顺序执行：
   - 先 review：自查改动内容，列出改动点、影响范围与风险；
   - 再编译：执行构建验证（至少 Debug，必要时 Release），报告结果与告警；
   - 然后停下等待用户通知：在用户明确通知之前，不得执行 `git push`；
     本地提交（`git commit`）前也要先向用户汇报改动清单并征得同意。

## 当前工程备注

- 原 Keil MDK 工程位于 `MDK-ARM/`，保持可用，不要删除或覆盖。
- CMake/GCC 工程入口为根目录 `CMakeLists.txt`、`CMakePresets.json`、`cmake/target.cmake`，
  该结构用于被 STM32CubeIDE for Visual Studio Code 扩展识别。
- 器件信息以 `cmake/target.cmake` 中的 `CMSIS_*` 变量为准（`CMSIS_Dname` 必须是 CMSIS Pack
  的器件名，如 `STM32F407VGTx`，而不是订货号 `STM32F407VGT6`）。
- 构建产物目录为 `build/`（已加入 `.gitignore`），构建类型为 `Debug` / `Release`。
