# Demo 代码替换记录（2026-10-20）

按用户明确授权，将 Traffic_Arrangement 当前代码替换到 Demo，保留 Demo 的 Git 历史。原版本完整备份在 Demo_runs/replacement_backup_* 下，包含 Git 数据及原权限。

Demo 默认绑定原工程 `/home/aipc/Traffic_ue4/Traffic_ue4.uproject`，默认结果目录为仓库外 `../Demo_runs/`。从 Demo 运行 `./run.sh`。如明确设置 TRAFFIC_UE_PROJECT_DIR，会使用该覆盖路径；不带该变量时使用原 Demo 工程。启动时核对实际编辑器工程路径与已加载原生库，不能用 agent 工程替代。

本次只替换仓库代码，不修改 UE 工程、原生库、资产、鼠标设置或桌面启动器。校验后恢复 Demo 文件只读权限。

验收边界：Traffic_Arrangement 同一版本 10 次独立冷启动结果为 9 次通过、1 次等待环停滞失败，证据位于 ../Traffic_Arrangement_Agent_runs/cold_start_10_20261020_130553/REPORT.md。这些结果针对 agent 工程，不能视为替换后的原 Demo UE 工程已完成多轮验收。迁移没有修复剩余停滞问题，也不保证每次成功。

STATE_TRANSITIONS.md 描述当前 Demo 的校验实现，VALIDATION_20261020.md 中的 agent 工程与外部日志路径描述迁移前验证背景。当前冻结清单已刷新为新版实际源码摘要，旧摘要由 Git 历史保留；冻结不等于新版物理验收通过。
