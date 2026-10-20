# Demo 仓库写保护

本仓库本机已启用写保护：所有受保护的源文件为只读，普通脚本或程序直接写入会
以 `Permission denied` 失败；本机提交也会被 `pre-commit` 钩子拒绝。Git 不分发本机钩子、写权限或 .repo-lock，新克隆需自行启用保护。

## 状态

```bash
./scripts/repo_guard.py status
# {"locked": true, "marker": true, "files": 624, "writable": 0}
```

## 解锁（必须显式授权）

解锁是可重新锁定的临时状态变化，必须已有修改授权，同时给出**原因**和**确认词**，且会打印警告横幅：

```bash
./scripts/repo_guard.py unlock --reason "修复 xxx" --confirm UNLOCK
```

解锁记录写入 `.git/repo_access.log`（不入版本库），包含时间、原因、用户。

## 重新锁定

修改并提交后立即重新锁定，再推送；文件数量随版本变化，以 status 实际输出为准：

```bash
./scripts/repo_guard.py lock --reason "修改已完成"
```

## 边界说明

这是**防误改**的强屏障，不是安全边界：所有进程以同一用户运行，任何脚本只要
显式调用 `unlock` 就能改。它的作用是让"不小心改到"变成"必须故意违规"，并且
每次解封都留痕。

## 验收运行不受影响

`./run.sh` 只在仓库外的 `../Demo_runs/` 建工作副本，不写本仓库；因此锁定状态
下仍可正常跑验收。
