# Demo 源码保护

Demo 使用文件只读权限、.repo-lock 状态标记和本机 pre-commit 钩子管理源码修改。实验通过仓库外工作副本执行，锁定状态支持正常验收。

## 查看状态

```bash
./scripts/repo_guard.py status
```

输出包含 locked、marker、files 和 writable；文件数量随提交变化。

## 授权修改

取得修改授权后，填写原因与确认词：

```bash
./scripts/repo_guard.py unlock --reason "修改说明" --confirm UNLOCK
```

解锁恢复文件写权限，并将时间、原因、用户写入本机 .git/repo_access.log。修改并提交后恢复只读，再推送：

```bash
./scripts/repo_guard.py lock --reason "修改已完成"
git push origin main
```

## 本机状态

.repo-lock、访问日志、Git 钩子和文件写权限由本机维护。新克隆可通过 repo_guard.py 启用文件保护，并按团队流程配置提交钩子。该机制面向同一系统用户下的误改防护，授权修改通过显式解锁完成。

run.sh 默认将工作副本和结果写入仓库外的 ../Demo_runs/，源码保持只读。
