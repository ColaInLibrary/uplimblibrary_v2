imprint
v1.0.4 —— 临时版本for化工项目半身，moveL控制腰+手臂，只能单臂使用。
v1.0.5 —— 新增末端负载设置
v1.0.6 —— 完善单边S形速度规划（speedL、 speedJ）

最后的提价日志记录：
commit 6304be16b89c4b967149192ca4f97dccf4e98e7b
Author: ColaInLibrary <1901151471@qq.com>
Date:   Thu Nov 13 11:05:19 2025 +0800

    [fix] 修复了Mujoco中显示异常，程序无法正常退出的bug

commit 47e1b099367e8e6a30ad2c34e3451126c4a5760b
Author: ColaInLibrary <1901151471@qq.com>
Date:   Thu Nov 13 10:31:01 2025 +0800

    [Refactor] 增加了机器人类型适配、Mujoco代码拆分

commit c297ed205b3169ae3582641d269cd3d01dc89c7b
Author: anhao <h.an@zj-humanoid.com>
Date:   Wed Nov 5 14:40:58 2025 +0800

    [fix]  2025.11.04新增雅可比矩阵以及更换WA2urdf

commit 43ddac689c5f3ff378a36d48cc9077880fb43fd6 (origin/yhq)
Author: anhao <h.an@zj-humanoid.com>
Date:   Thu Oct 30 09:54:13 2025 +0800

    version 1.0.6
    feat:增加末端负载设置
    fix:完善单边S形速度规划（speedL、speedJ）
