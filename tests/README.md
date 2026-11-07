# 滤波器宿主机测试

在 controller 仓库根目录使用支持 C++23 的宿主机编译器运行：

```sh
g++ -std=c++23 -O2 -Wall -Wextra -Werror -Iinclude tests/filter_test.cc src/biquad_filter.cc src/highpass_filter.cc src/bandpass_filter.cc src/bandstop_filter.cc -o filter_test
./filter_test
```

Windows 可将输出文件指定为 `filter_test.exe`，再运行 `./filter_test.exe`。测试不依赖 BSP、RTOS 或 ARM 工具链，不要定义 `NDEBUG`。

覆盖非法参数、NaN/Inf、IIR 溢出后的状态保留、清空和重新配置、窗口启动和环绕、正常参数的冲激响应与 -3 dB 边界，以及低截止频率、窄带和接近 Nyquist 的配置。

第二轮 PR 的三个失真案例必须返回配置失败，并保持原系数和历史：

- `high_pass_filter::set_params(0.02f, 1000.f)`
- `band_pass_filter::set_params(0.1f, 0.1001f, 1000.f)`
- `band_stop_filter::set_params(0.1f, 0.1001f, 1000.f)`

配置成功需要最终 float 系数通过 Jury 稳定性检查，并满足下列频点的幅值绝对误差不超过 `0.01`（单位通带增益的 1%）：

| 滤波器 | 检查频点及目标幅值 |
| --- | --- |
| 高通 | DC：0，截止频率：1/√2，Nyquist：1 |
| 带通 | DC/Nyquist：0，两个边界：1/√2，预畸变中心：1 |
| 带阻 | DC/Nyquist：1，两个边界：1/√2，预畸变中心：0 |

数字中心角频率为 `2*atan(sqrt(tan(pi*low/fs)*tan(pi*high/fs)))`。支持范围由最终系数的稳定性及上述频点的误差共同决定，不对整个频谱或所有频率/带宽组合承诺统一误差界限。构造配置失败时为直通；运行时配置失败则保留原状态。校验仅在配置阶段执行，`update()` 仍使用原 float 递推。

测试还在三个采样频率下扫描 270 组配置，以独立的复数多项式计算核验所有通过配置的关键频点；测试预期同时存在可接受与被拒绝的参数。
