# 实验配置

`tracking_wind.json` 是带吊载、SPAD 平滑阻尼、2 m/s 水平风、CTBR 及受约束旋翼分配的教学算例。
所有系数均显式记录；不代表任何实机辨识结果。模型世界系 Z-up，旋翼位置相对整机质心。

```bash
.venv/bin/python examples/python/tracking.py --config configs/experiments/tracking_wind.json --output-dir runs/wind
.venv/bin/python examples/python/replay_experiment.py runs/wind
```

加入 `--headless` 关闭显示；改变 `--render-every` 不改变物理结果。输出目录必须不存在。
完整配置、记录格式和指标见 [实验文档](../../docs/reproducible-experiments.md)。

配置顶层 `model` 仅保存物理参数，`control` 保存 Python `csim_control.ControlConfig` 参数。原生 Model 不接受控制配置。
