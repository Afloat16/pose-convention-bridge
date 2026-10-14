# pose-convention-bridge

[English](README.md) · **简体中文**

显式转换 OpenCV / OpenGL 和 w2c / c2w 位姿；TUM 使用 xyzw，COLMAP 使用 wxyz。不猜坐标轴、外参、单位或世界变换，不伪造图片名与内参；点云/网格需单独应用同一世界变换。

由 **Afloat16** 独立维护，MIT 许可。实验性工具，格式与限制详见 [English README](README.md) 和[需求证据](docs/EVIDENCE.md)。未声称修复上游运行时问题或获得上游认可。

## 安装与首个示例

Python 3.10+。克隆仓库、进入目录，创建并激活虚拟环境后：

```bash
python -m pip install -e '.[dev]'
pose-convention-bridge examples/poses.json --input-direction c2w --input-axes opencv --format nerfstudio --output transforms.json
```

基础离线路径使用标准库，无需 GPU、ROS 运行环境或模型权重。输入保持只读，输出必须是新文件，不覆盖已有文件。位姿转换有效完成返回 0，输入或写入失败返回 2。

## 验证

```bash
python -m unittest discover -s tests -v
python -m compileall -q src tests
python -m build
```

CI 配置覆盖三种系统的 Python 3.10 / 3.13，实际状态以 Actions 为准。示例均为合成数据，不包含用户附件或私人数据。


## 相关项目

按问题选择工具，查看[Afloat16项目导航](https://github.com/Afloat16/research-to-fix/blob/main/docs/PROJECTS.zh-CN.md)。
