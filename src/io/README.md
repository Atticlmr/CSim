# 模型导入实现

`model_import.cpp` 是 TinyXML2 DOM 与严格 URDF/MJCF 子集解析，`mesh.cpp` 读取无纹理 OBJ/STL。
`src/model/rigid_body.cpp` 独立验证描述并将固定部件合并为整机质心处刚体。
它们构成 `csim_io` 静态目标，链接系统 TinyXML2；公开描述不暴露 XML DOM。
Python 入口为 `load_model`，范围、坐标和错误语义见 [模型导入](../../docs/model-import.md)。
测试位于 `tests/python/test_model_import.py` 和 `tests/viewer/python_viewer_test.py`。
TODO：扩展材质/资源、defaults/include 与通用关节之前，先定义对应物理语义。
