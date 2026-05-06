# 安信可 UWB PDOA Viewer V1.0

基于 Web 的 UWB PDOA 实时定位可视化工具，替代安信可原版上位机软件。

## 功能

- 实时雷达地图（HiDPI 高清渲染）
- 多标签追踪 + 轨迹显示
- 双串口支持（USB 数据口 + TTL AT 指令口）
- JSON / HEX 协议解析
- 角度偏移 & 距离校正
- AT 指令终端
- 数据记录导出
- 可折叠侧栏 + 浮窗实时数据

## 在线预览

打开 GitHub Pages 即可查看界面：  
👉 https://shan-hou.github.io/uwb-pdoa-viewer/

> 注意：在线版仅展示界面，实际连接 UWB 设备需运行本地服务器。

## 本地运行

```bash
pip install -r requirements.txt
python server.py
```

浏览器自动打开 `http://localhost:8080`，连接 BU04 USB 端口即可使用。

## 硬件

- **BU04** — UWB 基站（PDOA）
- **BU03** — UWB 标签
