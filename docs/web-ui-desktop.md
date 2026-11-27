# 外接显示器上的 WebUI

Ubuntu 24.04 的本地图形会话可在 HDMI、DP、DVI 或 VGA 外屏接入后，在该屏打开 WebUI。窗口用 GTK 4 和嵌入式 WebKitGTK 渲染，只打开 `127.0.0.1` 上的工作台，不启动独立浏览器。拔掉最后一块外屏后关闭窗口并释放网页进程；不影响后台 WebUI、导航进程或远程访问。

在已有 WebUI 服务和 GNOME 等 systemd 图形会话的机器上，先安装依赖，再以图形会话用户运行：

```bash
sudo apt install python3-gi gir1.2-gtk-4.0 gir1.2-webkit-6.0
cd ~/yang_nav
bash tools/install_web_ui_desktop_service.sh
```

如果日志出现 `bwrap: setting up uid map: Permission denied` 或 `Failed to fully launch dbus-proxy`，Ubuntu 的 AppArmor 可能在限制 WebKit 沙箱创建用户命名空间。保持全局限制不变；以管理员身份安装 `tools/web_ui/yang-nav-web-ui-desktop.apparmor` 到 `/etc/apparmor.d/yang-nav-web-ui-desktop`，将配置里的可执行文件路径改为 `tools/web_ui/desktop.py` 的实际绝对路径，执行 `sudo apparmor_parser -r /etc/apparmor.d/yang-nav-web-ui-desktop`，然后重启下面的用户服务。

默认端口为 `8080`，非默认端口可作为安装脚本的第一个参数。开机免操作弹窗需要图形会话已自动登录；安装脚本不会更改登录设置。外屏需有 EDID 且已经由桌面启用；可在系统显示设置中启用已接入的外屏。带 EDID 的 HDMI/DP 假负载可能被系统当成外屏。

按 `Alt+F4` 可关闭窗口，接入外屏后会再次弹出。要立即重新打开、查看日志或关闭此功能，可分别执行：

```bash
systemctl --user restart yang-nav-web-ui-desktop.service
journalctl --user -u yang-nav-web-ui-desktop.service -b
systemctl --user disable --now yang-nav-web-ui-desktop.service
```
