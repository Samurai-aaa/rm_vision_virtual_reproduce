
---

### `04_camera_info_and_pnp.md`

PnP。

```markdown
# Camera Info and PnP

PnP 的作用是根据装甲板 2D 图像角点和真实 3D 尺寸，解算装甲板相对于相机的位置

## 输入

```text
装甲板 2D 四角点
装甲板真实尺寸
相机内参 K
畸变参数 D

```
## 输出

```text
rvec
tvec