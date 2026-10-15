# 数据集清单

[返回](README.md) · 更新：2026-10-15。这里只列资源，不下载整套数据；“有入口”不等于已取得访问权限。

| 数据集 / 来源 | 视角与标注 | 状态 / 用途 |
| --- | --- | --- |
| [iSAID](https://captain-whu.github.io/iSAID/dataset.html) | RGB 俯视；像素级实例/COCO；2,806 图、15 类；655,451 实例为所有类别合计 | 优先待测/异机训练来源；官方百度/Google Drive；学术非商业，包含 ship 与车辆 |
| [MariSat / P2M](https://github.com/amirabbes/P2M-Maritime-Segmentation) | 港口俯视；实例多边形/CVAT XML；1,260 图、8 船类，840/200/220 划分 | 待核实访问和许可；共享链接跳转微软登录，未取得整套数据；含货船/拖船等 |
| [UOW-Vessel](https://documents.uow.edu.au/~phung/UOW-Vessel.html) | 光学卫星俯视；实例多边形；3,500 图、35,598 船实例、10 类 | 待申请；填写协议后邮件获取；军用类别较多，先看民用船样例 |
| [Airbus Ship Detection](https://www.kaggle.com/competitions/airbus-ship-detection/data) | RGB 卫星俯视，768×768；逐船 RLE | 辅助候选；Kaggle 访问/竞赛条款待核；本机只有 1 张作者对照，不是完整数据集 |
| [ShipRSImageNet](https://github.com/zzndream/ShipRSImageNet) | 光学俯视；水平/旋转框为主；README 为 3,435 图、17,573 船实例、50 类，另有 V1.1 更新 | 检测候选；Google Drive/百度入口；polygons 是否为真实船体轮廓未解包核查；图像/标注限学术非商业 |
| [SeaDronesSee v2](https://seadronessee.cs.uni-tuebingen.de/) | 真 UAV，多俯仰角；8,930/1,547/3,750 图；检测/跟踪框 | UAV 检测补充，偏搜救人员/小艇；不是大型船体实例分割集 |
| [DOTA](https://captain-whu.github.io/DOTA/) | RGB 俯视遥感；ship、车辆等旋转框 | 匹配 OBB 定位；没有实例轮廓，与 iSAID 共用部分原图，不能跨集随机混作独立测试 |
| [ShipSG](https://github.com/DLR-MI/shipsg) | 固定港口斜视；实例 mask；3,505 图、11,625 船掩膜 | 非俯视优先；需申请访问，尚未确认专用现成模型权重 |
| [iSAID-Sliced-512](https://github.com/mihajlov39547/iSAID-Sliced-512) | 512×512 遥感切片，宣传为分割，README 示例却为 5 字段 bbox | 待核实真实标注；未验证为可直接训练 YOLO-seg 的数据，不替代 iSAID 原始实例标注 |
| COCO-Seg | 通用 RGB；含 boat 实例 | 通用预训练来源；此前 n-seg 原样推理失败，非海事俯视专用集；未下载整套数据 |
| HRSID / SSDD / TSSWD | SAR 俯视；具体框/实例格式依版本核查 | 非优先；不是当前 RGB 成像，不因同为俯视就直接使用 |
| SISP | 全色卫星；细粒度船舶实例研究 | 历史待核实；此前记录为尚未确认数据/权重可用，本轮未重新核查 |
| MODS / LaRS | USV 前视，分割/检测/全景任务依数据集 | 非优先；不能当 UAV 俯视实例数据；与 SeaDronesSee 同站托管不表示视角/任务相同 |
| [MTReD](https://github.com/RuiYiYong/MTReD) | 海事航拍图像，项目此前用于双帧重建对照 | 本机 4 张公开样例；没有随样例取得人工船体 mask 或米制三维真值，不作为有监督实例训练集 |
| Scene_Water 自采 bag | 海康 RGB，1280×1024，包含近景俯视船与车辆画面 | 本机保留原始 bag/抽帧；没有人工 bbox/mask/距离真值；标注和训练需另行明确 |

## 本机已有样本

| `datasets/` 子目录 | 内容 / 用途 |
| --- | --- |
| `scene_water_trial/` | 同一 bag 的 5/5.5/30/30.5 秒，4 张初期输入检查图 |
| `scene_water_audit/` | 60–600 秒的 11 张场景审计图 |
| `scene_water_yolo_clips/` | 180/300/420 秒附近的 36 张短窗口图；与 audit 去重后为 43 张本地图 |
| `mtred_ship/` | 4 张公开图，场景 01/03 各两张；历史对照用 `scene03_*.png` |
| `airbus_control/` | 作者 `test_image_1.jpg`；仅推理对照，不能当独立精度测试集 |

本地 bag：`/home/htfp/data/Scene_Water_20260805_013651`；只抽取相机 `/left_camera/image/compressed`。
`manifest.json` 是提帧来源和时间记录，不是标注；数据、bag 和环境不随 Git 同步。
