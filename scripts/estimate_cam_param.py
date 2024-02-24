import argparse
import cv2
import numpy as np
import copy
import yaml

ori_value = {
    'theta_xy': 250,
    'theta_z': 500,
    'focal': 200,
    'Tz': 200
}


class CameraPara:

    def open(self, file):   
        self.data = yaml.load(open(file), yaml.SafeLoader)
        self.Ki = np.array(self.data["Ki"]).reshape([3, 4])
        self.Ko = np.array(self.data["Ko"]).reshape([4, 4])

    def xy2uv(self, x, y):
        uv = np.dot(self.Ki, np.dot(self.Ko, np.array([x,y,0,1])))
        uv /= uv[2]
        return int(uv[0]), int(uv[1])
    

def xy2uv(x,y,Ki,Ko):
    # 计算uv
    uv = np.dot(Ki, np.dot(Ko, np.array([x,y,0,1])))
    # 归一化
    uv = uv/uv[2]
    return int(uv[0]), int(uv[1])


# 定义转换函数
def get_real_theta(value):
    return (value - ori_value["theta_xy"]) / 10.0

def get_real_theta_z(value):
    return (value - ori_value["theta_z"]) / 5.0

def get_real_focal(value):
    return (value - ori_value["focal"]) * 5

def get_real_transition(value):
    return (value-ori_value["Tz"]) * 0.04


# 定义滑动条回调函数
def update_value_display():
    global value_display,g_theta_x,g_theta_y,g_theta_z,g_focal,g_tz
    value_display.fill(0)  # 清空图像
    text = f"theta_x: {g_theta_x:.2f}"
    cv2.putText(value_display, text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
    text = f"theta_y: {g_theta_y:.2f}"
    cv2.putText(value_display, text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
    text = f"theta_z: {g_theta_z:.2f}"
    cv2.putText(value_display, text, (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
    text = f"focal: {g_focal}"
    cv2.putText(value_display, text, (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
    text = f"Tz: {g_tz:.2f}"
    cv2.putText(value_display, text, (10, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, 255, 2)
    cv2.imshow('Values', value_display)

def on_theta_x_change(value):
    global g_theta_x
    g_theta_x = get_real_theta(value)
    update_value_display()
    
def on_theta_y_change(value):
    global g_theta_y
    g_theta_y = get_real_theta(value)
    update_value_display()

def on_theta_z_change(value):
    global g_theta_z
    g_theta_z = get_real_theta_z(value)
    update_value_display()

def on_focal_change(value):
    global g_focal
    g_focal = get_real_focal(value)
    update_value_display()

def on_tz_change(value):
    global g_tz
    g_tz = get_real_transition(value)
    update_value_display()

cv2.namedWindow('Values')
# 初始化一个空白图像来显示实际值
value_display = np.zeros((400, 300), dtype=np.uint8)
g_theta_x = 0
g_theta_y = 0
g_theta_z = 0
g_focal = 0
g_tz = 0

def main(args):

    cam_para = CameraPara()
    cam_para.open(args.cam_para)

    if args.img is not None:
        img = cv2.imread(args.img)
    else:
        cap = cv2.VideoCapture(args.vid)
        import os.path as osp
        cap.set(cv2.CAP_PROP_POS_FRAMES, args.id)
        success, img = cap.read()
        assert success and img is not None
    # 获取img的大小
    height, width = img.shape[:2]
    ori_img = img.copy()

    cv2.namedWindow('CamParaSettings')
    # 添加ui界面来修改theta_x,theta_y,theta_z, 调节访问是-10到10，间隔0.2
    cv2.createTrackbar('theta_x', 'CamParaSettings', ori_value["theta_xy"],500, on_theta_x_change)
    cv2.createTrackbar('theta_y', 'CamParaSettings', ori_value["theta_xy"],500, on_theta_y_change)
    cv2.createTrackbar('theta_z', 'CamParaSettings', ori_value["theta_z"],1000, on_theta_z_change)
    cv2.createTrackbar('focal', 'CamParaSettings', ori_value["focal"],500, on_focal_change)
    cv2.createTrackbar('Tz', 'CamParaSettings', ori_value["Tz"],500, on_tz_change)

    global g_theta_x,g_theta_y,g_theta_z

    # 循环一直到按下q or esc
    while True:
        theta_x = g_theta_x/180.0*np.pi
        theta_y = g_theta_y/180.0*np.pi
        theta_z = g_theta_z/180.0*np.pi
        Ki = copy.copy(cam_para.Ki)
        Ko = copy.copy(cam_para.Ko)
        R = Ko[0:3,0:3]
        Rx = np.array([[1,0,0],[0,np.cos(theta_x),-np.sin(theta_x)],[0,np.sin(theta_x),np.cos(theta_x)]])
        Ry = np.array([[np.cos(theta_y),0,np.sin(theta_y)],[0,1,0],[-np.sin(theta_y),0,np.cos(theta_y)]])
        Rz = np.array([[np.cos(theta_z),-np.sin(theta_z),0],[np.sin(theta_z),np.cos(theta_z),0],[0,0,1]])
        R = np.dot(R, np.dot(Rx, np.dot(Ry,Rz)))
        Ko[0:3,0:3] = R
        Ko[2,3] += g_tz
        Ki[0,0] += g_focal
        Ki[1,1] += g_focal

        img = ori_img.copy()
        # x取值范围0-10，间隔0.1
        for x in np.arange(0,10,0.5):
            for y in np.arange(-5,5,0.5):
                u,v = xy2uv(x,y,Ki,Ko)
                cv2.circle(img, (u,v), 3, (0,255,0), -1)

        # 修改img的大小
        img = cv2.resize(img, (int(width*0.5),int(height*0.5)))
        cv2.imshow('img', img)
        key = cv2.waitKey(50)
        if key in [ord('q'), 27] :
            break
    
    if args.test_only:
        return

    ki_save = []
    for item in Ki.tolist():
        ki_save.extend(item)
    
    ko_save = []
    for item in Ko.tolist():
        ko_save.extend(item)
    
    yaml.dump({
        "Ki": str(ki_save),
        "Ko": str(ko_save),
        "a1": cam_para.data.get("a1", 100.0),
        "a2": cam_para.data.get("a2", 100.0),
        "wx": cam_para.data.get("wx", 5.0),
        "wy": cam_para.data.get("wy", 5.0),
        "vmax": cam_para.data.get("vmax", 5.0),
        "max_age": cam_para.data.get("max_age", 10.0),
        "high_score": cam_para.data.get("high_score", 0.5),
        "conf_threshold": cam_para.data.get("conf_threshold", 0.01)
    }, open(args.cam_para, "w"), yaml.Dumper)
    save_str = open(args.cam_para).read()
    save_str = save_str.replace("'", "")
    open(args.cam_para, "w").write(save_str)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some arguments.')
    parser.add_argument('-i', '--img', type=str, default=None, help='The image file')
    parser.add_argument('-v', '--vid', type=str, default=None, help='The video file')
    parser.add_argument('--id', type=int, default=0, help='The frame id of video')
    parser.add_argument('-f', '--cam_para', type=str, required=True,help='The estimated camera parameters file ')
    parser.add_argument("-t", "--test-only", action="store_true")
    args = parser.parse_args()
    assert (args.img or args.vid) is not None
    main(args)
