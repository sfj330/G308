import cv2
import numpy as np


def detect_track_with_red_contour(image_path):
    # 读取图像
    img = cv2.imread(image_path)
    if img is None:
        print("无法读取图像")
        return None

    # 转换为HSV颜色空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # 精确的黄色HSV范围（根据您的土黄色/赭石色调整）
    lower_yellow = np.array([15, 70, 70])  # 色调范围15-30
    upper_yellow = np.array([30, 255, 255])

    # 创建掩模
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 改进的形态学操作（针对您描述的黑色边线）
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)  # 去除黑色边线
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)  # 填充内部

    # 查找轮廓（只找最外层轮廓）
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 在原图上用红色绘制轮廓（BGR格式的红色是(0,0,255)）
    result = img.copy()
    cv2.drawContours(result, contours, -1, (0, 0, 255), 3)  # 红色轮廓，线宽3

    # 可选：添加文字标注
    if contours:
        cv2.putText(result, "Track Area", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    return result


# 使用示例
result_image = detect_track_with_red_contour('saidao.jpeg')

# 显示结果（如果不在PyCharm中运行，使用以下代码）
cv2.imshow('Track Detection', result_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

# 保存结果
cv2.imwrite('track_with_red_contour.jpg', result_image)