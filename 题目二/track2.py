import cv2
import numpy as np
import math


def visualize_path_planning(image_path):
    # 1. 图像读取与校验（增加错误处理）
    try:
        img = cv2.imread(image_path)
        if img is None:
            raise FileNotFoundError("无法读取图像文件")

        height, width = img.shape[:2]
        center_line = width // 2

        # 2. 精确起点检测（增强鲁棒性）
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 改进的红色检测（包含深红和亮红）
        red_mask = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255])) | \
                   cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))

        # 形态学处理确保起点完整
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        # 获取起点坐标（带异常处理）
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            raise ValueError("未检测到起点")

        M = cv2.moments(max(contours, key=cv2.contourArea))
        if M["m00"] == 0:
            raise ValueError("起点区域面积为零")

        start_point = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # 3. 赛道检测（优化参数）
        yellow_mask = cv2.inRange(hsv, np.array([15, 80, 80]), np.array([30, 255, 255]))
        yellow_mask[:, width - 150:] = 0  # 去除右侧干扰

        # 增强赛道连续性
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)

        # 4. 斜向路径规划（从起点到中线）
        path = []
        current_pos = start_point
        step = 15
        target_reached = False

        while not target_reached and len(path) < 100:  # 防止无限循环
            # 计算移动方向（右斜向上）
            dx = min(step, center_line - current_pos[0])  # 确保不越过中线
            dy = -step  # 向上移动

            new_pos = (current_pos[0] + dx, current_pos[1] + dy)

            # 检查新位置有效性
            if (0 <= new_pos[0] < width and 0 <= new_pos[1] < height and
                    yellow_mask[new_pos[1], new_pos[0]] > 0):
                path.append(new_pos)
                current_pos = new_pos
            else:
                # 尝试更垂直的路径
                new_pos = (current_pos[0], current_pos[1] - step)
                if yellow_mask[new_pos[1], current_pos[0]] > 0:
                    path.append(new_pos)
                    current_pos = new_pos

            # 终止条件：接近中线且足够高
            if abs(current_pos[0] - center_line) < 20 and current_pos[1] < height // 2:
                target_reached = True

        # 5. 可视化（增强显示效果）
        result = img.copy()

        # 绘制主路径（亮绿色粗线）
        if len(path) > 1:
            pts = np.array(path, np.int32).reshape((-1, 1, 2))
            cv2.polylines(result, [pts], False, (50, 255, 50), 4)

        # 标记关键点
        cv2.circle(result, start_point, 12, (0, 0, 255), -1)  # 红色起点
        cv2.line(result, (center_line, 0), (center_line, height), (255, 0, 0), 2)  # 蓝色中线

        # 添加方向箭头
        if len(path) > 10:
            arrow_start = path[-5]
            arrow_end = (path[-5][0] + 50, path[-5][1] - 70)
            cv2.arrowedLine(result, arrow_start, arrow_end, (255, 100, 255), 4, tipLength=0.3)

        # 添加标题文本
        cv2.putText(result, "Optimized Path Planning", (width // 4, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        return result

    except Exception as e:
        print(f"处理错误: {str(e)}")
        # 创建错误提示图像
        error_img = np.zeros((300, 500, 3), np.uint8)
        cv2.putText(error_img, f"Error: {str(e)}", (50, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return error_img


# 使用示例（带显示和保存功能）
if __name__ == "__main__":
    input_image = "saidao_2.png"  # 替换为您的图像路径

    # 执行路径规划
    result_img = visualize_path_planning(input_image)

    # 显示结果
    cv2.imshow("Path Planning Result", result_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # 保存结果
    cv2.imwrite("path_planning_result.jpg", result_img)
    print("结果已保存为 path_planning_result.jpg")