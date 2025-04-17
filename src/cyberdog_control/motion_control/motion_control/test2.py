import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 读取CSV文件
df = pd.read_csv('rgb_values.csv')

# 转换为 numpy 数组并 reshape 成图像尺寸 (480, 640, 3)
rgb_array = df.to_numpy().reshape((480, 640, 3)).astype(np.uint8)

# 显示图像
plt.imshow(rgb_array)
plt.axis('off')
plt.show()

