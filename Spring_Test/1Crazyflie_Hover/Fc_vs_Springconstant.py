import numpy as np
import matplotlib.pyplot as plt

# Data
F_c = np.array([10.1, 10.96, 12.08, 12.428, 13.67, 15.5, 16.08, 17.34, 17.7, 19.26, 20.48])
spring_constant = np.array([1.905905, 1.415, 1.322, 1.311, 1.383, 1.3, 1.272, 1.242, 1.249, 1.181, 1.129])

# 2차 다항회귀 계수 계산
coeffs = np.polyfit(F_c, spring_constant, 2)
poly = np.poly1d(coeffs)

# 회귀 곡선 생성
x_fit = np.linspace(F_c.min(), F_c.max(), 200)
y_fit = poly(x_fit)

# 그래프 그리기
plt.figure()
# 원본 데이터 (파란색)
plt.plot(F_c, spring_constant, marker='o', linestyle='-', color='tab:blue', label='Original Data')
# 회귀 곡선 (빨간색)
plt.plot(x_fit, y_fit, linestyle='--', color='tab:red', label='Quadratic Fit')
plt.xlabel('Weight (g)')
plt.ylabel('Spring Constant (N/m)')
plt.title('Spring Constant vs Weight with Quadratic Regression')
plt.legend()
plt.grid(True)
plt.show()
print(poly)