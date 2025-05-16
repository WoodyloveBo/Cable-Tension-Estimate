import numpy as np
import matplotlib.pyplot as plt

# Data
weight = np.array([10.1, 10.96, 12.08, 12.428, 13.67, 15.5, 16.08, 17.34, 17.7, 19.26, 20.48])
spring_constant = np.array([2.303, 2.0279, 1.8225, 1.6469, 1.615, 1.2258, 1.1856, 1.2146, 1.1808, 1.1309, 1.1146])

# 2차 다항회귀 계수 계산
coeffs = np.polyfit(weight, spring_constant, 2)
poly = np.poly1d(coeffs)

# 회귀 곡선 생성
x_fit = np.linspace(weight.min(), weight.max(), 200)
y_fit = poly(x_fit)

# 다항식 출력
print("2차 다항식:")
print(poly)

# 그래프 그리기
plt.figure()
plt.plot(weight, spring_constant, marker='o', linestyle='-', color='tab:blue', label='Data')
plt.plot(x_fit, y_fit, linestyle='--', color='tab:red', label='Quadratic Fit')
plt.xlabel('Weight (g)')
plt.ylabel('Spring Constant (N/m)')
plt.title('Spring Constant vs Weight with Quadratic Regression')
plt.legend()
plt.grid(True)
plt.show()