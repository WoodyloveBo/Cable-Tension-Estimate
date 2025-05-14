import pandas as pd
import numpy as np
from sklearn.metrics import mean_squared_error, r2_score
import matplotlib.pyplot as plt

# 실험 데이터
data_text = """
PWM,Thrust
40060,50.1
40236,52.22
40517,50.42
40773,52.53
40787,51.85
40822,49.765
40912,49.9
41122,51.19
41128,53.0
41350,51.42
41574,50.97
41768,51.23
41809,52.59
41897,52.39
41928,52.55
42406,53.32
42679,53.44
43014,53.87
43018,55.98
43028,55.86
43151,54.05
43325,55.14
43374,55.71
43408,53.79
44427,56.19
44446,57.05
44817,56.4
44830,55.99
44871,57.85
44889,57.139
44971,57.52
44999,56.98
45058,58.71
45064,57.34
45154,59.17
45268,56.83
45301,56.61
45353,57.82
45491,59.38
45645,59.98
45864,59.08
46211,58.84
46236,59.13
46406,58.69
"""

# 데이터프레임으로 변환
from io import StringIO
df = pd.read_csv(StringIO(data_text))

# 2차 다항 회귀 피팅
coeffs = np.polyfit(df['PWM'], df['Thrust'], deg=2)
poly2_func = np.poly1d(coeffs)
df['Fitted_Thrust'] = poly2_func(df['PWM'])

# RMSE 및 R² 계산
rmse = np.sqrt(mean_squared_error(df['Thrust'], df['Fitted_Thrust']))
r2 = r2_score(df['Thrust'], df['Fitted_Thrust'])

# 결과 출력
print(f"[2차 다항 회귀 결과]")
print(f"RMSE ≈ {rmse:.3f} g")
print(f"R² ≈ {r2:.3f}")
print(f"\n회귀식:\n{poly2_func}")

# 시각화
plt.figure(figsize=(10, 6))
plt.scatter(df['PWM'], df['Thrust'], label='Measured Thrust', color='blue', alpha=0.7)
plt.plot(df['PWM'], df['Fitted_Thrust'], label='2nd Degree Polynomial Fit', color='red')    
plt.xlabel('PWM')
plt.ylabel('Thrust (g)')
plt.title('2nd Degree Polynomial Fit (PWM → Thrust)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
