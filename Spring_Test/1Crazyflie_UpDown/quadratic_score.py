import pandas as pd
import numpy as np
from sklearn.metrics import mean_squared_error, r2_score
import matplotlib.pyplot as plt

# 실험 데이터
data_text = """
PWM,Thrust
36328,50.24
36647,50.87
37059,49.19
37309,50.91
37604,49.39
37640,50.72
37646,49.75
38419,51.127
38671,51.66
38798,51.471
38903,51.5
39134,51.78
39231,52.01
39627,55.61
39628,52.08
39827,53.5
40127,55.62
40136,53.58
40163,55.49
40221,54.84
40312,54.87
40403,55.72
40536,55.02
40613,55.38
40765,53.65
40821,53.15
41135,56.26
41414,56.3
41435,56.43
41518,56.44
42098,57.48
42163,57.66
42600,57.85
43213,57.96
43598,58.89
43990,58.89
44051,59.34
44106,60.09
44192,59.33
44256,59.94
44490,59.97
44566,59.86
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
