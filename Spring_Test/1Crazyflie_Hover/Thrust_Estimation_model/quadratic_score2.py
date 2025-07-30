import pandas as pd
import numpy as np
from sklearn.metrics import mean_squared_error, r2_score
import matplotlib.pyplot as plt

# 실험 데이터
data_text = """
PWM,Thrust
28166,39.98
28726,40.02
28907,40.03
28920,40.02
29071,40.03
36013,50.1
36064,50
36378,50.1
36436,51.79
36483,50.06
36567,51.9
36694,51.81
36725,50.12
36729,51.87
36997,51.83
37896,51.94
37925,51.97
38048,51.99
38387,51.92
38615,52.45
38672,51.99
38680,52.39
38691,52.44
38747,52.44
39177,52.42
39190,53.73
39368,53.69
39423,53.65
39593,53.54
39743,53.59
40097,55.4
40480,55.92
40689,55.42
40770,56.03
40843,56.06
40909,55.46
40985,55.59
41059,56.1
41107,55.45
41652,56.02
41814,57.27
41851,57.32
42062,57.31
42077,57.32
42097,57.34
42253,57.72
42519,57.67
42747,57.68
42846,57.72
42916,60.01
43083,60.1
43206,57.7
43409,60.23
45824,59.19
46425,59.2
47470,60.44
50187,60.51
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

