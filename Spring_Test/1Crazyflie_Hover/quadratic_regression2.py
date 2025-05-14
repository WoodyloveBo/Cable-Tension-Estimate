import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import re

# 실험 데이터
data_text = """
PWM	Thrust
28166   39.98
28726   40.02
28907   40.03
28920   40.02
29071   40.03
36013	50.1
36064	50
36378	50.1
36436	51.79
36483	50.06
36567	51.9
36694	51.81
36725	50.12
36729	51.87
36997	51.83
37896	51.94
37925	51.97
38048	51.99
38387	51.92
38615	52.45
38672	51.99
38680	52.39
38691	52.44
38747	52.44
39177	52.42
39190	53.73
39368	53.69
39423	53.65
39593	53.54
39743	53.59
40097	55.4
40480	55.92
40689	55.42
40770	56.03
40843	56.06
40909	55.46
40985	55.59
41059	56.1
41107	55.45
41652	56.02
41814	57.27
41851	57.32
42062	57.31
42077	57.32
42097	57.34
42253	57.72
42519	57.67
42747	57.68
42846	57.72
42916	60.01
43083	60.1
43206	57.7
43409	60.23
45824	59.19
46425	59.2
47470	60.44
50187	60.51
"""

# 데이터 파싱
lines = data_text.strip().split('\n')[1:]
pwm = []
thrust = []
for line in lines:
    cols = re.split(r'\s+', line)
    pwm.append(int(cols[0]))
    thrust.append(float(cols[1]))

df = pd.DataFrame({'PWM': pwm, 'Thrust_g': thrust})

# 2차 다항식 회귀 피팅
coeffs = np.polyfit(df['PWM'], df['Thrust_g'], 2)
poly_func = np.poly1d(coeffs)
df['Fitted_Thrust'] = poly_func(df['PWM'])

# 2차 다항 회귀 방정식 출력
print("2차 다항 회귀 방정식: Thrust_g = ({:.6e})*PWM^2 + ({:.6e})*PWM + ({:.6e})".format(
    coeffs[0], coeffs[1], coeffs[2]
))

# 그래프
plt.figure(figsize=(10, 6))
plt.scatter(df['PWM'], df['Thrust_g'], label='Measured Data', alpha=0.7)
plt.plot(df['PWM'], df['Fitted_Thrust'], label='Quadratic Fit Curve')
plt.scatter(df['PWM'], df['Fitted_Thrust'], label='Fitted Points', s=20, marker='x')  # 피팅 점 표시

plt.xlabel('PWM')
plt.ylabel('Thrust (g)')
plt.title('PWM to Thrust Mapping (Quadratic model)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
