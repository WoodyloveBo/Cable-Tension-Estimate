import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import re

# 실험 데이터
data_text = """
PWM	Thrust
40060 50.1
40236 52.22
40517 50.42
40773 52.53
40787 51.85
40822 49.765
40912 49.9
41122 51.19
41128 53.0
41350 51.42
41574 50.97
41768 51.23
41809 52.59
41897 52.39
41928 52.55
42406 53.32
42679 53.44
43014 53.87
43018 55.98
43028 55.86
43151 54.05
43325 55.14
43374 55.71
43408 53.79
44427 56.19
44446 57.05
44817 56.4
44830 55.99
44871 57.85
44889 57.139
44971 57.52
44999 56.98
45058 58.71
45064 57.34
45154 59.17
45268 56.83
45301 56.61
45353 57.82
45491 59.38
45645 59.98
45864 59.08
46211 58.84
46236 59.13
46406 58.69
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
