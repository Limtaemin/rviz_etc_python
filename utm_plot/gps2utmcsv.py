#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pandas as pd
import utm

# CSV 파일 읽기 (위도, 경도, 고도)
df = pd.read_csv('왼쪽 차선.csv', header=0, names=['latitude', 'longitude', 'altitude'])

# 위도, 경도 값을 float으로 변환
df['latitude'] = df['latitude'].astype(float)
df['longitude'] = df['longitude'].astype(float)

# UTM 좌표로 변환
df['utm_x'], df['utm_y'], df['zone_number'], df['zone_letter'] = zip(*df.apply(lambda row: utm.from_latlon(row['latitude'], row['longitude']), axis=1))

# 변환된 UTM 좌표 출력
print(df[['utm_x', 'utm_y', 'altitude']])

# 필요한 경우 CSV로 저장
df[['utm_x', 'utm_y', 'altitude']].to_csv('utm_converted.csv', index=False)
