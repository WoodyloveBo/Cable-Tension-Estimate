#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import glob
import os
import shutil
import sys

import rosbag
from sensor_msgs.msg import PointCloud

def extract_point_index9_preserve(input_bag_path, output_bag_path):
    """
    input_bag_path의 모든 토픽을 그대로 복사하면서,
    /natnet_ros/pointcloud 토픽만 points[8]만 추출하여
    /point_index9 토픽으로 저장합니다.
    """
    total = 0
    converted = 0

    with rosbag.Bag(input_bag_path, 'r') as inbag, rosbag.Bag(output_bag_path, 'w') as outbag:
        for topic, msg, t in inbag.read_messages():
            total += 1
            if topic == '/natnet_ros/pointcloud':
                # 방어적으로 길이 확인
                if hasattr(msg, 'points') and len(msg.points) > 8:
                    new_msg = PointCloud()
                    new_msg.header = msg.header
                    # points[8] 하나만 유지
                    new_msg.points = [msg.points[8]]
                    # channels가 있다면 그대로 복사 (PointCloud는 channels가 옵션임)
                    if hasattr(msg, 'channels'):
                        new_msg.channels = msg.channels  # 필요 시 채널도 전달
                    outbag.write('/point_index9', new_msg, t)
                    converted += 1
                else:
                    # points가 부족하면 해당 메시지는 스킵하거나 그대로 복사할지 선택
                    # 여기서는 안전하게 "그대로 복사"합니다.
                    outbag.write(topic, msg, t)
            else:
                outbag.write(topic, msg, t)

    return total, converted


def main():
    parser = argparse.ArgumentParser(
        description="배치 처리: 22g_T?.bag 등 패턴에 매칭되는 모든 bag에서 pointcloud의 9번 포인트만 추출."
    )
    parser.add_argument(
        "-p", "--pattern",
        default="22g_T?.bag",
        help="입력 파일 패턴 (glob). 기본값: %(default)s",
    )
    parser.add_argument(
        "--suffix",
        default="_index9",
        help="출력 파일명에 붙일 접미사 (확장자 앞). 기본값: %(default)s",
    )
    parser.add_argument(
        "--inplace",
        action="store_true",
        help="원본을 덮어씁니다. 원본은 .bak 으로 백업 후 덮어씀.",
    )
    args = parser.parse_args()

    inputs = sorted(glob.glob(args.pattern))
    if not inputs:
        print(f"[!] 패턴에 매칭되는 파일이 없습니다: {args.pattern}")
        sys.exit(1)

    print(f"[*] 대상 파일 수: {len(inputs)}")
    for in_path in inputs:
        base, ext = os.path.splitext(in_path)
        out_path = f"{base}{args.suffix}{ext}"

        if args.inplace:
            # 원본 -> .bak 백업
            bak_path = f"{base}.bak{ext}"
            if not os.path.exists(bak_path):
                shutil.copy2(in_path, bak_path)
                print(f"  - 백업 생성: {bak_path}")
            # 임시 출력 파일에 작업 후 원본으로 교체
            tmp_out = f"{base}{args.suffix}.tmp{ext}"
            print(f"  > 변환(덮어쓰기): {in_path} -> {in_path} (임시: {tmp_out})")
            total, converted = extract_point_index9_preserve(in_path, tmp_out)
            # 교체
            os.replace(tmp_out, in_path)
        else:
            print(f"  > 변환: {in_path} -> {out_path}")
            total, converted = extract_point_index9_preserve(in_path, out_path)

        print(f"    메시지 총 {total}개, 변환된 /natnet_ros/pointcloud -> /point_index9 {converted}개")

    print("[✓] 배치 변환 완료")

if __name__ == "__main__":
    main()

