"""
대회장 실측값 모음 - 코스 답사하면서 GPS로 딴 좌표를 여기에만 적으면 됨.
다른 파일(각 mission_N.py의 로직)은 안 건드려도 됨.

값 넣는 방법: 배를 각 지점에 세워두고 gps_node1이 보여주는 위도/경도를
그대로 (lat, lon) 튜플로 넣으면 됨. 또는 GPS 웹맵(gps_web_map)에서
지도 클릭으로 찍은 좌표를 써도 됨.

네이밍 규칙: m{N}s = 미션N 시작좌표, m{N}e = 미션N 종료좌표.
m0s/m0e = 미션0(초기 장소 이동 전용, 순수 GPS 이동만).

미션별 흐름(팀 합의):
  MOVING: m{N}s로 이동
  TASK  : m{N}e를 기본/fallback 축으로 삼아 미션 수행
    - 통과형(1, 5): task 방향과 end 방향을 가중 평균
    - 완수형(2, 3, 4): 임무 완료 후에만 end로 이동
"""

# TODO: 실제 좌표(위도, 경도)로 채우기. (lat, lon) 튜플. None이면 미확정.
MISSION_TARGETS = {
    'm0s': None, 'm0e': None,   # 미션0: 출발지 -> 미션1 시작 전 장소이동
    'm1s': None, 'm1e': None,   # 미션1: 항로추종(게이트)
    'm2s': None, 'm2e': None,   # 미션2: 위치유지
    'm3s': None, 'm3e': None,   # 미션3: 도킹
    'm4s': None, 'm4e': None,   # 미션4: 탐색(선회)
    'm5s': None, 'm5e': None,   # 미션5: 항로추종(추가/예비)
}

# 미션3(도킹) 전용 - SLAM+GPS 결합으로 미리 딴 도크 슬롯 좌표 (좌/중/우)
DOCK_SECTORS = {
    'left': None,    # (lat, lon) TODO: SLAM 지도 + georef로 실측 후 채우기
    'center': None,
    'right': None,
}

# 미션별 목표 색/모양 지정 - 대회 당일 공지되면 여기만 고치면 됨
# 미션3(도킹): color+shape 둘 다 확인. 미션4(탐색): color만 확인(shape 불필요)
MISSION_TARGETS_CONFIG = {
    'mission_3': {'color': 'R', 'shape': 'triangle'},
    'mission_4': {'color': 'R'},
}
