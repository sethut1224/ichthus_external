builtin_interfaces/Time stamp

int32 msg_type              # SPAT or MAP
int32 SPAT_MSG_TYPE = 2
int32 MAP_MSG_TYPE = 4

# SPaT
int64 spat_id_region        # 교차로 식별 고유 ID
int32 spat_movement_cnt     # 교차로 개수
string[] spat_movement_name # 교차로 명칭
int32[] spat_signal_group   # 메시지에 해당되는 차선 목록
int32[] spat_eventstate     # 신호 상태
int32[] spat_minendtime     # 예상 최소 종료 시간

# eventState
#0(점멸등, unavaliable)     : 알 수 없거나 오류에 사용됨.
#3(빨간불, stop and remain) : 정지선에서 차량을 멈춤.
#5(초록불, permissive_movement_allowed) : 모든 충돌 교통에 대해 신중히 대처해야 한다. 
#6(초록불, protected-Movement-Allowed)  : 지시된 방향으로 진행
#7(노란불, permissive-clearance) : 반드시 멈춰야 함.
#8(노란불, protected-clearance)  : 멈출 수 없을 경우 계속 진행 가능