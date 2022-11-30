builtin_interfaces/Time stamp

uint8 mission_status
uint8 item_count

ItemData[] item_list

#######################################
####     MissionListStage2 구조     ####
#######################################

# mission_status : Stage2 미션 상태
# 0x00 : 미션 대기 (차량이 정차 또는 대기 및 정비 중인 경우, 시작하기 5초 전 미션 및 경로 정보 포함 전달)
# 0x01 : 미션 시작 (미션 선택 및 자율주행 진행)
# 0x02 : 미션 중단 (미션이 시작되었으나 중계 또는 관제로 인한 일시 중단)
# 0x03 : 미션 종료 (미션 완료 또는 실격으로 인한 종료)
# 0x03 : 테스트 모드

# item_count     : 메시지 내 포함된 item 개수