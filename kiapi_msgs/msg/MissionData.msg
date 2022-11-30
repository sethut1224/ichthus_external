uint8 mission_id
uint8 status
uint8 event_count
uint8 distance
uint8 route_id
int32 start_lat
int32 start_lon
int32 end_lat
int32 end_lon

#######################################
####        MissionData 구조        ####
#######################################

# mission_id  : 미션 ID
# status      : 선택 대기(0x00), 선택 완료(0x01)
# event_count : 이벤트 개수
# distance    : 미션의 예상 이동 거리
# route_id    : Mission Route 데이터 리스트 내 해당 미션에 대한 Mission Route ID
# start_lat   : 미션 출발지 위치의 위도 (value : 10^-7 degree)
# start_lon   : 미션 출발지 위치의 경도 (value : 10^-7 degree)
# end_lat     : 미션 도착지 위치의 위도 (value : 10^-7 degree)
# end_lon     : 미션 도착지 위치의 경도 (value : 10^-7 degree)