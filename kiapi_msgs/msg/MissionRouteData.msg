uint8 mission_route_id
uint8 route_node_total_count
uint8 route_node_index
uint8 route_node_type
int32 route_node_pos_lat
int32 route_node_pos_lon

########################################
####      MissionRouteData 구조      ####
########################################

# mission_route_id       : 주행 미션 경로 ID
# route_node_total_count : Route를 구성하는 Node의 전체 개수
# route_node_index       : Route를 구성하는 해당 Node의 Index 순번
# route_node_type        : Route 내 획득 Node의 종류
# 경로(0x00)
# 출발(0x01)
# 도착(0x02)

# route_node_pos_lat     : 미션 주행 경로의 Index 순번에 대한 Node 포인트의 위치의 위도 (value : 10^-7 degree)
# route_node_pos_lon     : 미션 주행 경로의 Index 순번에 대한 Node 포인트의 위치의 경도 (value : 10^-7 degree)