uint8 item_id
uint8 item_type
uint8 item_status
int32 score
uint8 speed
uint8 duration
int32 pos_lat
int32 pos_long
uint8 extend

########################################
####           ItemData 구조         ####
########################################

# item_id     : 아이템 ID
# item_type   : 아이템 종류
# 0x01 : 가점
# 0x02 : 감점
# 0x03 : 가속 주행

# item_status : 아이템 취득 여부
# 0x00 : 미습득
# 0x01 : 습득

# score       : 아이템 종류가 가점, 감점인 경우 반영될 점수 (sec)
# 0   : Unavailable (item_type이 가속 주행)
# < 0 : 감점 점수
# > 0 : 가점 점수

# speed       : 가속 시 최대 주행 가능한 속도값
# 0     : 정지
# 1~254 : 최대 주행 가능한 속도 (km/h)
# 255   : Unavailable (item_type이 랩타임 가/감산)

# duration    : 가속 시 지속 시간
# 0     : 지속시간 만료 (가/감속 시간 종료)
# 1~253 : 습득 시점부터 가/감속을 유지할 수 있는 잔여시간 또는 유지시간 (sec)
# 254   : 습득 시점부터 완주까지 최대 속도로 주행 가능한 상태
# 255   : Unavailable (item_type이 랩타임 가/감산)

# pos_lat     : 아이템 위치의 위도 (value : 10^-7 degree)
# pos_long    : 아이템 위치의 경도 (value : 10^-7 degree)
# extend      : 아이템 위치에 대한 취득 반경 (meter)