bool isprocessed
uint32 now_checkpoint_index #현재 checkpoint index
bool is_published
bool is_final_path_publish #출발 이후 도착까지 경로 pub을 위한 
uint32 start_pose_index # 시작 pose index
uint32 goal_pose_index # 마지막 pose index
bool start_pose_arrive # 필요
bool goal_pose_arrive #필요
bool is_find_start_index